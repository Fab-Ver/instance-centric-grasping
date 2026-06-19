#!/usr/bin/env python3
"""Single-object grasp evaluation.

ICGNet mis-segments multi-object scenes (a documented domain gap), so evaluation
runs on ONE object at a time, no distractors. Each run spawns a single
object of the target class, triggers a grasp, and logs the outcome plus the per-attempt
failure reason reported by grasp_executor.

Output (in results/tests/, versioned — never overwrites a previous run):
  results/tests/eval_<R>runs_<classes>_v<N>.csv          — one row per run
  results/tests/eval_<R>runs_<classes>_v<N>_summary.txt  — aggregated metrics
where <R>=runs per class, <classes>=dash-joined class list, <N>=0 or last+1.

Each class maps to exactly one model in catalog.yaml, so target_class spawn is
deterministic (no per-run model variance to confound the metrics).

Usage:
  ./scripts/run_evaluation.py                      # 20 runs/class, all 6 classes
  ./scripts/run_evaluation.py --runs-per-class 10
  ./scripts/run_evaluation.py --classes can ball --runs-per-class 5
"""
import os
import re
import csv
import glob
import time
import argparse
import subprocess
from collections import Counter, defaultdict

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from icgnet_msgs.srv import ExecuteGrasp

MAX_ATTEMPTS = 5
TARGET_ENTITY = 'target_obj'
ENTITY_WAIT_TIMEOUT = 8.0   # max wall-time to confirm a remove/spawn took effect in gz
RESET_MAX_TRIES = 3         # remove+spawn cycles before giving up on a run


class Evaluator(Node):
    def __init__(self):
        super().__init__('grasp_evaluator')
        self.client = self.create_client(ExecuteGrasp, '/icgnet/execute_grasp')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /icgnet/execute_grasp service...')
        # Track which dynamic entities are currently in the scene. gz publishes the pose of
        # every non-static model each step on dynamic_pose/info (remapped to /model_poses),
        # so an entity's presence/absence here is the ground truth for remove/spawn success.
        self._present_entities: set[str] = set()
        self.create_subscription(TFMessage, '/model_poses', self._poses_cb, 10)

    def _poses_cb(self, msg: TFMessage):
        self._present_entities = {t.child_frame_id for t in msg.transforms}

    def _spin(self, duration):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_for_entity(self, name, present, timeout=ENTITY_WAIT_TIMEOUT):
        """Block until `name` is present (present=True) or absent (present=False) in gz."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.2)
            if (name in self._present_entities) == present:
                return True
        return False

    def remove_target(self):
        """Delete the single object and confirm it is gone. Returns True when absent."""
        proc = subprocess.run([
            'ros2', 'service', 'call', '/world/icgnet_world/remove',
            'ros_gz_interfaces/srv/DeleteEntity',
            f'{{entity: {{name: "{TARGET_ENTITY}", type: 2}}}}'
        ], capture_output=True, text=True)
        if self._wait_for_entity(TARGET_ENTITY, present=False):
            return True
        self.get_logger().warn(
            f'{TARGET_ENTITY} still present {ENTITY_WAIT_TIMEOUT:.0f}s after remove; '
            f'service output: {proc.stdout.strip()[-200:]}')
        return False

    def spawn_object(self, target_class):
        """Spawn a single object of the class and confirm it appears. Returns True on success."""
        self.get_logger().info(f'Spawning single object: target_class={target_class}')
        proc = subprocess.run([
            'ros2', 'run', 'icgnet_main', 'spawn_object',
            '--ros-args', '-p', f'target_class:={target_class}',
            # gz-sim server is already up for the whole batch and presence is verified below,
            # so skip the cold-start wait (~5s/run saved).
            '-p', 'gz_server_wait:=0.0'
        ], capture_output=True, text=True)
        if proc.returncode != 0:
            self.get_logger().warn(
                f'spawn_object exited {proc.returncode} (likely name collision with a stale '
                f'entity): {proc.stdout.strip()[-200:]}')
            return False
        if not self._wait_for_entity(TARGET_ENTITY, present=True):
            self.get_logger().warn(f'{TARGET_ENTITY} not visible {ENTITY_WAIT_TIMEOUT:.0f}s after spawn')
            return False
        return True

    def reset_target(self, target_class):
        """Remove the old object and spawn a fresh one, retrying on failure.

        Breaks the stale-scene cascade: a failed remove leaves the previous object in place,
        so the next spawn hits a name collision; we detect it (non-zero exit) and retry the
        full remove+spawn cycle. Returns True only when a fresh object is confirmed in scene.
        """
        for attempt in range(1, RESET_MAX_TRIES + 1):
            self.remove_target()
            if self.spawn_object(target_class):
                return True
            self.get_logger().warn(
                f'reset_target {target_class}: attempt {attempt}/{RESET_MAX_TRIES} failed, retrying')
            time.sleep(1.0)
        return False

    def execute_grasp(self, target_class, run_id, inference_dump, grasping_dump, timeout_sec=1200.0):
        req = ExecuteGrasp.Request()
        req.target = target_class
        req.max_attempts = MAX_ATTEMPTS
        req.skip_place = False
        req.run_id = run_id
        req.inference_dump_path = inference_dump
        req.grasping_dump_path = grasping_dump
        future = self.client.call_async(req)
        # Safety net: the executor bounds every move (move_timeout), so it should always
        # respond. This guards against an unforeseen wedge so one run can't freeze the batch.
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            self.get_logger().error(f'execute_grasp did not return within {timeout_sec:.0f}s')
            return None
        return future.result()


def write_summary(rows, summary_path, mode):
    """Aggregate the per-run rows into GSR, avg-attempts-to-success and failure histograms."""
    by_class = defaultdict(list)
    for r in rows:
        by_class[r['class']].append(r)

    run_failures = Counter()       # run-level: failure_reason of failed runs
    attempt_failures = Counter()   # attempt-level: every non-SUCCESS attempt code
    for r in rows:
        if not r['success']:
            run_failures[r['failure_reason'] or 'UNKNOWN'] += 1
        for code in r['attempt_reasons']:
            if code != 'SUCCESS':
                attempt_failures[code] += 1

    grasp_target = 'any (class-agnostic)' if mode == 'any' else 'spawned class (target-driven)'
    lines = ['=' * 60, f'EVALUATION SUMMARY (single-object, mode={mode})',
             f'Grasp target = {grasp_target}', '=' * 60, '']

    total_succ = sum(1 for r in rows if r['success'])
    lines.append(f'Overall GSR: {total_succ}/{len(rows)} = {100.0 * total_succ / max(len(rows), 1):.1f}%')
    lines.append('')
    lines.append('Per-class GSR:')
    for cls, runs in by_class.items():
        succ = sum(1 for r in runs if r['success'])
        lines.append(f'  {cls:10s} {succ}/{len(runs)} = {100.0 * succ / len(runs):.1f}%')
    lines.append('')

    succ_attempts = [r['attempts'] for r in rows if r['success']]
    if succ_attempts:
        avg = sum(succ_attempts) / len(succ_attempts)
        first = sum(1 for a in succ_attempts if a == 1)
        lines.append(f'Avg attempts until success (successful runs): {avg:.2f}')
        lines.append(f'First-attempt successes: {first}/{len(succ_attempts)} '
                     f'({100.0 * first / len(succ_attempts):.1f}% of successful runs)')
    else:
        lines.append('Avg attempts until success: n/a (no successful runs)')
    lines.append('')

    lines.append('Failed runs by reason (run-level):')
    for code, n in run_failures.most_common():
        lines.append(f'  {n:4d} × {code}')
    lines.append('')

    lines.append('Failed attempts by reason (attempt-level, includes retries on successful runs):')
    for code, n in attempt_failures.most_common():
        lines.append(f'  {n:4d} × {code}')
    lines.append('')

    # Grasp-proposal scores: pair each attempt's ICGNet score with its outcome to gauge
    # whether higher-scored proposals actually correlate with success.
    def _stats(vals):
        return (f'mean={sum(vals) / len(vals):.3f} min={min(vals):.3f} max={max(vals):.3f} '
                f'(n={len(vals)})') if vals else 'n/a'

    succ_scores, fail_scores = [], []
    for r in rows:
        for code, sc in zip(r['attempt_reasons'], r['attempt_scores']):
            (succ_scores if code == 'SUCCESS' else fail_scores).append(sc)
    lines.append('Grasp-proposal scores by attempt outcome:')
    lines.append(f'  SUCCESS attempts: {_stats(succ_scores)}')
    lines.append(f'  FAILED  attempts: {_stats(fail_scores)}')
    lines.append('')

    lines.append('Best-proposal score per class (top candidate, attempt 1):')
    for cls, runs in by_class.items():
        best = [r['attempt_scores'][0] for r in runs if r['attempt_scores']]
        lines.append(f'  {cls:10s} {_stats(best)}')
    lines.append('')

    text = '\n'.join(lines)
    with open(summary_path, 'w') as f:
        f.write(text + '\n')
    return text


def resolve_output_paths(report_dir, runs_per_class, classes, mode):
    """Return output paths under report_dir, named by runs/classes/mode/version.

    Returns (csv, summary, inference_dump, grasping_dump). The eval CSV uses the
    eval_<R>runs_<cls...>_<mode> base; the JSONL dumps share the same
    <R>runs_<cls...>_<mode>_v<N> suffix with an inference_/grasping_ prefix, so the
    three files line up by Run_ID. The <mode> tag ('target' or 'any') keeps the two
    experiments' outputs in separate files. The version is 0 if no eval CSV with that
    base exists yet, otherwise (highest + 1), so a run never overwrites a previous result.
    """
    os.makedirs(report_dir, exist_ok=True)
    suffix = f"{runs_per_class}runs_{'-'.join(classes)}_{mode}"
    base = f"eval_{suffix}"
    versions = []
    for p in glob.glob(os.path.join(report_dir, f"{base}_v*.csv")):
        m = re.search(rf"{re.escape(base)}_v(\d+)\.csv$", os.path.basename(p))
        if m:
            versions.append(int(m.group(1)))
    version = max(versions) + 1 if versions else 0
    stem = os.path.join(report_dir, f"{base}_v{version}")
    inference_dump = os.path.join(report_dir, f"inference_{suffix}_v{version}.jsonl")
    grasping_dump = os.path.join(report_dir, f"grasping_{suffix}_v{version}.jsonl")
    return f"{stem}.csv", f"{stem}_summary.txt", inference_dump, grasping_dump


def main():
    parser = argparse.ArgumentParser(description='Single-object grasp evaluation')
    parser.add_argument('--runs-per-class', type=int, default=20,
                        help='number of runs per object class (default: 20)')
    parser.add_argument('--classes', nargs='+',
                        default=['mug', 'box', 'can', 'bottle', 'cylindric', 'ball'],
                        help='object classes to evaluate (default: all 6 catalog classes)')
    parser.add_argument('--mode', choices=['target', 'any'], default='target',
                        help="grasp target: 'target' = the spawned class (target-driven, "
                             "needs correct ICGNet classification); 'any' = class-agnostic "
                             "(isolates segmentation+grasp from the classification step). "
                             "Outputs go to separate _<mode>_ files (default: target)")
    args = parser.parse_args()

    rclpy.init()
    node = Evaluator()

    csv_path, summary_path, inference_dump, grasping_dump = resolve_output_paths(
        'results/tests', args.runs_per_class, args.classes, args.mode)
    node.get_logger().info(
        f'Writing results to {csv_path}\n  inference dump: {inference_dump}\n'
        f'  grasping dump:  {grasping_dump}')

    total_runs = len(args.classes) * args.runs_per_class
    rows = []

    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Run_ID', 'Target_Class', 'Detected_Classes', 'Success', 'Attempts',
                         'First_Attempt', 'Planning_Time', 'Execution_Time', 'Collision_Detected',
                         'Target_Not_Found', 'Failure_Reason', 'Attempt_Reasons', 'Attempt_Scores'])

        run_id = 0
        for cls in args.classes:
            for _ in range(args.runs_per_class):
                run_id += 1
                node.get_logger().info(
                    f"========== RUN {run_id}/{total_runs}: Target={cls} ==========")

                if not node.reset_target(cls):
                    node.get_logger().error(
                        f"Run {run_id}: could not place a fresh '{cls}' after "
                        f"{RESET_MAX_TRIES} tries — logging SPAWN_FAIL, skipping grasp")
                    writer.writerow([run_id, cls, '', 0, 0, 0, 0.0, 0.0, 0, 0,
                                     'SPAWN_FAIL', '', ''])
                    rows.append({'class': cls, 'success': False, 'attempts': 0,
                                 'failure_reason': 'SPAWN_FAIL', 'attempt_reasons': [],
                                 'attempt_scores': []})
                    f.flush()
                    continue

                # mode='target': require the spawned class; mode='any': accept any grasp on
                # the (single) spawned object, removing the classification requirement.
                grasp_target = 'any' if args.mode == 'any' else cls
                res = node.execute_grasp(grasp_target, run_id, inference_dump, grasping_dump)

                if res:
                    success = bool(res.success)
                    attempts = res.grasps_attempted
                    first_attempt = 1 if (success and attempts == 1) else 0
                    attempt_reasons = list(res.attempt_reasons)
                    attempt_scores = list(res.attempt_scores)
                    detected_classes = list(res.detected_classes)
                    failure_reason = res.failure_reason
                    row = {
                        'class': cls, 'success': success, 'attempts': attempts,
                        'failure_reason': failure_reason, 'attempt_reasons': attempt_reasons,
                        'attempt_scores': attempt_scores,
                    }
                    writer.writerow([
                        run_id, cls, ';'.join(detected_classes), 1 if success else 0, attempts,
                        first_attempt, round(res.planning_time, 2), round(res.execution_time, 2),
                        1 if res.collision_detected else 0, 1 if res.target_not_found else 0,
                        failure_reason, ';'.join(attempt_reasons),
                        ';'.join(f"{s:.3f}" for s in attempt_scores),
                    ])
                    node.get_logger().info(
                        f"Run {run_id}: success={success} attempts={attempts} "
                        f"detected={detected_classes} "
                        f"failure_reason={failure_reason} attempts_log={attempt_reasons}")
                else:
                    row = {'class': cls, 'success': False, 'attempts': 0,
                           'failure_reason': 'SERVICE_NULL', 'attempt_reasons': [],
                           'attempt_scores': []}
                    writer.writerow([run_id, cls, '', 0, 0, 0, 0.0, 0.0, 0, 0, 'SERVICE_NULL', '', ''])
                rows.append(row)
                f.flush()

    summary = write_summary(rows, summary_path, args.mode)
    node.get_logger().info(f'\n{summary}')
    node.get_logger().info(f'Results: {csv_path}  |  Summary: {summary_path}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
