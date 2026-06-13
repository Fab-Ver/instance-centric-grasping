#!/usr/bin/env python3
"""Phase 1 evaluation — single-object grasping.

Scope (2026-06-13): ICGNet mis-segments multi-object scenes (documented domain gap),
so evaluation runs on ONE object at a time, no distractors. Each run spawns a single
object of the target class, triggers a grasp, and logs the outcome plus the per-attempt
failure reason reported by grasp_executor.

Output (in report/, versioned — never overwrites a previous run):
  report/eval_<R>runs_<classes>_v<N>.csv          — one row per run
  report/eval_<R>runs_<classes>_v<N>_summary.txt  — aggregated metrics
where <R>=runs per class, <classes>=dash-joined class list, <N>=0 or last+1.

Each class maps to exactly one model in catalog.yaml, so target_class spawn is
deterministic (no per-run model variance to confound the metrics).

Usage:
  ./scripts/run_evaluation_phase1.py                      # 20 runs/class, all 6 classes
  ./scripts/run_evaluation_phase1.py --runs-per-class 10
  ./scripts/run_evaluation_phase1.py --classes can ball --runs-per-class 5
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
from icgnet_msgs.srv import ExecuteGrasp

MAX_ATTEMPTS = 5
TARGET_ENTITY = 'target_obj'


class EvaluatorPhase1(Node):
    def __init__(self):
        super().__init__('evaluator_phase1')
        self.client = self.create_client(ExecuteGrasp, '/icgnet/execute_grasp')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /icgnet/execute_grasp service...')

    def remove_target(self):
        """Delete the single object from gz-sim (bridged as DeleteEntity, type 2 = MODEL)."""
        subprocess.run([
            'ros2', 'service', 'call', '/world/icgnet_world/remove',
            'ros_gz_interfaces/srv/DeleteEntity',
            f'{{entity: {{name: "{TARGET_ENTITY}", type: 2}}}}'
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1.0)

    def spawn_object(self, target_class):
        """Spawn a single object of the given class (no distractors)."""
        self.get_logger().info(f'Spawning single object: target_class={target_class}')
        subprocess.run([
            'ros2', 'run', 'icgnet_main', 'spawn_object',
            '--ros-args', '-p', f'target_class:={target_class}'
        ])
        time.sleep(2.0)

    def execute_grasp(self, target_class, timeout_sec=1200.0):
        req = ExecuteGrasp.Request()
        req.target = target_class
        req.max_attempts = MAX_ATTEMPTS
        req.skip_place = False
        future = self.client.call_async(req)
        # Safety net: the executor bounds every move (move_timeout), so it should always
        # respond. This guards against an unforeseen wedge so one run can't freeze the batch.
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            self.get_logger().error(f'execute_grasp did not return within {timeout_sec:.0f}s')
            return None
        return future.result()


def write_summary(rows, summary_path):
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

    lines = ['=' * 60, 'PHASE 1 EVALUATION SUMMARY (single-object)', '=' * 60, '']

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

    text = '\n'.join(lines)
    with open(summary_path, 'w') as f:
        f.write(text + '\n')
    return text


def resolve_output_paths(report_dir, runs_per_class, classes):
    """Return (csv_path, summary_path) under report_dir, named by runs/classes/version.

    Base name: eval_<R>runs_<cls1-cls2-...>. The version is 0 if no file with that base
    exists yet, otherwise (highest existing version + 1), so a run never overwrites a
    previous result.
    """
    os.makedirs(report_dir, exist_ok=True)
    base = f"eval_{runs_per_class}runs_{'-'.join(classes)}"
    versions = []
    for p in glob.glob(os.path.join(report_dir, f"{base}_v*.csv")):
        m = re.search(rf"{re.escape(base)}_v(\d+)\.csv$", os.path.basename(p))
        if m:
            versions.append(int(m.group(1)))
    version = max(versions) + 1 if versions else 0
    stem = os.path.join(report_dir, f"{base}_v{version}")
    return f"{stem}.csv", f"{stem}_summary.txt"


def main():
    parser = argparse.ArgumentParser(description='Phase 1 single-object grasp evaluation')
    parser.add_argument('--runs-per-class', type=int, default=20,
                        help='number of runs per object class (default: 20)')
    parser.add_argument('--classes', nargs='+',
                        default=['mug', 'box', 'can', 'bottle', 'cylindric', 'ball'],
                        help='object classes to evaluate (default: all 6 catalog classes)')
    args = parser.parse_args()

    rclpy.init()
    node = EvaluatorPhase1()

    csv_path, summary_path = resolve_output_paths('report', args.runs_per_class, args.classes)
    node.get_logger().info(f'Writing results to {csv_path}')

    total_runs = len(args.classes) * args.runs_per_class
    rows = []

    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Run_ID', 'Target_Class', 'Detected_Classes', 'Success', 'Attempts',
                         'First_Attempt', 'Planning_Time', 'Execution_Time', 'Collision_Detected',
                         'Target_Not_Found', 'Failure_Reason', 'Attempt_Reasons'])

        run_id = 0
        for cls in args.classes:
            for _ in range(args.runs_per_class):
                run_id += 1
                node.get_logger().info(
                    f"========== RUN {run_id}/{total_runs}: Target={cls} ==========")

                node.remove_target()
                node.spawn_object(cls)

                res = node.execute_grasp(cls)

                if res:
                    success = bool(res.success)
                    attempts = res.grasps_attempted
                    first_attempt = 1 if (success and attempts == 1) else 0
                    attempt_reasons = list(res.attempt_reasons)
                    detected_classes = list(res.detected_classes)
                    failure_reason = res.failure_reason
                    row = {
                        'class': cls, 'success': success, 'attempts': attempts,
                        'failure_reason': failure_reason, 'attempt_reasons': attempt_reasons,
                    }
                    writer.writerow([
                        run_id, cls, ';'.join(detected_classes), 1 if success else 0, attempts,
                        first_attempt, round(res.planning_time, 2), round(res.execution_time, 2),
                        1 if res.collision_detected else 0, 1 if res.target_not_found else 0,
                        failure_reason, ';'.join(attempt_reasons),
                    ])
                    node.get_logger().info(
                        f"Run {run_id}: success={success} attempts={attempts} "
                        f"detected={detected_classes} "
                        f"failure_reason={failure_reason} attempts_log={attempt_reasons}")
                else:
                    row = {'class': cls, 'success': False, 'attempts': 0,
                           'failure_reason': 'SERVICE_NULL', 'attempt_reasons': []}
                    writer.writerow([run_id, cls, '', 0, 0, 0, 0.0, 0.0, 0, 0, 'SERVICE_NULL', ''])
                rows.append(row)
                f.flush()

    summary = write_summary(rows, summary_path)
    node.get_logger().info(f'\n{summary}')
    node.get_logger().info(f'Results: {csv_path}  |  Summary: {summary_path}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
