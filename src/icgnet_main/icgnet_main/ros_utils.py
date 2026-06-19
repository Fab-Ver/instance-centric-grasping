"""Shared ROS2 helpers used across icgnet_main nodes."""
import time

FUTURE_POLL_S = 0.02


def wait_for_future(future, timeout: float = 5.0) -> bool:
    """Block until `future` completes or `timeout` (wall-clock seconds) elapses."""
    deadline = time.time() + timeout
    while not future.done():
        if time.time() > deadline:
            return False
        time.sleep(FUTURE_POLL_S)
    return True


def read_workspace_bounds(node) -> dict:
    """Read workspace_{x,y,z}_{min,max} params into a {'x': (lo, hi), ...} dict.

    The six double parameters must already be declared on `node`.
    """
    def _val(name: str) -> float:
        return node.get_parameter(name).get_parameter_value().double_value

    return {
        axis: (_val(f'workspace_{axis}_min'), _val(f'workspace_{axis}_max'))
        for axis in ('x', 'y', 'z')
    }
