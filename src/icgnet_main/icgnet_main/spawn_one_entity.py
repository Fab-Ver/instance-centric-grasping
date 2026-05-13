import rclpy
from rclpy.node import Node
import sys
import subprocess
import os

def main():
    if len(sys.argv) < 3:
        print("Usage: ros2 run icgnet_main spawn_one_entity <model_name> <entity_name>")
        print("Example: ros2 run icgnet_main spawn_one_entity coke_can my_test_can")
        return

    model_name = sys.argv[1]
    entity_name = sys.argv[2]

    # Fixed safe position for debug
    x, y, z = 0.65, 0.0, 0.45

    cmd = [
        'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
        '-entity', entity_name,
        '-database', model_name,
        '-x', str(x), '-y', str(y), '-z', str(z)
    ]

    print(f"DEBUG SPAWNER: Spawning '{model_name}' as '{entity_name}' at (0.65, 0.0)...")
    
    try:
        # Run and show output immediately
        process = subprocess.Popen(cmd)
        process.wait()
        
        if process.returncode == 0:
            print(f"DEBUG SPAWNER: SUCCESS.")
        else:
            print(f"DEBUG SPAWNER: FAILED with code {process.returncode}")
    except Exception as e:
        print(f"DEBUG SPAWNER: EXCEPTION: {e}")

if __name__ == '__main__':
    main()
