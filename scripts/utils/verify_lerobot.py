import sys
import os
import time
import numpy as np

# Add lerobot to path
lerobot_path = os.path.expanduser("~/lerobot/src")
if lerobot_path not in sys.path:
    sys.path.insert(0, lerobot_path)

try:
    from lerobot.common.robots.so101_follower import SO101Follower, SO101FollowerConfig
    print("Imported SO101Follower from lerobot.common.robots.so101_follower")
except ImportError:
    try:
        from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
        print("Imported SO101Follower from lerobot.robots.so101_follower")
    except ImportError as e:
        print(f"Failed to import SO101Follower: {e}")
        sys.exit(1)

def test_connection():
    port = "/dev/tty.usbmodem5B3E1224691"
    print(f"Connecting to {port} using standard SO101Follower...")
    
    try:
        # Configure with degrees for easier debugging
        config = SO101FollowerConfig(
            port=port,
            use_degrees=True,
            # calibration_dir='.cache/huggingface/lerobot/...' # Optional, relies on default if not set
        )
        
        robot = SO101Follower(config)
        robot.connect(calibrate=False)
        print("Connected!")
        
        # Read state
        print("Reading state...")
        # Robot.update() might be needed or just read property?
        # LeRobot robots usually have a data attribute or similar.
        # Check if 'teleop_step' or similar exists, or just access properties.
        
        # In LeRobot, usually we read from 'robot.state' or 'robot.capture_observation()'
        observation = robot.get_observation()
        print(f"Observation keys: {observation.keys()}")
        print(f"Shoulder Pan: {observation.get('shoulder_pan')}")
        
        print("Disconnecting...")
        robot.disconnect()
        print("Done.")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_connection()
