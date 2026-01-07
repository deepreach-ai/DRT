"""
SO-ARM Backend using LeRobot SO-101 Follower - FIXED VERSION
Key changes:
1. Adjusted scaling factor from 500 to 50 (10x more sensitive)
2. Better debug logging
3. Fixed delta accumulation issue
"""
from typing import Tuple, Optional, Dict, Any
import numpy as np
import time
import threading
import sys
import os

# Add lerobot to path if needed
lerobot_path = os.path.expanduser("~/lerobot/src")
if lerobot_path not in sys.path:
    sys.path.insert(0, lerobot_path)

from robot_backend import RobotBackend, BackendStatus

# Import OpenCV
try:
    import cv2
    OPENCV_AVAILABLE = True
except ImportError:
    cv2 = None
    OPENCV_AVAILABLE = False

# Import RealSense
try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    rs = None
    REALSENSE_AVAILABLE = False

# Import LeRobot SO-101
try:
    from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
    from lerobot.robots.robot import Robot
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.cameras.utils import make_cameras_from_configs
    LEROBOT_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Could not import LeRobot: {e}")
    LEROBOT_AVAILABLE = False


class SafeFeetechMotorsBus(FeetechMotorsBus):
    """
    A safer version of FeetechMotorsBus that handles multi-turn positions 
    gracefully during calibration.
    """
    def _get_half_turn_homings(self, positions: Dict[Any, Any]) -> Dict[Any, Any]:
        """
        Override to handle multi-turn positions by using modulo.
        """
        half_turn_homings = {}
        for motor, pos in positions.items():
            model = self._get_motor_model(motor)
            max_res = self.model_resolution_table[model] - 1
            resolution = max_res + 1
            
            pos_mod = pos % resolution
            target_center = int(max_res / 2)
            offset = pos_mod - target_center
            
            max_offset = 2047
            if offset > max_offset:
                offset = max_offset
            elif offset < -max_offset:
                offset = -max_offset
                
            half_turn_homings[motor] = offset
            
        return half_turn_homings


class FlexibleSO101Follower(SO101Follower):
    """
    A subclass of SO101Follower that dynamically detects available motors
    and initializes with only the connected ones.
    """
    def __init__(self, config: SO101FollowerConfig):
        Robot.__init__(self, config)
        self.config = config
        
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        all_motors = {
            "shoulder_pan": Motor(1, "sts3215", norm_mode_body),
            "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
            "elbow_flex": Motor(3, "sts3215", norm_mode_body),
            "wrist_flex": Motor(4, "sts3215", norm_mode_body),
            "wrist_roll": Motor(5, "sts3215", norm_mode_body),
            "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
        }
        
        print(f"[FlexibleSO101Follower] Scanning for motors on {config.port}...")
        found_motors = {}
        
        try:
            temp_bus = SafeFeetechMotorsBus(port=config.port, motors={})
            temp_bus.connect(handshake=False)
            
            for name, motor in all_motors.items():
                try:
                    model = temp_bus.ping(motor.id)
                    if model is not None:
                        found_motors[name] = motor
                    else:
                        print(f"  - Missing {name} (ID {motor.id})")
                except Exception:
                    print(f"  - Error checking {name} (ID {motor.id})")
            
            temp_bus.disconnect()
            
        except Exception as e:
            print(f"[FlexibleSO101Follower] Scan failed: {e}")
            found_motors = all_motors

        if not found_motors:
            print("[FlexibleSO101Follower] No motors found! Trying full list.")
            found_motors = all_motors
        else:
            print(f"[FlexibleSO101Follower] Initializing with {len(found_motors)}/{len(all_motors)} motors")

        if self.calibration:
            filtered_calibration = {k: v for k, v in self.calibration.items() if k in found_motors}
            if len(filtered_calibration) != len(self.calibration):
                print(f"[FlexibleSO101Follower] Filtered calibration from {len(self.calibration)} to {len(filtered_calibration)} motors")
            self.calibration = filtered_calibration

        self.bus = SafeFeetechMotorsBus(
            port=self.config.port,
            motors=found_motors,
            calibration=self.calibration,
        )
        self.cameras = make_cameras_from_configs(config.cameras)


class SOARMBackend(RobotBackend):
    """Backend for SO-ARM100/SO-101 robot using LeRobot - FIXED VERSION"""
    
    def __init__(self, port: str = "/dev/tty.usbmodem5B3E1187881", name: str = "so_arm"):
        super().__init__(name)
        self.port = port
        self.robot = None
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_orientation = np.array([1.0, 0.0, 0.0, 0.0])
        self.command_count = 0
        
        # Camera state
        self.cameras = {}
        self.camera_started = False
        
        # Debug counters
        self.zero_command_count = 0
        self.nonzero_command_count = 0
        
        if not LEROBOT_AVAILABLE:
            raise RuntimeError("LeRobot is not available. Please install it first.")
    
    def connect(self) -> bool:
        """Connect to SO-ARM robot and Camera"""
        try:
            print(f"[SOARMBackend] Connecting to SO-ARM at {self.port}...")
            self.status = BackendStatus.CONNECTING
            
            config = SO101FollowerConfig(
                port=self.port,
                disable_torque_on_disconnect=True,
                use_degrees=False
            )
            
            self.robot = FlexibleSO101Follower(config)
            self.robot.connect(calibrate=False)  # FIXED: Skip interactive calibration
            print(f"[SOARMBackend] ✓ Connected to SO-ARM")

            # Skip camera initialization for now
            self.cameras = {}
            self.camera_started = False
            
            self.status = BackendStatus.CONNECTED
            self.last_update_time = time.time()
            return True
            
        except Exception as e:
            print(f"[SOARMBackend] ✗ Connection failed: {e}")
            import traceback
            traceback.print_exc()
            self.status = BackendStatus.ERROR
            return False
    
    def disconnect(self):
        """Disconnect from SO-ARM robot"""
        try:
            print(f"[SOARMBackend] Disconnecting...")
            
            if self.robot is not None:
                self.robot.disconnect()
                self.robot = None
                print("[SOARMBackend] Robot disconnected")
            
            self.status = BackendStatus.DISCONNECTED
            print(f"[SOARMBackend] ✓ Disconnected")
            print(f"[SOARMBackend] Stats: {self.nonzero_command_count} movement commands, {self.zero_command_count} zero commands")
        except Exception as e:
            print(f"[SOARMBackend] Error during disconnect: {e}")
            self.status = BackendStatus.ERROR
    
    def render(self, width: int = 960, height: int = 540) -> Optional[bytes]:
        """Render camera view - not implemented"""
        return None

    def send_target_pose(self, position: np.ndarray, 
                         orientation: np.ndarray,
                         velocity_limit: float = 0.1,
                         gripper_state: float = -1.0) -> bool:
        """
        Send target pose to SO-ARM
        FIXED: Better scaling and debug logging
        """
        if not self.is_connected() or self.robot is None:
            print("[SOARMBackend] Not connected!")
            return False
        
        try:
            with self.update_lock:
                current_joints = self.get_current_joints()
                if current_joints is None:
                    return False
                
                # Calculate delta from last position
                delta_pos = position - self.current_position
                
                # Update internal state
                self.current_position = position.copy()
                self.current_orientation = orientation.copy()
                
                # FIXED: Lower threshold to detect more movements
                delta_magnitude = np.linalg.norm(delta_pos)
                if delta_magnitude < 0.0001 and gripper_state == -1.0:
                    self.zero_command_count += 1
                    return True  # Silent success for zero commands
                
                self.nonzero_command_count += 1
                
                # FIXED: Adjusted scale factor - was 500, now 50 (10x more sensitive)
                # With keyboard increment of 0.02m, this gives 1 unit = 0.02m * 50 = 1.0 position units
                pos_scale = 50.0  
                
                d_j1 = delta_pos[0] * pos_scale
                d_j2 = delta_pos[1] * pos_scale 
                d_j3 = delta_pos[2] * pos_scale
                
                new_joints = current_joints.copy()
                
                if len(new_joints) >= 1: new_joints[0] += d_j1
                if len(new_joints) >= 2: new_joints[1] += d_j2
                if len(new_joints) >= 3: new_joints[2] += d_j3
                
                # More verbose logging
                print(f"[SOARMBackend] CMD #{self.nonzero_command_count}: Delta={delta_pos} (mag={delta_magnitude:.4f})")
                print(f"[SOARMBackend]   Joint deltas: [{d_j1:.1f}, {d_j2:.1f}, {d_j3:.1f}]")
                print(f"[SOARMBackend]   New joints: {new_joints[:3]}")

                return self.send_joint_positions(new_joints)
                
        except Exception as e:
            print(f"[SOARMBackend] Error sending command: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def send_joint_positions(self, joint_positions: np.ndarray) -> bool:
        """Direct joint control"""
        if not self.is_connected() or self.robot is None:
            return False
        
        try:
            with self.update_lock:
                sorted_motors = sorted(self.robot.bus.motors.items(), key=lambda x: x[1].id)
                
                if len(joint_positions) > len(sorted_motors):
                    joint_positions = joint_positions[:len(sorted_motors)]
                elif len(joint_positions) < len(sorted_motors):
                    print(f"[SOARMBackend] Error: joint count mismatch. Expected {len(sorted_motors)}, got {len(joint_positions)}")
                    return False
                
                target_dict = {name: int(val) for (name, _), val in zip(sorted_motors, joint_positions)}
                
                self.robot.bus.sync_write("Goal_Position", target_dict)
                
                self.command_count += 1
                self.last_update_time = time.time()
                print(f"[SOARMBackend]   ✓ Sent to motors: {target_dict}")
                return True
        except Exception as e:
            print(f"[SOARMBackend] Error sending joint command: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def get_current_pose(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Get current robot pose"""
        if not self.is_connected() or self.robot is None:
            return None, None
        
        try:
            with self.update_lock:
                if self.robot and self.robot.bus:
                    _ = self.robot.bus.sync_read("Present_Position")
                
                return self.current_position.copy(), self.current_orientation.copy()
                
        except Exception as e:
            # print(f"[SOARMBackend] Error reading pose: {e}")
            return None, None
    
    def get_current_joints(self) -> Optional[np.ndarray]:
        """Get current joint positions"""
        if not self.is_connected() or self.robot is None:
            return None
        
        try:
            joints_dict = self.robot.bus.sync_read("Present_Position")
            sorted_motors = sorted(self.robot.bus.motors.items(), key=lambda x: x[1].id)
            joint_values = [joints_dict[name] for name, _ in sorted_motors]
            
            return np.array(joint_values)
        except Exception as e:
            # Suppress spam
            return None
    
    def get_status(self) -> Dict[str, Any]:
        """Get SO-ARM backend status"""
        status = {
            'name': self.name,
            'status': self.status.value,
            'port': self.port,
            'command_count': self.command_count,
            'nonzero_commands': self.nonzero_command_count,
            'zero_commands': self.zero_command_count,
            'last_update': self.last_update_time,
            'lerobot_available': LEROBOT_AVAILABLE,
        }
        
        if self.is_connected() and self.robot is not None:
            try:
                joints = self.get_current_joints()
                if joints is not None:
                    status['current_joints'] = joints.tolist()
            except:
                pass
        
        return status
