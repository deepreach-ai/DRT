"""
SO-ARM Backend using LeRobot SO-101 Follower
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
        Original implementation fails if position is outside single-turn range 
        because calculated offset exceeds 12-bit signed limit.
        """
        half_turn_homings = {}
        for motor, pos in positions.items():
            model = self._get_motor_model(motor)
            max_res = self.model_resolution_table[model] - 1
            resolution = max_res + 1
            
            # Apply modulo to handle multi-turn (e.g. 32459 -> 3787)
            pos_mod = pos % resolution
            
            # Calculate offset to center the range
            # We want: (pos_mod - offset) approx equals (max_res / 2)
            target_center = int(max_res / 2)
            offset = pos_mod - target_center
            
            # Ensure offset is within valid range [-2047, 2047] for 12-bit signed magnitude
            # (STS3215 Homing Offset is limited to this range)
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
        # Initialize Robot base (skipping SO101Follower.__init__ to avoid strict check)
        Robot.__init__(self, config)
        self.config = config
        
        # Define all expected motors (copied from SO101Follower)
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
            # Create a temporary bus to scan
            # We pass empty motors dict to avoid immediate validation
            temp_bus = SafeFeetechMotorsBus(port=config.port, motors={})
            temp_bus.connect(handshake=False)
            
            # Ping each expected ID
            for name, motor in all_motors.items():
                try:
                    # Ping returns model number if found, None otherwise
                    model = temp_bus.ping(motor.id)
                    if model is not None:
                        found_motors[name] = motor
                        # print(f"  - Found {name} (ID {motor.id})")
                    else:
                        print(f"  - Missing {name} (ID {motor.id})")
                except Exception:
                    print(f"  - Error checking {name} (ID {motor.id})")
            
            temp_bus.disconnect()
            
        except Exception as e:
            print(f"[FlexibleSO101Follower] Scan failed: {e}")
            # If scan fails completely, fall back to full list (will likely fail again in main init)
            found_motors = all_motors

        if not found_motors:
            print("[FlexibleSO101Follower] No motors found! Trying full list.")
            found_motors = all_motors
        else:
            print(f"[FlexibleSO101Follower] Initializing with {len(found_motors)}/{len(all_motors)} motors")

        # Filter calibration to match found motors
        if self.calibration:
            filtered_calibration = {k: v for k, v in self.calibration.items() if k in found_motors}
            if len(filtered_calibration) != len(self.calibration):
                print(f"[FlexibleSO101Follower] Filtered calibration from {len(self.calibration)} to {len(filtered_calibration)} motors")
            self.calibration = filtered_calibration

        # Initialize the real bus with found motors
        self.bus = SafeFeetechMotorsBus(
            port=self.config.port,
            motors=found_motors,
            calibration=self.calibration,
        )
        self.cameras = make_cameras_from_configs(config.cameras)


class SOARMBackend(RobotBackend):
    """Backend for SO-ARM100/SO-101 robot using LeRobot"""
    
    def __init__(self, port: str = "/dev/tty.usbmodem5B3E1187881", name: str = "so_arm"):
        super().__init__(name)
        self.port = port
        self.robot = None
        self.current_position = np.array([0.0, 0.0, 0.0])  # Will be updated from robot
        self.current_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # quat [w,x,y,z]
        self.command_count = 0
        
        # Camera state
        self.pipeline = None
        self.camera_config = None
        self.camera_started = False
        
        if not LEROBOT_AVAILABLE:
            raise RuntimeError("LeRobot is not available. Please install it first.")
    
    def connect(self) -> bool:
        """Connect to SO-ARM robot and Camera"""
        try:
            print(f"[SOARMBackend] Connecting to SO-ARM at {self.port}...")
            self.status = BackendStatus.CONNECTING
            
            # 1. Connect to Robot
            # Create robot config
            config = SO101FollowerConfig(
                port=self.port,
                disable_torque_on_disconnect=True,
                use_degrees=False
            )
            
            # Initialize robot
            self.robot = FlexibleSO101Follower(config)
            self.robot.connect()
            print(f"[SOARMBackend] ✓ Connected to SO-ARM")

            # 2. Connect to Camera
            # Prioritize RealSense, fallback to OpenCV webcam if RealSense fails
            self.camera_type = "none"
            
            # Try RealSense first
            # WARNING: pyrealsense2 on macOS arm64 is unstable and causes segfaults
            # We will default to OpenCV for stability unless explicitly forced
            USE_REALSENSE = False # Set to True only if you are sure it works
            
            if REALSENSE_AVAILABLE and USE_REALSENSE:
                try:
                    print("[SOARMBackend] Initializing RealSense camera...")
                    self.pipeline = rs.pipeline()
                    self.camera_config = rs.config()
                    self.camera_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                    self.pipeline.start(self.camera_config)
                    self.camera_started = True
                    self.camera_type = "realsense"
                    print("[SOARMBackend] ✓ RealSense camera started")
                except Exception as e:
                    print(f"[SOARMBackend] ⚠ Warning: Failed to start RealSense: {e}")
                    self.pipeline = None
            else:
                if REALSENSE_AVAILABLE:
                    print("[SOARMBackend] ℹ RealSense disabled for stability (segfault protection). Using OpenCV fallback.")
            
            # If RealSense failed, try OpenCV webcam
            if not self.camera_started and OPENCV_AVAILABLE:
                try:
                    print("[SOARMBackend] Trying fallback to OpenCV webcam (ID 0)...")
                    self.cap = cv2.VideoCapture(0)
                    if self.cap.isOpened():
                        self.camera_started = True
                        self.camera_type = "opencv"
                        print("[SOARMBackend] ✓ OpenCV webcam started")
                    else:
                        print("[SOARMBackend] ⚠ Failed to open webcam 0")
                except Exception as e:
                    print(f"[SOARMBackend] ⚠ Error opening webcam: {e}")
            
            self.status = BackendStatus.CONNECTED
            self.last_update_time = time.time()
            return True
            
        except Exception as e:
            print(f"[SOARMBackend] ✗ Connection failed: {e}")
            self.status = BackendStatus.ERROR
            return False
    
    def disconnect(self):
        """Disconnect from SO-ARM robot and Camera"""
        try:
            print(f"[SOARMBackend] Disconnecting...")
            
            # Stop Camera
            if self.camera_started:
                if getattr(self, 'pipeline', None):
                    try:
                        self.pipeline.stop()
                        print("[SOARMBackend] RealSense camera stopped")
                    except Exception as e:
                        print(f"[SOARMBackend] Error stopping RealSense: {e}")
                    self.pipeline = None
                
                if getattr(self, 'cap', None):
                    try:
                        self.cap.release()
                        print("[SOARMBackend] OpenCV webcam stopped")
                    except Exception as e:
                        print(f"[SOARMBackend] Error stopping OpenCV webcam: {e}")
                    self.cap = None
                
                self.camera_started = False
                self.camera_type = "none"

            # Disconnect Robot
            if self.robot is not None:
                self.robot.disconnect()
                self.robot = None
                print("[SOARMBackend] Robot disconnected")
            
            self.status = BackendStatus.DISCONNECTED
            print(f"[SOARMBackend] ✓ Disconnected")
        except Exception as e:
            print(f"[SOARMBackend] Error during disconnect: {e}")
            self.status = BackendStatus.ERROR
    
    def render(self, width: int = 960, height: int = 540) -> Optional[bytes]:
        """Render a frame from camera"""
        if not self.camera_started:
            return None
            
        try:
            img = None
            
            # Case 1: RealSense
            if self.camera_type == "realsense" and self.pipeline:
                frames = self.pipeline.wait_for_frames(timeout_ms=100)
                color_frame = frames.get_color_frame()
                if color_frame:
                    img = np.asanyarray(color_frame.get_data())
            
            # Case 2: OpenCV Webcam
            elif self.camera_type == "opencv" and self.cap:
                ret, frame = self.cap.read()
                if ret:
                    img = frame
            
            if img is None:
                return None
            
            # Resize if needed (RealSense is 640x480, target usually 960x540)
            # Use OpenCV for resizing and encoding
            if OPENCV_AVAILABLE:
                if img.shape[1] != width or img.shape[0] != height:
                    img = cv2.resize(img, (width, height))
                
                # Encode to JPEG
                _, jpeg = cv2.imencode('.jpg', img)
                return jpeg.tobytes()
            else:
                # Fallback to PIL if OpenCV is not available
                try:
                    from PIL import Image
                    from io import BytesIO
                    
                    # RealSense/OpenCV typically BGR for OpenCV processing
                    # If we use PIL, we need to convert BGR to RGB
                    
                    # Manual BGR to RGB conversion for numpy array
                    if len(img.shape) == 3 and img.shape[2] == 3:
                        img_rgb = img[..., ::-1]
                    else:
                        img_rgb = img
                        
                    pil_img = Image.fromarray(img_rgb)
                    
                    if pil_img.size[0] != width or pil_img.size[1] != height:
                        pil_img = pil_img.resize((width, height), Image.Resampling.LANCZOS)
                    
                    buf = BytesIO()
                    pil_img.save(buf, format="JPEG", quality=75)
                    return buf.getvalue()
                except ImportError:
                    # Neither OpenCV nor PIL available
                    return None
                except Exception as e:
                    print(f"[SOARMBackend] PIL render error: {e}")
                    return None
                
        except Exception as e:
            # print(f"[SOARMBackend] Render error: {e}") # Don't spam logs
            return None

    def send_target_pose(self, position: np.ndarray, 
                         orientation: np.ndarray,
                         velocity_limit: float = 0.1) -> bool:
        """
        Send target pose to SO-ARM
        
        Since we lack proper IK, we will map Cartesian inputs to individual joints
        for simple teleoperation testing.
        
        Mapping (Simple):
        - X axis -> Shoulder Pan (Joint 1)
        - Y axis -> Shoulder Lift (Joint 2)
        - Z axis -> Elbow Flex (Joint 3)
        - Roll   -> Wrist Flex (Joint 4)
        - Pitch  -> Wrist Roll (Joint 5)
        - Yaw    -> Gripper (Joint 6)
        """
        if not self.is_connected() or self.robot is None:
            print("[SOARMBackend] Not connected!")
            return False
        
        try:
            with self.update_lock:
                # 1. Read current joints
                current_joints = self.get_current_joints()
                if current_joints is None:
                    return False
                
                # 2. Calculate delta from current "virtual" position
                # The controller sends absolute target positions, but since we don't have FK,
                # we don't know where we are in Cartesian space.
                # So we have to rely on relative movement or assume a starting pose.
                
                # BETTER APPROACH FOR TESTING:
                # Map the *change* in target position to *change* in joint angles.
                # The controller maintains 'self.current_position'.
                # But here we receive the TARGET position.
                
                # Let's map World X/Y/Z directly to Joint 1/2/3 offsets from a "home" position.
                # This is very rough but allows movement.
                
                # Home position (approximate mid-range)
                # These are raw values (degrees usually)
                # shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
                # Note: You might need to adjust these base values/scales
                
                # Scale factors (meters to degrees)
                pos_scale = 500.0  # 1cm -> 5 degrees
                rot_scale = 50.0   # 1 rad -> ~50 degrees
                
                # We need to calculate the difference from the LAST command or initial state
                # But send_target_pose receives absolute world coordinates.
                # Without FK, we can't map absolute world -> absolute joint.
                
                # HACK: Use the difference between current target and stored "current_position"
                # in the backend to drive incremental joint updates.
                
                # delta_pos = position - self.current_position
                # The position coming in is accumulated in control_logic.py: target_position = self.current_position + delta_pos_world
                # So position IS the new absolute target in Cartesian space.
                # However, our self.current_position in backend was just initialized to [0,0,0] and updated blindly.
                
                # The problem is: control_logic.py thinks we are at [0,0,0] initially.
                # When we press 'forward', it sends target [0, 0.02, 0].
                # delta_pos becomes [0, 0.02, 0].
                
                # BUT: The delta needs to be applied to the CURRENT JOINTS.
                # If we just add delta to current joints every time, it might work if delta_pos is truly a delta.
                # But here 'position' is absolute.
                # We need the DELTA from the LAST command.
                
                # Let's simply calculate delta from our own stored previous position
                delta_pos = position - self.current_position
                
                # If the user holds the key, control_logic sends increasing positions:
                # t0: [0, 0, 0]
                # t1: [0, 0.02, 0] -> delta [0, 0.02, 0]
                # t2: [0, 0.04, 0] -> delta [0, 0.02, 0]
                
                # So this logic holds.
                
                # Update internal state
                self.current_position = position
                self.current_orientation = orientation
                
                # If delta is too small, ignore (noise or no movement)
                if np.linalg.norm(delta_pos) < 0.0001:
                    return True
                
                # Calculate joint increments
                # Joint 1 (Pan) <- X
                # Joint 2 (Lift) <- Y
                # Joint 3 (Elbow) <- Z
                
                # REVERSE DIRECTION CHECK:
                # Typically:
                # Y+ (Forward) -> Should extend arm or lift shoulder?
                # Let's try:
                # X (Left/Right) -> Pan (Joint 1)
                # Y (Forward/Back) -> Lift (Joint 2)
                # Z (Up/Down) -> Elbow (Joint 3) - maybe?
                
                d_j1 = delta_pos[0] * pos_scale
                d_j2 = delta_pos[1] * pos_scale 
                d_j3 = delta_pos[2] * pos_scale
                
                # Create new targets
                new_joints = current_joints.copy()
                
                # Apply changes
                # Note: Joint directions might need negation based on mounting
                if len(new_joints) >= 1: new_joints[0] += d_j1
                if len(new_joints) >= 2: new_joints[1] += d_j2 # Try inverting if moves wrong way
                if len(new_joints) >= 3: new_joints[2] += d_j3
                
                print(f"[SOARMBackend] Moving: Delta={delta_pos} -> Joints+={[d_j1, d_j2, d_j3]}")

                # Send using our joint control method
                return self.send_joint_positions(new_joints)
                
        except Exception as e:
            print(f"[SOARMBackend] Error sending command: {e}")
            return False
    
    def send_joint_positions(self, joint_positions: np.ndarray) -> bool:
        """
        Direct joint control (easier than IK)
        
        Args:
            joint_positions: Target joint angles (radians or normalized -1 to 1)
        """
        if not self.is_connected() or self.robot is None:
            return False
        
        try:
            with self.update_lock:
                # Convert array back to dict for the bus
                # Assuming joint_positions matches the sorted motor order from get_current_joints
                sorted_motors = sorted(self.robot.bus.motors.items(), key=lambda x: x[1].id)
                
                # Write joint positions to robot using the bus
                # Filter out joints that are not in the current robot
                if len(joint_positions) != len(sorted_motors):
                    # Try to map by matching names/IDs if possible, or just truncate
                    # The leader sends 6 joints (usually). The follower has 5.
                    # sorted_motors only contains the 5 connected motors.
                    
                    # If leader has more joints (e.g. 6) than follower (5), truncate the list
                    if len(joint_positions) > len(sorted_motors):
                        # Assuming the first N joints match
                        joint_positions = joint_positions[:len(sorted_motors)]
                    else:
                        print(f"[SOARMBackend] Error: joint count mismatch. Expected {len(sorted_motors)}, got {len(joint_positions)}")
                        return False
                
                target_dict = {name: int(val) for (name, _), val in zip(sorted_motors, joint_positions)}
                
                # Write joint positions to robot using the bus
                self.robot.bus.sync_write("Goal_Position", target_dict)
                
                self.command_count += 1
                self.last_update_time = time.time()
                print(f"[SOARMBackend] 📥 Joint command sent: {target_dict}")
                return True
        except Exception as e:
            print(f"[SOARMBackend] Error sending joint command: {e}")
            return False
    
    def get_current_pose(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Get current robot pose"""
        if not self.is_connected() or self.robot is None:
            return None, None
        
        try:
            with self.update_lock:
                # Read current joint positions (just to verify connection)
                # Use only connected motors
                if self.robot and self.robot.bus:
                    _ = self.robot.bus.sync_read("Present_Position")
                
                # TODO: Implement forward kinematics to get Cartesian pose
                # For now, return dummy pose
                return self.current_position.copy(), self.current_orientation.copy()
                
        except Exception as e:
            print(f"[SOARMBackend] Error reading pose: {e}")
            return None, None
    
    def get_current_joints(self) -> Optional[np.ndarray]:
        """Get current joint positions"""
        if not self.is_connected() or self.robot is None:
            return None
        
        try:
            # Using bus directly for raw values (faster/easier for simple check)
            # The robot.bus is FeetechMotorsBus
            # sync_read might expect list of motors, or defaults to all in self.motors
            # If we call it without arguments, it reads all motors in self.motors
            # Our self.robot.bus.motors should only contain connected motors now.
            
            joints_dict = self.robot.bus.sync_read("Present_Position")
            
            # Convert dict to array in order of motor IDs
            # Get list of motor names sorted by ID to ensure consistent order
            sorted_motors = sorted(self.robot.bus.motors.items(), key=lambda x: x[1].id)
            joint_values = [joints_dict[name] for name, _ in sorted_motors]
            
            return np.array(joint_values)
        except Exception as e:
            # print(f"[SOARMBackend] Error reading joints: {e}") # Suppress spam
            return None
    
    def get_status(self) -> Dict[str, Any]:
        """Get SO-ARM backend status"""
        status = {
            'name': self.name,
            'status': self.status.value,
            'port': self.port,
            'command_count': self.command_count,
            'last_update': self.last_update_time,
            'lerobot_available': LEROBOT_AVAILABLE,
            'camera_available': REALSENSE_AVAILABLE,
            'camera_started': self.camera_started
        }
        
        if self.is_connected() and self.robot is not None:
            try:
                joints = self.get_current_joints()
                if joints is not None:
                    status['current_joints'] = joints.tolist()
            except:
                pass
        
        return status
