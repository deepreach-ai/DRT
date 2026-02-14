"""
Dual SO-ARM Backend for controlling two independent arms (Left and Right)
"""
import numpy as np
import time
import threading
import os
import io
from typing import Tuple, Optional, Dict, Any

from PIL import Image

from server.robot_backend import RobotBackend, BackendStatus
from server.backends.soarm_backend import SOARMBackend

class DualSOARMBackend(RobotBackend):
    """Backend for controlling two SO-ARM101 robots simultaneously"""
    
    def __init__(self, left_port: str, right_port: str, name: str = "dual_so_arm"):
        super().__init__(name)
        self.left_port = left_port
        self.right_port = right_port
        
        # Initialize two independent backends
        self.left_arm = SOARMBackend(port=left_port, name="left_arm")
        self.right_arm = SOARMBackend(port=right_port, name="right_arm")
        
        self.command_count = 0
        
    def connect(self) -> bool:
        """Connect to both arms"""
        prev_camera_env = os.getenv("TELEOP_ENABLE_CAMERAS")
        try:
            os.environ["TELEOP_ENABLE_CAMERAS"] = "false"
            print(f"[DualSOARM] Connecting to Left Arm at {self.left_port}...")
            left_ok = self.left_arm.connect()

            os.environ["TELEOP_ENABLE_CAMERAS"] = "true"
            print(f"[DualSOARM] Connecting to Right Arm at {self.right_port}...")
            right_ok = self.right_arm.connect()
        finally:
            if prev_camera_env is None:
                os.environ.pop("TELEOP_ENABLE_CAMERAS", None)
            else:
                os.environ["TELEOP_ENABLE_CAMERAS"] = prev_camera_env
        
        if left_ok and right_ok:
            self.status = BackendStatus.CONNECTED
        elif left_ok or right_ok:
            self.status = BackendStatus.CONNECTING # Partially connected
            print("[DualSOARM] ⚠️ Only one arm connected!")
        else:
            self.status = BackendStatus.ERROR
            return False
            
        self.last_update_time = time.time()
        return True
        
    def disconnect(self):
        """Disconnect from both arms"""
        self.left_arm.disconnect()
        self.right_arm.disconnect()
        self.status = BackendStatus.DISCONNECTED
        
    def send_target_pose(self, position: np.ndarray, 
                         orientation: np.ndarray,
                         velocity_limit: float = 0.1,
                         gripper_state: float = -1.0,
                         handedness: str = "right") -> bool:
        """Route command to the correct arm and apply base offsets"""
        # World frame to robot base frame conversion
        # Left base is at y=0.2, Right base is at y=-0.2 in URDF
        local_pos = position.copy()
        
        if handedness.lower() == "left":
            local_pos[1] -= 0.2 # Subtract y offset
            if self.left_arm.is_connected():
                return self.left_arm.send_target_pose(local_pos, orientation, velocity_limit, gripper_state)
            return False
        else:
            local_pos[1] += 0.2 # Subtract y offset (since right is at -0.2, we add 0.2 to get 0)
            if self.right_arm.is_connected():
                return self.right_arm.send_target_pose(local_pos, orientation, velocity_limit, gripper_state)
            return False
            
    def get_current_pose(self, handedness: str = "right") -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Get pose for specific arm"""
        if handedness.lower() == "left":
            pos, ori = self.left_arm.get_current_pose()
            if pos is not None:
                pos = pos.copy()
                pos[1] += 0.2
            return pos, ori
        pos, ori = self.right_arm.get_current_pose()
        if pos is not None:
            pos = pos.copy()
            pos[1] -= 0.2
        return pos, ori

    def render(self, width: int = 960, height: int = 540, camera: Optional[str] = None) -> Optional[bytes]:
        cam = camera.lower() if isinstance(camera, str) else None

        if cam == "left":
            return self.left_arm.render(width=width, height=height)
        if cam == "right":
            return self.right_arm.render(width=width, height=height)

        left_jpg = self.left_arm.render(width=max(1, width // 2), height=height)
        right_jpg = self.right_arm.render(width=max(1, width // 2), height=height)

        if left_jpg is None and right_jpg is None:
            return None
        if left_jpg is None:
            return right_jpg
        if right_jpg is None:
            return left_jpg

        try:
            left_img = Image.open(io.BytesIO(left_jpg)).convert("RGB")
            right_img = Image.open(io.BytesIO(right_jpg)).convert("RGB")

            w_half = width // 2
            left_img = left_img.resize((w_half, height))
            right_img = right_img.resize((width - w_half, height))

            canvas = Image.new("RGB", (width, height))
            canvas.paste(left_img, (0, 0))
            canvas.paste(right_img, (w_half, 0))

            out = io.BytesIO()
            canvas.save(out, format="JPEG", quality=85)
            return out.getvalue()
        except Exception:
            return left_jpg
        
    def get_joint_positions(self) -> Optional[np.ndarray]:
        """
        Get combined joint positions for both arms
        Returns: [left_joints..., right_joints...]
        """
        l_joints = self.left_arm.get_joint_positions()
        r_joints = self.right_arm.get_joint_positions()
        
        if l_joints is None and r_joints is None:
            return None
            
        # If one is missing, use zeros
        if l_joints is None: l_joints = np.zeros(6)
        if r_joints is None: r_joints = np.zeros(6)
        
        return np.concatenate([l_joints, r_joints])
        
    def get_target_joint_positions(self) -> Optional[np.ndarray]:
        """Get combined target joint positions for both arms"""
        l_targets = self.left_arm.get_target_joint_positions()
        r_targets = self.right_arm.get_target_joint_positions()
        
        if l_targets is None and r_targets is None:
            return None
            
        if l_targets is None: l_targets = np.zeros(6)
        if r_targets is None: r_targets = np.zeros(6)
        
        return np.concatenate([l_targets, r_targets])
        
    def get_status(self) -> Dict[str, Any]:
        """Combined status"""
        return {
            "name": self.name,
            "status": self.status.value,
            "left_arm": self.left_arm.get_status(),
            "right_arm": self.right_arm.get_status(),
            "last_update": self.last_update_time
        }
