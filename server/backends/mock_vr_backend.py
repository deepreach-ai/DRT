"""
Mock VR Backend - Simulates 3 cameras for VR development without hardware
"""
from typing import Optional, Dict, Any, Tuple
import numpy as np
import time
import cv2
from robot_backend import RobotBackend, BackendStatus


class MockVRBackend(RobotBackend):
    """Mock backend with 3 simulated cameras for VR testing"""
    
    def __init__(self, name: str = "mock_vr"):
        super().__init__(name)
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_orientation = np.array([1.0, 0.0, 0.0, 0.0])
        self.command_count = 0
        
        # Simulated cameras
        self.cameras = {
            'left': True,   # Simulated left camera
            'right': True,  # Simulated right camera
            'depth': True,  # Simulated depth camera
        }
        
        # Color for each camera (for visual distinction)
        self.camera_colors = {
            'left': (100, 100, 255),   # Reddish
            'right': (100, 255, 100),  # Greenish
            'depth': (255, 100, 100),  # Blueish
        }
        
        self.frame_counter = 0
    
    def connect(self) -> bool:
        """Simulate connection"""
        print(f"[MockVRBackend] Connecting...")
        time.sleep(0.5)  # Simulate connection delay
        
        self.status = BackendStatus.CONNECTED
        self.last_update_time = time.time()
        
        print(f"[MockVRBackend] ✓ Connected")
        print(f"[MockVRBackend] ✓ 3 simulated cameras active")
        return True
    
    def disconnect(self):
        """Simulate disconnection"""
        print(f"[MockVRBackend] Disconnecting...")
        self.status = BackendStatus.DISCONNECTED
        print(f"[MockVRBackend] ✓ Disconnected")
    
    def send_target_pose(self, position: np.ndarray, 
                         orientation: np.ndarray,
                         velocity_limit: float = 0.1) -> bool:
        """Accept and simulate pose commands"""
        self.current_position = position.copy()
        self.current_orientation = orientation.copy()
        self.command_count += 1
        self.last_update_time = time.time()
        
        if self.command_count % 10 == 0:  # Log every 10th command
            print(f"[MockVRBackend] Position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
        
        return True
    
    def get_current_pose(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Return current simulated pose"""
        return self.current_position.copy(), self.current_orientation.copy()
    
    def render(self, width: int = 960, height: int = 540) -> Optional[bytes]:
        """Render main camera view (combined stereo)"""
        return self.render_stereo_frame()
    
    def render_camera(self, camera_name: str) -> Optional[bytes]:
        """
        Render a specific simulated camera
        
        Args:
            camera_name: 'left', 'right', or 'depth'
        """
        if camera_name not in self.cameras:
            return None
        
        try:
            # Create a colored frame with text
            frame = self._generate_mock_frame(camera_name, 640, 480)
            
            # Encode to JPEG
            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            return jpeg.tobytes()
            
        except Exception as e:
            print(f"[MockVRBackend] Error rendering {camera_name}: {e}")
            return None
    
    def render_stereo_frame(self) -> Optional[bytes]:
        """Generate side-by-side stereo frame for VR"""
        try:
            # Generate both frames
            left_frame = self._generate_mock_frame('left', 640, 480)
            right_frame = self._generate_mock_frame('right', 640, 480)
            
            # Side-by-side
            stereo = np.hstack([left_frame, right_frame])  # Shape: (480, 1280, 3)
            
            # Encode
            _, jpeg = cv2.imencode('.jpg', stereo, [cv2.IMWRITE_JPEG_QUALITY, 85])
            return jpeg.tobytes()
            
        except Exception as e:
            print(f"[MockVRBackend] Stereo render error: {e}")
            return None
    
    def _generate_mock_frame(self, camera_name: str, width: int, height: int) -> np.ndarray:
        """Generate a mock camera frame with animated content"""
        
        # Base color from camera type
        base_color = self.camera_colors.get(camera_name, (128, 128, 128))
        
        # Create gradient background (animated)
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Animated gradient
        offset = (self.frame_counter % 255)
        for y in range(height):
            intensity = int((y / height) * 128) + offset % 128
            color = tuple(int(c * intensity / 255) for c in base_color)
            frame[y, :] = color
        
        # Add camera label
        font = cv2.FONT_HERSHEY_SIMPLEX
        label = f"{camera_name.upper()} CAMERA (SIMULATED)"
        text_size = cv2.getTextSize(label, font, 1.2, 2)[0]
        text_x = (width - text_size[0]) // 2
        text_y = 60
        
        # Black background for text
        cv2.rectangle(frame, 
                     (text_x - 10, text_y - 40), 
                     (text_x + text_size[0] + 10, text_y + 10),
                     (0, 0, 0), -1)
        
        # White text
        cv2.putText(frame, label, (text_x, text_y), 
                   font, 1.2, (255, 255, 255), 2)
        
        # Add frame counter
        counter_text = f"Frame: {self.frame_counter}"
        cv2.putText(frame, counter_text, (20, height - 30),
                   font, 0.8, (255, 255, 255), 2)
        
        # Add current position info
        pos_text = f"Pos: [{self.current_position[0]:.2f}, {self.current_position[1]:.2f}, {self.current_position[2]:.2f}]"
        cv2.putText(frame, pos_text, (20, height - 60),
                   font, 0.6, (255, 255, 255), 1)
        
        # Add timestamp
        timestamp = time.strftime("%H:%M:%S")
        cv2.putText(frame, timestamp, (width - 150, height - 30),
                   font, 0.8, (255, 255, 255), 2)
        
        # Add animated moving dot (to show it's live)
        dot_x = int((self.frame_counter * 5) % width)
        dot_y = height // 2
        cv2.circle(frame, (dot_x, dot_y), 10, (255, 255, 0), -1)
        
        # Increment frame counter
        self.frame_counter += 1
        
        return frame
    
    def get_status(self) -> Dict[str, Any]:
        """Get mock backend status"""
        return {
            'name': self.name,
            'status': self.status.value,
            'command_count': self.command_count,
            'last_update': self.last_update_time,
            'cameras': {
                'left': True,
                'right': True,
                'depth': True,
            },
            'camera_types': {
                'left': 'mock_simulated',
                'right': 'mock_simulated',
                'depth': 'mock_simulated',
            },
            'type': 'mock_vr',
            'frame_count': self.frame_counter,
        }
