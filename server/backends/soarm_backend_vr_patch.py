# Multi-Camera Extension for SOARMBackend
# Add these methods to your SOARMBackend class in server/backends/soarm_backend.py

"""
INSTRUCTIONS:
1. Add these imports at the top of soarm_backend.py:
   import cv2  # Should already be there
   
2. Modify the __init__ method to initialize multi-camera state:
   Add after self.command_count = 0:
   
        # Multi-camera support for VR
        self.cameras = {
            'left': None,     # Webcam for left eye (Arm 1)
            'right': None,    # Webcam for right eye (Arm 2) 
            'depth': None,    # RealSense depth camera
        }
        self.camera_captures = {
            'left': None,
            'right': None,
        }
        
3. Replace the connect_cameras() section in connect() method with the new one below

4. Add the new methods at the end of the class (before get_status())
"""

# ============================================================
# UPDATED connect() method - Camera Section
# ============================================================

def connect(self) -> bool:
    """Connect to SO-ARM robot and Cameras"""
    try:
        print(f"[SOARMBackend] Connecting to SO-ARM at {self.port}...")
        self.status = BackendStatus.CONNECTING
        
        # 1. Connect to Robot (existing code - don't change)
        config = SO101FollowerConfig(
            port=self.port,
            disable_torque_on_disconnect=True,
            use_degrees=False
        )
        self.robot = FlexibleSO101Follower(config)
        self.robot.connect()
        print(f"[SOARMBackend] ✓ Connected to SO-ARM")

        # 2. Connect to Cameras - NEW MULTI-CAMERA SETUP
        self._init_multi_cameras()
        
        self.status = BackendStatus.CONNECTED
        self.last_update_time = time.time()
        return True
        
    except Exception as e:
        print(f"[SOARMBackend] ✗ Connection failed: {e}")
        self.status = BackendStatus.ERROR
        return False


# ============================================================
# NEW METHODS - Add to SOARMBackend class
# ============================================================

def _init_multi_cameras(self):
    """Initialize 3-camera setup for VR"""
    print("[SOARMBackend] Initializing multi-camera setup...")
    
    # Strategy: Try each camera, track what's available
    available_devices = self._scan_video_devices()
    print(f"[SOARMBackend] Found {len(available_devices)} video devices: {available_devices}")
    
    # Priority order for device assignment
    # Device 0: Usually built-in webcam (use for depth if no RealSense)
    # Device 1: First external webcam (left)
    # Device 2: Second external webcam (right)
    
    # Try RealSense first (if available)
    USE_REALSENSE = os.getenv('USE_REALSENSE', 'false').lower() == 'true'
    
    if REALSENSE_AVAILABLE and USE_REALSENSE:
        try:
            print("[SOARMBackend] Initializing RealSense D435...")
            self.pipeline = rs.pipeline()
            self.camera_config = rs.config()
            self.camera_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.camera_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.pipeline.start(self.camera_config)
            self.cameras['depth'] = 'realsense'
            self.camera_started = True
            print("[SOARMBackend] ✓ RealSense D435 connected")
        except Exception as e:
            print(f"[SOARMBackend] ⚠ RealSense failed: {e}")
    
    # Try OpenCV webcams
    if OPENCV_AVAILABLE:
        # Left camera (try device 1, fallback to 0)
        for device_id in [1, 0]:
            if device_id in available_devices:
                try:
                    cap = cv2.VideoCapture(device_id)
                    if cap.isOpened():
                        # Test read
                        ret, frame = cap.read()
                        if ret:
                            self.camera_captures['left'] = cap
                            self.cameras['left'] = f'opencv_{device_id}'
                            print(f"[SOARMBackend] ✓ Left camera connected (device {device_id})")
                            break
                        else:
                            cap.release()
                except Exception as e:
                    print(f"[SOARMBackend] ⚠ Left camera device {device_id} failed: {e}")
        
        # Right camera (try device 2, fallback to any remaining)
        for device_id in [2, 3]:
            if device_id in available_devices and device_id != int(self.cameras.get('left', 'opencv_-1').split('_')[-1]):
                try:
                    cap = cv2.VideoCapture(device_id)
                    if cap.isOpened():
                        ret, frame = cap.read()
                        if ret:
                            self.camera_captures['right'] = cap
                            self.cameras['right'] = f'opencv_{device_id}'
                            print(f"[SOARMBackend] ✓ Right camera connected (device {device_id})")
                            break
                        else:
                            cap.release()
                except Exception as e:
                    print(f"[SOARMBackend] ⚠ Right camera device {device_id} failed: {e}")
        
        # Fallback depth camera if no RealSense
        if not self.cameras['depth'] and 0 in available_devices:
            try:
                cap = cv2.VideoCapture(0)
                if cap.isOpened():
                    ret, frame = cap.read()
                    if ret:
                        self.camera_captures['depth'] = cap
                        self.cameras['depth'] = 'opencv_0'
                        print(f"[SOARMBackend] ✓ Depth fallback camera connected (device 0)")
                    else:
                        cap.release()
            except Exception as e:
                print(f"[SOARMBackend] ⚠ Depth fallback failed: {e}")
    
    # Summary
    active_cameras = [k for k, v in self.cameras.items() if v]
    print(f"[SOARMBackend] Camera setup complete: {len(active_cameras)}/3 cameras active")
    print(f"[SOARMBackend] Active cameras: {active_cameras}")
    
    if len(active_cameras) == 0:
        print("[SOARMBackend] ⚠ WARNING: No cameras available! VR will not work.")
    elif len(active_cameras) < 3:
        print(f"[SOARMBackend] ⚠ WARNING: Only {len(active_cameras)}/3 cameras active. Some VR features limited.")


def _scan_video_devices(self, max_devices=10):
    """Scan for available video capture devices"""
    available = []
    
    if not OPENCV_AVAILABLE:
        return available
    
    for i in range(max_devices):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            available.append(i)
            cap.release()
    
    return available


def render_camera(self, camera_name: str) -> Optional[bytes]:
    """
    Render a specific camera stream
    
    Args:
        camera_name: 'left', 'right', or 'depth'
    
    Returns:
        JPEG-encoded frame bytes, or None if unavailable
    """
    if camera_name not in self.cameras or not self.cameras[camera_name]:
        return None
    
    try:
        camera_type = self.cameras[camera_name]
        
        # Case 1: RealSense depth camera
        if camera_type == 'realsense' and camera_name == 'depth':
            return self._render_realsense_depth()
        
        # Case 2: OpenCV webcam
        elif camera_type.startswith('opencv'):
            # For left/right, use camera_captures
            if camera_name in self.camera_captures and self.camera_captures[camera_name]:
                cap = self.camera_captures[camera_name]
                ret, frame = cap.read()
                if ret:
                    # Resize to consistent resolution
                    frame = cv2.resize(frame, (640, 480))
                    # Encode to JPEG
                    _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    return jpeg.tobytes()
            
            # For depth fallback
            elif camera_name == 'depth' and 'depth' in self.camera_captures:
                cap = self.camera_captures['depth']
                ret, frame = cap.read()
                if ret:
                    frame = cv2.resize(frame, (640, 480))
                    _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    return jpeg.tobytes()
        
        return None
        
    except Exception as e:
        print(f"[SOARMBackend] Error rendering {camera_name} camera: {e}")
        return None


def _render_realsense_depth(self) -> Optional[bytes]:
    """Render RealSense depth as colored overlay"""
    if not self.pipeline:
        return None
    
    try:
        frames = self.pipeline.wait_for_frames(timeout_ms=100)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not (depth_frame and color_frame):
            return None
        
        # Convert to numpy
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # Colorize depth
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), 
            cv2.COLORMAP_JET
        )
        
        # Blend with RGB
        overlay = cv2.addWeighted(color_image, 0.7, depth_colormap, 0.3, 0)
        
        # Resize and encode
        overlay = cv2.resize(overlay, (640, 480))
        _, jpeg = cv2.imencode('.jpg', overlay, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return jpeg.tobytes()
        
    except Exception as e:
        print(f"[SOARMBackend] Depth render error: {e}")
        return None


def render_stereo_frame(self) -> Optional[bytes]:
    """Generate side-by-side stereo frame for VR"""
    try:
        # Get both left and right frames
        left_bytes = self.render_camera('left')
        right_bytes = self.render_camera('right')
        
        if not (left_bytes and right_bytes):
            return None
        
        # Decode JPEG to numpy
        left_img = cv2.imdecode(np.frombuffer(left_bytes, np.uint8), cv2.IMREAD_COLOR)
        right_img = cv2.imdecode(np.frombuffer(right_bytes, np.uint8), cv2.IMREAD_COLOR)
        
        # Ensure same size
        left_img = cv2.resize(left_img, (640, 480))
        right_img = cv2.resize(right_img, (640, 480))
        
        # Side-by-side
        stereo = np.hstack([left_img, right_img])  # Shape: (480, 1280, 3)
        
        # Encode
        _, jpeg = cv2.imencode('.jpg', stereo, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return jpeg.tobytes()
        
    except Exception as e:
        print(f"[SOARMBackend] Stereo frame error: {e}")
        return None


# ============================================================
# UPDATED disconnect() method - add camera cleanup
# ============================================================

def disconnect(self):
    """Disconnect from SO-ARM robot and Cameras"""
    try:
        print(f"[SOARMBackend] Disconnecting...")
        
        # Stop Cameras
        if self.camera_started or any(self.cameras.values()):
            # RealSense
            if getattr(self, 'pipeline', None):
                try:
                    self.pipeline.stop()
                    print("[SOARMBackend] RealSense camera stopped")
                except Exception as e:
                    print(f"[SOARMBackend] Error stopping RealSense: {e}")
                self.pipeline = None
            
            # OpenCV webcams
            for name, cap in self.camera_captures.items():
                if cap:
                    try:
                        cap.release()
                        print(f"[SOARMBackend] {name} camera stopped")
                    except Exception as e:
                        print(f"[SOARMBackend] Error stopping {name} camera: {e}")
            
            self.camera_captures = {'left': None, 'right': None, 'depth': None}
            self.cameras = {'left': None, 'right': None, 'depth': None}
            self.camera_started = False

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


# ============================================================
# UPDATED get_status() method - add camera info
# ============================================================

def get_status(self) -> Dict[str, Any]:
    """Get SO-ARM backend status"""
    status = {
        'name': self.name,
        'status': self.status.value,
        'port': self.port,
        'command_count': self.command_count,
        'last_update': self.last_update_time,
        'lerobot_available': LEROBOT_AVAILABLE,
        'cameras': {
            'left': self.cameras.get('left') is not None,
            'right': self.cameras.get('right') is not None,
            'depth': self.cameras.get('depth') is not None,
        },
        'camera_types': self.cameras,
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
