import time
import numpy as np
from typing import Optional, Tuple, Dict, Any
import threading
import socket
import json
import logging
import sys
import os

# Add project root to path if needed
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))

from server.robot_backend import RobotBackend, BackendStatus

logger = logging.getLogger(__name__)

class IsaacSimBackend(RobotBackend):
    """
    Backend for communicating with Isaac Sim via TCP Socket.
    Acts as a Server: Isaac Sim (Client) connects to this backend.
    """
    def __init__(self, name: str = "isaac_sim", host: str = "0.0.0.0", port: int = 9000):
        super().__init__(name)
        self.host = host
        self.port = port
        
        # State
        self.current_position = np.array([0.0, 0.0, 0.5])
        self.current_orientation = np.array([1.0, 0.0, 0.0, 0.0])
        self.command_count = 0
        self.last_error: Optional[str] = None
        self.lock = threading.Lock()
        
        # Video state
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # Network
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self._running = False
        self._accept_thread = None
        self._receive_thread = None

    def connect(self) -> bool:
        """Start the TCP server and listen for connections"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            
            self._running = True
            self.status = BackendStatus.CONNECTING
            
            # Start accept thread
            self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
            self._accept_thread.start()
            
            print(f"[IsaacBackend] Listening on {self.host}:{self.port}")
            return True
            
        except Exception as e:
            self.status = BackendStatus.ERROR
            self.last_error = str(e)
            print(f"[IsaacBackend] Failed to start server: {e}")
            return False

    def disconnect(self):
        """Stop server and close connections"""
        self._running = False
        self.status = BackendStatus.DISCONNECTED
        
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
            self.client_socket = None
            
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
            self.server_socket = None

    def _accept_loop(self):
        """Background thread to accept incoming connections"""
        while self._running:
            try:
                if self.server_socket is None:
                    break
                    
                # Blocking accept
                client_sock, address = self.server_socket.accept()
                print(f"[IsaacBackend] Client connected from {address}")
                
                with self.lock:
                    self.client_socket = client_sock
                    self.client_address = address
                    self.status = BackendStatus.CONNECTED
                
                # Start receiver for this client
                self._receive_loop()
                
            except Exception as e:
                if self._running:
                    print(f"[IsaacBackend] Accept error: {e}")
                    time.sleep(1)

    def _receive_loop(self):
        """Handle communication with connected client"""
        buffer = ""
        while self._running and self.client_socket:
            try:
                data = self.client_socket.recv(4096)
                if not data:
                    break
                
                buffer += data.decode('utf-8')
                
                # Process complete lines (assuming JSON per line)
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        self._process_message(line)
                        
            except Exception as e:
                print(f"[IsaacBackend] Receive error: {e}")
                break
        
        print(f"[IsaacBackend] Client disconnected")
        with self.lock:
            self.client_socket = None
            self.status = BackendStatus.CONNECTING  # Go back to listening

    def _process_message(self, message: str):
        """Process incoming JSON message"""
        try:
            data = json.loads(message)
            if data.get('type') == 'state':
                payload = data.get('payload', {})
                pos = payload.get('position')
                orient = payload.get('orientation')
                
                with self.lock:
                    if pos:
                        self.current_position = np.array(pos)
                    if orient:
                        self.current_orientation = np.array(orient)
                    self.last_update_time = time.time()
                    
        except json.JSONDecodeError:
            pass
        except Exception as e:
            print(f"[IsaacBackend] Message processing error: {e}")

    def send_target_pose(self, position: np.ndarray, orientation: np.ndarray, velocity_limit: float = 0.1, gripper_state: float = -1.0) -> bool:
        if not self.is_connected() or not self.client_socket:
            return False
            
        try:
            message = {
                "type": "command",
                "timestamp": time.time(),
                "payload": {
                    "target_position": position.tolist(),
                    "target_orientation": orientation.tolist(),
                    "velocity_limit": velocity_limit,
                    "gripper_state": gripper_state
                }
            }
            
            # Send as JSON line
            data = json.dumps(message) + "\n"
            self.client_socket.sendall(data.encode('utf-8'))
            
            with self.lock:
                self.command_count += 1
                
            return True
        except Exception as e:
            print(f"[IsaacBackend] Send error: {e}")
            with self.lock:
                self.last_error = str(e)
            return False

    def get_current_pose(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        with self.lock:
            return self.current_position.copy(), self.current_orientation.copy()

    def get_status(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "status": self.status.value,
            "command_count": self.command_count,
            "current_position": self.current_position.tolist(),
            "current_orientation": self.current_orientation.tolist(),
            "last_error": self.last_error,
            "last_update": self.last_update_time,
            "endpoint": f"{self.host}:{self.port}",
            "client": str(self.client_address) if self.client_address else None,
            "has_video": self.latest_frame is not None
        }

    def update_frame(self, jpg_bytes: bytes):
        """Update the latest video frame"""
        with self.frame_lock:
            self.latest_frame = jpg_bytes

    def render(self, width: int = 960, height: int = 540) -> Optional[bytes]:
        """Render the latest frame"""
        with self.frame_lock:
            if self.latest_frame:
                return self.latest_frame
        return None
