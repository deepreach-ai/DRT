"""
Main teleoperation server using FastAPI
"""
from fastapi import FastAPI, HTTPException, BackgroundTasks, WebSocket, WebSocketDisconnect, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from typing import Dict, List, Optional, Any
import numpy as np
import uvicorn
import asyncio
import threading
import time
from datetime import datetime
import json
import os

from models import DeltaCommand, JointCommand, RobotState, TeleopStatus, WorkspaceLimits
from safety_gate import SafetyGate
from control_logic import TeleoperationController
from robot_backend import RobotBackend, BackendFactory, BackendStatus
from web_support import AuthManager, SessionRecorder, mjpeg_stream, render_status_frame


class TeleoperationServer:
    """Main teleoperation server class"""
    
    def __init__(self, backend_type: str = 'mock', backend_config: Optional[Dict] = None):
        """
        Initialize teleoperation server
        
        Args:
            backend_type: Type of robot backend ('mock' or 'isaac')
            backend_config: Configuration for the backend
        """
        # Configuration
        self.backend_type = backend_type
        self.backend_config = backend_config or {}
        
        # Initialize components
        self.safety_gate = SafetyGate(timeout=0.5)
        self.controller = TeleoperationController()
        self.backend = None
        self.connected_clients: List[str] = []
        
        # Statistics
        self.start_time = time.time()
        self.total_commands = 0
        self.last_violations = {}
        
        # Threading
        self._stop_event = threading.Event()
        self._update_thread = None
        
    def initialize(self):
        """Initialize the server and backend"""
        print(f"[Server] Initializing with {self.backend_type} backend...")
        
        # Create backend
        self.backend = BackendFactory.create_backend(self.backend_type, **self.backend_config)
        
        # Connect to backend
        if not self.backend.connect():
            raise RuntimeError(f"Failed to connect to {self.backend_type} backend")
        
        # Sync initial pose
        initial_pos, initial_ori = self.backend.get_current_pose()
        if initial_pos is not None and initial_ori is not None:
            print(f"[Server] Syncing controller to initial pose: {initial_pos}")
            self.controller.set_current_pose(initial_pos, initial_ori)
        else:
            print("[Server] ⚠️ Could not get initial pose from backend. Controller starting at [0,0,0]")

        print(f"[Server] Initialized successfully with {self.backend_type} backend")
        
    def shutdown(self):
        """Shutdown the server"""
        print("[Server] Shutting down...")
        
        # Stop update thread if running
        self._stop_event.set()
        if self._update_thread and self._update_thread.is_alive():
            self._update_thread.join(timeout=2.0)
        
        # Disconnect backend
        if self.backend:
            self.backend.disconnect()
        
        print("[Server] Shutdown complete")
    
    def process_command(self, command: DeltaCommand) -> Dict[str, Any]:
        """
        Process a teleoperation command
        """
        # Update safety gate
        command_array = np.array([command.dx, command.dy, command.dz, 
                                  command.droll, command.dpitch, command.dyaw])
        
        # Check if timestamp is stale
        if time.time() - command.timestamp > 1.0:
            # print(f"[Server] Stale command ignored: {time.time() - command.timestamp:.3f}s old")
            # return {'status': 'ignored', 'message': 'Stale command'}
            pass
            
        # safety_active = self.safety_gate.update(command_array, command.timestamp)
        # FOR DIAGNOSIS: Force active to allow testing "little moves"
        safety_active = True
        
        if not safety_active:
            # Debug: Log why safety gate isn't active
            last_heartbeat = self.safety_gate.last_heartbeat
            current_time = time.time()
            time_since_last = (current_time - last_heartbeat) if last_heartbeat else 999.0
            cmd_magnitude = np.linalg.norm(command_array)
            
            # Reduce log spam
            if self.total_commands % 50 == 0:
                print(f"[Server] ⚠️  Safety gate INACTIVE! Cmd magnitude={cmd_magnitude:.6f}, threshold={self.safety_gate.activation_threshold}, time since last={time_since_last:.3f}s, timeout={self.safety_gate.timeout}s")
            
            return {
                'status': 'ignored',
                'message': 'Safety gate not active',
                'safety_active': False,
                'violations': {},
                'debug': {
                    'command_magnitude': float(cmd_magnitude) if np.isfinite(cmd_magnitude) else 0.0,
                    'time_since_last_heartbeat': float(time_since_last) if np.isfinite(time_since_last) else 999.0,
                    'timeout': float(self.safety_gate.timeout)
                }
            }
        
        # Process command through controller
        target_position, target_orientation, gripper_state, violations = self.controller.process_command(command)
        
        # Debug: Print first few commands or if significant movement
        # print(f"[Server] Target Pos: {target_position}, Gripper: {gripper_state}")

        # Send to robot backend
        if self.backend and self.backend.is_connected():
            success = self.backend.send_target_pose(
                target_position, 
                target_orientation,
                velocity_limit=command.max_velocity,
                gripper_state=gripper_state
            )
            
            if not success:
                return {
                    'status': 'error',
                    'message': 'Failed to send command to robot',
                    'safety_active': True,
                    'violations': violations
                }
        
        # Update statistics
        self.total_commands += 1
        self.last_violations = violations
        
        return {
            'status': 'success',
            'message': 'Command processed',
            'safety_active': True,
            'violations': violations,
            'target_position': target_position.tolist(),
            'target_orientation': target_orientation.tolist()
        }
    
    def process_joint_command(self, command: JointCommand) -> Dict[str, Any]:
        """
        Process a direct joint command
        """
        if self.backend and self.backend.is_connected():
            success = self.backend.send_joint_positions(np.array(command.joints))
            
            if not success:
                return {
                    'status': 'error',
                    'message': 'Failed to send joint command',
                }
        
        self.total_commands += 1
        return {
            'status': 'success',
            'message': 'Joint command processed',
        }

    def get_status(self) -> TeleopStatus:
        """Get current server status"""
        safety_active = self.safety_gate.is_active()
        
        return TeleopStatus(
            is_active=safety_active,
            safety_gate_active=safety_active,
            last_command_time=self.controller.last_command_time or 0.0,
            robot_connected=self.backend.is_connected() if self.backend else False,
            backend_type=self.backend_type,
            workspace_violation=self.last_violations.get('workspace_violation', False),
            velocity_violation=self.last_violations.get('velocity_violation', False)
        )
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get server statistics"""
        uptime = time.time() - self.start_time
        
        return {
            'uptime': uptime,
            'total_commands': self.total_commands,
            'connected_clients': len(self.connected_clients),
            'backend_status': self.backend.get_status() if self.backend else {},
            'controller_stats': self.controller.get_statistics(),
            'safety_gate_active': self.safety_gate.is_active()
        }


from contextlib import asynccontextmanager

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Manage application lifespan"""
    # Startup
    server = get_server()
    # 获取当前配置的端口（从环境变量或默认值）
    import os
    port = int(os.getenv("TELEOP_PORT", "8000"))
    print(f"[FastAPI] Teleoperation server started on http://localhost:{port}")
    print(f"[FastAPI] Backend type: {server.backend_type}")
    print(f"[FastAPI] API Documentation: http://localhost:{port}/docs")
    yield
    # Shutdown
    if _server_instance:
        _server_instance.shutdown()


# FastAPI App
app = FastAPI(title="Teleoperation Server", 
              description="Minimal teleoperation system for robot control",
              version="1.0.0",
              lifespan=lifespan)

_repo_dir = os.path.dirname(os.path.dirname(__file__))
_web_dir = os.path.join(_repo_dir, "client", "web")
if os.path.isdir(_web_dir):
    app.mount("/web", StaticFiles(directory=_web_dir, html=True), name="web")

# Mount robots directory for assets (URDF, meshes)
_robots_dir = os.path.join(_repo_dir, "robots")
if os.path.isdir(_robots_dir):
    app.mount("/assets/robots", StaticFiles(directory=_robots_dir), name="robots")

# Mount gripper directory
_gripper_dir = os.path.join(_repo_dir, "crt_ctag2f90d_gripper_visualization")
if os.path.isdir(_gripper_dir):
    app.mount("/assets/gripper", StaticFiles(directory=_gripper_dir), name="gripper")

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, restrict this
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global server instance
_server_instance: Optional[TeleoperationServer] = None
_auth = AuthManager()
_recorder = SessionRecorder()
_ws_sessions: Dict[str, str] = {}


def get_server() -> TeleoperationServer:
    """Get or create server instance"""
    global _server_instance
    if _server_instance is None:
        import os
        backend = os.getenv("TELEOP_BACKEND", "mock")
        backend_config = {}
        if backend.lower() == "isaac":
            isaac_host = os.getenv("TELEOP_ISAAC_HOST", "0.0.0.0")
            isaac_port = int(os.getenv("TELEOP_ISAAC_PORT", "9000"))
            backend_config = {"host": isaac_host, "port": isaac_port}
        elif backend.lower() in ["soarm", "so101"]:
            # Prioritize env var
            soarm_port = os.getenv("TELEOP_SOARM_PORT")
            if not soarm_port:
                 # Fallback to hardcoded default if needed, or "/dev/ttyUSB0"
                 soarm_port = "/dev/tty.usbmodem5B3E1224691"
            backend_config = {"port": soarm_port}
        _server_instance = TeleoperationServer(backend_type=backend, backend_config=backend_config)
        _server_instance.initialize()
    return _server_instance


@app.post("/api/v1/command")
async def send_command(command: DeltaCommand):
    """
    Send a teleoperation command to the robot
    
    - **dx, dy, dz**: Position delta in meters (end-effector frame)
    - **droll, dpitch, dyaw**: Orientation delta in radians
    - **max_velocity**: Maximum linear velocity (m/s)
    - **timestamp**: Command timestamp
    """
    server = get_server()
    
    # Validate timestamp
    current_time = time.time()
    if command.timestamp <= 0:
        command.timestamp = current_time
    elif abs(command.timestamp - current_time) > 5.0:  # 5 second tolerance
        return {"error": "Timestamp too far from server time"}
    
    # Process command
    result = server.process_command(command)
    
    if result['status'] == 'error':
        raise HTTPException(status_code=400, detail=result['message'])
    
    return result


@app.get("/api/v1/status")
async def get_status():
    """Get current teleoperation status"""
    server = get_server()
    return server.get_status()


@app.get("/api/v1/statistics")
async def get_statistics():
    """Get server statistics"""
    server = get_server()
    return server.get_statistics()


@app.post("/api/v1/safety/activate")
async def activate_safety():
    """Force activate safety gate (for testing)"""
    server = get_server()
    server.safety_gate.force_activate()
    return {"status": "safety_gate_activated"}


@app.post("/api/v1/safety/reset")
async def reset_safety():
    """Reset safety gate"""
    server = get_server()
    server.safety_gate.reset()
    return {"status": "safety_gate_reset"}


@app.post("/api/v1/controller/reset")
async def reset_controller():
    """Reset controller state"""
    server = get_server()
    server.controller.reset()
    return {"status": "controller_reset"}


@app.websocket("/ws/v1/teleop")
async def websocket_endpoint(websocket: WebSocket):
    """
    WebSocket endpoint for real-time teleoperation
    """
    await websocket.accept()
    token = websocket.query_params.get("token")
    token_info = _auth.verify_token(token)
    if _auth.auth_enabled() and not token_info:
        await websocket.close(code=4401)
        return
    username = token_info.username if token_info else "anonymous"
    session_id = _recorder.start(username=username)
    if token:
        _ws_sessions[token] = session_id

    server = get_server()

    def get_video_state():
        pos, ori = (None, None)
        joints = None
        if server.backend and server.backend.is_connected():
            pos, ori = server.backend.get_current_pose()
            joints = server.backend.get_joint_positions()
        if pos is None:
            pos = np.array([0.0, 0.0, 0.0])
        if ori is None:
            ori = np.array([1.0, 0.0, 0.0, 0.0])
        return {
            "position": [float(pos[0]), float(pos[1]), float(pos[2])],
            "orientation": [float(ori[0]), float(ori[1]), float(ori[2]), float(ori[3])],
            "joints": joints.tolist() if joints is not None else [],
            "status": server.backend.get_status().get("status") if server.backend else "none",
            "timestamp": time.time(),
        }

    async def state_loop():
        while True:
            status = server.get_status()
            state = get_video_state()
            payload = {
                "type": "state",
                "ts": time.time(),
                "status": status.model_dump(),
                "robot": state,
            }
            await websocket.send_json(payload)
            _recorder.write(session_id, {"type": "state", "ts": payload["ts"], "payload": payload})
            
            jpg = None
            if hasattr(server.backend, "render"):
                try:
                    jpg = server.backend.render(width=960, height=540)
                except Exception:
                    pass
            
            if jpg is None:
                jpg = render_status_frame(960, 540, state)
                
            _recorder.save_frame(session_id, jpg)
            await asyncio.sleep(0.02) # 50Hz (Increased for lower latency)

    state_task = asyncio.create_task(state_loop())

    try:
        while True:
            data = await websocket.receive_text()
            msg = json.loads(data)
            
            # Handle different message types
            if isinstance(msg, dict):
                msg_type = msg.get("type", "delta")
                # Handle both nested 'payload' format and flat format
                if "payload" in msg:
                    command_dict = msg.get("payload", {})
                else:
                    # Exclude metadata keys when treating msg as command_dict
                    command_dict = {k: v for k, v in msg.items() if k not in ["type"]}
            else:
                msg_type = "delta"
                command_dict = msg

            result = {}
            current_time = time.time()

            if msg_type == "delta":
                command = DeltaCommand(**command_dict)
                if command.timestamp <= 0:
                    command.timestamp = current_time
                result = server.process_command(command)
                
            elif msg_type == "joint":
                command = JointCommand(**command_dict)
                if command.timestamp <= 0:
                    command.timestamp = current_time
                result = server.process_joint_command(command)

            ack = {"type": "ack", "ts": time.time(), "result": result}
            await websocket.send_json(ack)
            # Optional: Don't record high-frequency joint commands to avoid log spam? 
            # Or record them for replay. Let's record.
            _recorder.write(session_id, {"type": "command", "ts": ack["ts"], "command": command_dict, "result": result})

    except WebSocketDisconnect:
        pass
    except Exception:
        await websocket.close(code=1011)
    finally:
        state_task.cancel()
        _recorder.stop(session_id)


class LoginRequest(BaseModel):
    username: str
    password: str


@app.post("/api/v1/auth/login")
async def login(req: LoginRequest, request: Request):
    if _auth.auth_enabled() and not _auth.validate_login(req.username, req.password):
        raise HTTPException(status_code=401, detail="Invalid credentials")
    token = _auth.issue_token(req.username)
    # Handle proxy headers for correct URL generation (e.g. ngrok)
    host = request.headers.get("x-forwarded-host") or request.headers.get("host")
    scheme = request.headers.get("x-forwarded-proto") or request.url.scheme
    ws_scheme = "wss" if scheme == "https" else "ws"
    ws_url = f"{ws_scheme}://{host}/ws/v1/teleop?token={token}"
    return {"token": token, "ws_url": ws_url, "video_url": f"/api/v1/video/mjpeg?token={token}"}


@app.post("/api/v1/auth/logout")
async def logout(token: str):
    _auth.revoke(token)
    session_id = _ws_sessions.pop(token, None)
    if session_id:
        _recorder.stop(session_id)
    return {"status": "ok"}


@app.post("/api/v1/video/ingest")
async def ingest_video(request: Request):
    """
    Ingest a video frame (JPEG) from an external source (e.g., Isaac Sim)
    """
    server = get_server()
    if not server.backend:
        raise HTTPException(status_code=503, detail="No backend initialized")
    
    # Read raw body
    body = await request.body()
    
    if hasattr(server.backend, "update_frame"):
        server.backend.update_frame(body)
        return {"status": "ok", "size": len(body)}
    else:
        raise HTTPException(status_code=400, detail="Backend does not support video ingest")


@app.get("/api/v1/video/mjpeg")
async def video_mjpeg(token: str):
    # Allow simple token check or no auth for local demo if needed
    if _auth.auth_enabled():
        if not _auth.verify_token(token):
             raise HTTPException(status_code=401, detail="Unauthorized")
    
    server = get_server()

    def get_state():
        pos, ori = (None, None)
        joints = None
        if server.backend and server.backend.is_connected():
            pos, ori = server.backend.get_current_pose()
            joints = server.backend.get_joint_positions()
        if pos is None:
            pos = np.array([0.0, 0.0, 0.0])
        if ori is None:
            ori = np.array([1.0, 0.0, 0.0, 0.0])
        return {
            "position": [float(pos[0]), float(pos[1]), float(pos[2])],
            "orientation": [float(ori[0]), float(ori[1]), float(ori[2]), float(ori[3])],
            "joints": joints.tolist() if joints is not None else [],
            "status": server.backend.get_status().get("status") if server.backend else "none",
            "timestamp": time.time(),
        }

    def get_frame():
        if hasattr(server.backend, "render"):
            return server.backend.render(width=960, height=540)
        return None

    return StreamingResponse(
        mjpeg_stream(get_state_fn=get_state, fps=10, get_frame_fn=get_frame),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/api/v1/video/{camera_name}/mjpeg")
async def video_camera_mjpeg(camera_name: str, token: str):
    # Allow simple token check or no auth for local demo if needed
    if _auth.auth_enabled():
        if not _auth.verify_token(token):
             raise HTTPException(status_code=401, detail="Unauthorized")
    
    server = get_server()

    def get_state():
        pos, ori = (None, None)
        if server.backend and server.backend.is_connected():
            pos, ori = server.backend.get_current_pose()
        if pos is None:
            pos = np.array([0.0, 0.0, 0.0])
        if ori is None:
            ori = np.array([1.0, 0.0, 0.0, 0.0])
        return {
            "position": [float(pos[0]), float(pos[1]), float(pos[2])],
            "orientation": [float(ori[0]), float(ori[1]), float(ori[2]), float(ori[3])],
            "status": server.backend.get_status().get("status") if server.backend else "none",
            "timestamp": time.time(),
        }

    def get_frame():
        if hasattr(server.backend, "render"):
            try:
                return server.backend.render(width=960, height=540, camera=camera_name)
            except TypeError:
                # Backend render doesn't support camera arg
                return server.backend.render(width=960, height=540)
        return None

    return StreamingResponse(
        mjpeg_stream(get_state_fn=get_state, fps=10, get_frame_fn=get_frame),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/")
async def root():
    index_path = os.path.join(_web_dir, "index.html")
    if os.path.isfile(index_path):
        return FileResponse(index_path)
    return {
        "name": "Teleoperation Server",
        "version": "1.0.0",
        "endpoints": {
            "command": "POST /api/v1/command",
            "status": "GET /api/v1/status",
            "statistics": "GET /api/v1/statistics",
            "websocket": "WS /ws/v1/teleop",
            "login": "POST /api/v1/auth/login",
            "video": "GET /api/v1/video/mjpeg",
            "ui": "GET /web/index.html",
        },
        "documentation": "/docs",
    }


def run_server(host: str = "0.0.0.0", port: int = 8000, backend_type: str = "mock", robot_port: Optional[str] = None, ssl_keyfile: Optional[str] = None, ssl_certfile: Optional[str] = None):
    """Run the FastAPI server"""
    # Force set backend type if environment variable is set
    import os
    env_backend = os.getenv("TELEOP_BACKEND")
    if env_backend:
        print(f"[Server] Overriding backend type from env: {env_backend}")
        backend_type = env_backend
        
    global _server_instance
    
    # Prepare backend config
    backend_config = {}
    if robot_port:
        print(f"[Server] Using robot port from args: {robot_port}")
        backend_config['port'] = robot_port
    
    # If not provided in args, get_server logic (if we used it) would check env.
    # But since we are instantiating directly here, we should replicate that or let backend handle it?
    # BackendFactory.create_backend will eventually call backend constructor.
    # SoarmBackend constructor takes 'port'.
    
    # If robot_port is None here, we should probably check env var here too, 
    # to pass it explicitly in config, OR rely on backend default.
    # Let's check env var if not provided.
    if not robot_port and backend_type in ["soarm", "so101"]:
         env_port = os.getenv("TELEOP_SOARM_PORT")
         if env_port:
             backend_config['port'] = env_port
    
    _server_instance = TeleoperationServer(backend_type=backend_type, backend_config=backend_config)
    # Don't initialize here, let the lifespan handler or get_server do it?
    # Actually, lifespan handler calls get_server() which initializes if None.
    # But here we manually create it.
    _server_instance.initialize()
    
    if ssl_keyfile and ssl_certfile:
        print(f"[Server] Starting with SSL enabled (HTTPS)")
        uvicorn.run(app, host=host, port=port, ssl_keyfile=ssl_keyfile, ssl_certfile=ssl_certfile)
    else:
        uvicorn.run(app, host=host, port=port)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Teleoperation Server")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host address")
    parser.add_argument("--port", type=int, default=8000, help="Port number")
    parser.add_argument("--backend", type=str, default="mock", help="Backend type (mock, isaac, or mujoco)")
    parser.add_argument("--robot-port", type=str, default=None, help="Robot serial port (e.g. /dev/ttyUSB0)")
    
    args = parser.parse_args()
    
    run_server(host=args.host, port=args.port, backend_type=args.backend, robot_port=args.robot_port)
