"""
Main teleoperation server using FastAPI
"""
from fastapi import FastAPI, HTTPException, BackgroundTasks, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Dict, List, Optional, Any
import numpy as np
import uvicorn
import asyncio
import threading
import time
from datetime import datetime
import json

from models import DeltaCommand, RobotState, TeleopStatus, WorkspaceLimits
from safety_gate import SafetyGate
from control_logic import TeleoperationController
from robot_backend import RobotBackend, BackendFactory, BackendStatus


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
        
        Args:
            command: Delta command from client
            
        Returns:
            Processing result with status and any violations
        """
        # Update safety gate
        command_array = np.array([command.dx, command.dy, command.dz, 
                                  command.droll, command.dpitch, command.dyaw])
        safety_active = self.safety_gate.update(command_array, command.timestamp)
        
        if not safety_active:
            return {
                'status': 'error',
                'message': 'Safety gate not active',
                'safety_active': False,
                'violations': {}
            }
        
        # Process command through controller
        target_position, target_orientation, violations = self.controller.process_command(command)
        
        # Send to robot backend
        if self.backend and self.backend.is_connected():
            success = self.backend.send_target_pose(
                target_position, 
                target_orientation,
                velocity_limit=command.max_velocity
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


def get_server() -> TeleoperationServer:
    """Get or create server instance"""
    global _server_instance
    if _server_instance is None:
        import os
        backend = os.getenv("TELEOP_BACKEND", "mock")
        _server_instance = TeleoperationServer(backend_type=backend)
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
    server = get_server()
    
    try:
        while True:
            # Receive command
            data = await websocket.receive_text()
            command_dict = json.loads(data)
            command = DeltaCommand(**command_dict)
            
            # Process command
            result = server.process_command(command)
            
            # Send response
            await websocket.send_json(result)
            
    except WebSocketDisconnect:
        print(f"[WebSocket] Client disconnected")
    except Exception as e:
        print(f"[WebSocket] Error: {e}")
        await websocket.close(code=1011)


@app.get("/")
async def root():
    """Root endpoint with API information"""
    return {
        "name": "Teleoperation Server",
        "version": "1.0.0",
        "endpoints": {
            "command": "POST /api/v1/command",
            "status": "GET /api/v1/status",
            "statistics": "GET /api/v1/statistics",
            "websocket": "WS /ws/v1/teleop"
        },
        "documentation": "/docs"
    }


def run_server(host: str = "0.0.0.0", port: int = 8000, backend_type: str = "mock"):
    """Run the FastAPI server"""
    global _server_instance
    _server_instance = TeleoperationServer(backend_type=backend_type)
    _server_instance.initialize()
    uvicorn.run(app, host=host, port=port)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Teleoperation Server")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host address")
    parser.add_argument("--port", type=int, default=8000, help="Port number")
    parser.add_argument("--backend", type=str, default="mock", help="Backend type (mock or isaac)")
    
    args = parser.parse_args()
    
    run_server(host=args.host, port=args.port, backend_type=args.backend)
