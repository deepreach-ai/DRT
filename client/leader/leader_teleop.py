"""
Teleoperation Client for Leader Arm (SO-100/SO-101)
Reads joint states from a local Leader Arm and sends them to the Teleop Server.
"""
import argparse
import asyncio
import json
import time
import numpy as np
import websockets
import sys
import os

# Try to import lerobot
try:
    from lerobot.teleoperators.so100_leader.so100_leader import SO100Leader
    from lerobot.teleoperators.so100_leader.config_so100_leader import SO100LeaderConfig
    from lerobot.teleoperators.so101_leader.so101_leader import SO101Leader
    from lerobot.teleoperators.so101_leader.config_so101_leader import SO101LeaderConfig
    LEROBOT_AVAILABLE = True
except ImportError as e:
    print(f"Error importing LeRobot: {e}")
    print("Please ensure 'lerobot' is installed and in your PYTHONPATH.")
    LEROBOT_AVAILABLE = False

async def run_teleop(server_url, leader_port, robot_type="so100", token=""):
    """
    Main teleoperation loop
    """
    if not LEROBOT_AVAILABLE:
        return

    print(f"Initializing {robot_type} Leader on {leader_port}...")
    
    # Initialize Leader Robot
    leader = None
    try:
        if robot_type == "so100":
            config = SO100LeaderConfig(port=leader_port)
            leader = SO100Leader(config)
        elif robot_type == "so101":
            config = SO101LeaderConfig(port=leader_port)
            leader = SO101Leader(config)
        else:
            print(f"Unknown robot type: {robot_type}")
            return

        leader.connect()
        print("Leader connected successfully!")
        
    except Exception as e:
        print(f"Failed to connect to leader robot: {e}")
        return

    # Connect to Server
    # Ensure scheme is ws or wss
    ws_base = server_url
    if ws_base.startswith("http://"):
        ws_base = ws_base.replace("http://", "ws://")
    elif ws_base.startswith("https://"):
        ws_base = ws_base.replace("https://", "wss://")
    elif not ws_base.startswith("ws"):
        ws_base = f"ws://{ws_base}"
        
    # Try to login to get token if not provided
    if not token and ws_base.startswith("ws"):
        # Convert ws/wss to http/https
        auth_url = ws_base.replace("ws", "http", 1)
        print(f"No token provided. Attempting to login to {auth_url}...")
        try:
            import requests
            resp = requests.post(f"{auth_url}/api/v1/auth/login", json={"username": "operator", "password": "operator"})
            if resp.status_code == 200:
                data = resp.json()
                token = data.get("token")
                print(f"Login successful! Token: {token[:8]}...")
            else:
                print(f"Login failed: {resp.text}")
        except Exception as e:
            print(f"Auto-login error: {e}. Please provide token manually if auth is enabled.")

    ws_url = f"{ws_base}/ws/v1/teleop?token={token}"
    print(f"Connecting to server: {ws_url}")
    
    try:
        # Disable ping/pong to avoid timeouts if server is busy or network is laggy
        # Or increase timeout values
        async with websockets.connect(ws_url, ping_interval=None) as websocket:
            print("Connected to server. Starting teleoperation loop...")
            print("Press Ctrl+C to stop.")
            
            # Main Loop
            while True:
                start_time = time.time()
                
                # 1. Read Leader Joints
                if not leader.is_connected:
                    print("Leader disconnected!")
                    break
                
                # Use the bus directly if needed, or the high level method
                # The 'read' method typically reads from the bus
                # data = leader.read("Present_Position")
                
                try:
                    # The bus sync_read returns a dict of {motor_name: value}
                    joints_dict = leader.bus.sync_read("Present_Position")
                    
                    # We need to convert this to an ordered list matching the Follower's expectation
                    sorted_motors = sorted(leader.bus.motors.items(), key=lambda x: x[1].id)
                    joint_values = [int(joints_dict[name]) for name, _ in sorted_motors]
                except Exception as e:
                    print(f"Error reading joints: {e}")
                    # Don't break immediately on one read error, maybe retry?
                    # But if bus is down, we should probably reconnect
                    await asyncio.sleep(0.1)
                    continue
                
                # 2. Send to Server
                cmd = {
                    "type": "joint",
                    "payload": {
                        "joints": joint_values,
                        "timestamp": time.time(),
                        "client_id": "leader_arm"
                    }
                }
                
                await websocket.send(json.dumps(cmd))
                
                # 3. Receive ACK (optional, don't block too long)
                # We can fire and forget, or wait for ack to throttle
                # But accumulating unread messages can fill buffers
                try:
                    # Non-blocking read to clear buffer
                    while True:
                        await asyncio.wait_for(websocket.recv(), timeout=0.001)
                except (asyncio.TimeoutError, asyncio.CancelledError):
                    pass
                
                # Rate limiting (e.g., 50Hz)
                elapsed = time.time() - start_time
                sleep_time = max(0, 0.02 - elapsed)
                await asyncio.sleep(sleep_time)
                
    except websockets.exceptions.ConnectionClosed as e:
        print(f"Connection to server closed: {e}")
    except Exception as e:
        print(f"Error in teleop loop: {e}")
    finally:
        print("Disconnecting leader...")
        leader.disconnect()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Leader Arm Teleoperation Client")
    parser.add_argument("--url", type=str, default="ws://localhost:8000", help="Server URL (e.g., ws://localhost:8000)")
    parser.add_argument("--port", type=str, required=True, help="Leader Arm Serial Port (e.g., /dev/tty.usbmodem...)")
    parser.add_argument("--type", type=str, default="so100", choices=["so100", "so101"], help="Robot Type")
    parser.add_argument("--token", type=str, default="", help="Auth Token")
    
    args = parser.parse_args()
    
    # Adjust URL to http/https base if needed (script expects base url)
    # Actually the script constructs ws url: f"{server_url}/ws/v1/teleop..."
    # So if user passes ws://..., we should handle it
    
    base_url = args.url
    if base_url.startswith("ws"):
        base_url = base_url.replace("ws", "http", 1)
        
    try:
        asyncio.run(run_teleop(base_url, args.port, args.type, args.token))
    except KeyboardInterrupt:
        print("Stopped by user")
