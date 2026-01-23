#!/usr/bin/env python3
"""
Run script for teleoperation server
"""
import argparse
import sys
import os

# Add server directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'server'))

try:
    from teleop_server import run_server, TeleoperationServer
except ImportError:
    from server.teleop_server import run_server, TeleoperationServer


def main():
    parser = argparse.ArgumentParser(description="Teleoperation Server")
    parser.add_argument("--host", default="0.0.0.0", help="Host to bind to")
    parser.add_argument("--port", type=int, default=8000, help="Port to listen on")
    parser.add_argument("--backend", default="mock", 
                       choices=["mock", "isaac", "mujoco"], help="Backend type")
    parser.add_argument("--mujoco-xml", help="Path to MuJoCo XML file")
    parser.add_argument("--mujoco-ee", help="Name of the end-effector site")
    parser.add_argument("--mujoco-camera", help="MuJoCo camera name (e.g. world_cam)")
    parser.add_argument("--no-server", action="store_true",
                       help="Initialize only, don't run server")
    
    args = parser.parse_args()

    # Set environment variables for MuJoCo backend
    if args.mujoco_xml:
        os.environ["TELEOP_MUJOCO_XML"] = os.path.abspath(args.mujoco_xml)
    if args.mujoco_ee:
        os.environ["TELEOP_MUJOCO_EE_SITE"] = args.mujoco_ee
    if args.mujoco_camera:
        os.environ["TELEOP_MUJOCO_CAMERA"] = args.mujoco_camera
    
    if args.no_server:
        # Just initialize and test
        server = TeleoperationServer(backend_type=args.backend)
        server.initialize()
        print(f"Server initialized with {args.backend} backend")
        print("Press Ctrl+C to exit...")
        try:
            while True:
                import time
                time.sleep(1)
        except KeyboardInterrupt:
            server.shutdown()
    else:
        # Run the full server
        print(f"Starting teleoperation server on {args.host}:{args.port}")
        print(f"Using backend: {args.backend}")
        run_server(host=args.host, port=args.port, backend_type=args.backend)


if __name__ == "__main__":
    main()
