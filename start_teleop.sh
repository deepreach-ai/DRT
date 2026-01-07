#!/bin/bash
# Simple startup script for teleop system
# Usage: ./start_teleop.sh [mock|mujoco|isaac]

set -e

TELEOP_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$TELEOP_DIR"

BACKEND="${1:-mock}"

echo "=========================================="
echo "  Starting Teleoperation System"
echo "=========================================="
echo ""
echo "Backend: $BACKEND"
echo "Server will run on: http://localhost:8000"
echo "Web UI available at: http://localhost:8000/web/"
echo ""

case $BACKEND in
    mock)
        echo "Starting with Mock backend (no robot needed)..."
        python run_server.py --backend mock --host 0.0.0.0 --port 8000
        ;;
    mujoco)
        echo "Starting with MuJoCo backend (SO-101 simulation)..."
        python run_server.py --backend mujoco \
            --mujoco-xml robots/so101.xml \
            --mujoco-ee gripperframe \
            --host 0.0.0.0 \
            --port 8000
        ;;
    isaac)
        echo "Starting with Isaac Sim backend..."
        python run_server.py --backend isaac --host 0.0.0.0 --port 8000
        ;;
    *)
        echo "Unknown backend: $BACKEND"
        echo "Usage: $0 [mock|mujoco|isaac]"
        exit 1
        ;;
esac
