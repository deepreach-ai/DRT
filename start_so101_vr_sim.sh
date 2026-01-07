#!/bin/bash
# VR Control Startup Script for SO-ARM101 in MuJoCo Simulation
# This script starts the teleoperation server with VR support for Quest 3S

set -e

echo "🤖 Starting SO-ARM101 VR Simulation System"
echo "=================================================="
echo ""

# Configuration
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROBOT_XML="${SCRIPT_DIR}/robots/so101.xml"
EE_SITE="gripperframe"
PORT=8000

# Color codes for terminal output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if MuJoCo XML exists
if [ ! -f "$ROBOT_XML" ]; then
    echo "❌ Error: Robot XML file not found at $ROBOT_XML"
    exit 1
fi

# Get local IP address for Quest 3S connection
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS
    LOCAL_IP=$(ifconfig | grep "inet " | grep -v 127.0.0.1 | awk '{print $2}' | head -n 1)
else
    # Linux
    LOCAL_IP=$(hostname -I | awk '{print $1}')
fi

echo -e "${BLUE}📋 Configuration:${NC}"
echo "   Robot Model: SO-ARM101 (5-DOF)"
echo "   XML File: $ROBOT_XML"
echo "   End Effector: $EE_SITE"
echo "   Server Port: $PORT"
echo "   Local IP: $LOCAL_IP"
echo ""

echo -e "${YELLOW}🔧 Starting MuJoCo Backend...${NC}"

# Handle headless rendering for AWS/Linux if needed
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    if [ -z "$DISPLAY" ]; then
        echo "   (Headless environment detected, using EGL rendering)"
        export MUJOCO_GL=egl
    fi
fi

# Start the server with MuJoCo backend
python3 run_server.py \
    --backend mujoco \
    --mujoco-xml "$ROBOT_XML" \
    --mujoco-ee "$EE_SITE" \
    --port $PORT &

SERVER_PID=$!

# Wait for server to start
echo "Waiting for server to initialize..."
sleep 3

# Check if server is running
if ! ps -p $SERVER_PID > /dev/null; then
    echo "❌ Error: Server failed to start"
    exit 1
fi

echo -e "${GREEN}✅ Server started successfully!${NC}"
echo ""
echo "=================================================="
echo -e "${GREEN}🥽 VR SETUP INSTRUCTIONS${NC}"
echo "=================================================="
echo ""
echo "On your Meta Quest 3S:"
echo ""
echo "1. 📱 Open Browser app in Quest"
echo "2. 🌐 Navigate to: http://${LOCAL_IP}:${PORT}/web/vr.html?urdf=so101"
echo "3. 🔐 Login if prompted (default: operator/operator)"
echo "4. 🎮 Click 'Enter VR Mode' button"
echo ""
echo "Alternative (Ngrok - Recommended for Latency/HTTPS):"
echo "1. Ensure ngrok is running on your Mac: ./scripts/run_ngrok.sh"
echo "2. Open the https://...ngrok-free.app/web/vr.html?urdf=so101 URL in Quest"
echo ""
echo "=================================================="
echo -e "${BLUE}📊 Server URLs:${NC}"
echo "=================================================="
echo ""
echo "Web VR Interface: http://${LOCAL_IP}:$PORT/web/vr.html?urdf=so101"
echo "API Endpoint:     http://${LOCAL_IP}:$PORT/api/v1/"
echo ""
echo "=================================================="
echo -e "${GREEN}🎮 CONTROLS${NC}"
echo "=================================================="
echo ""
echo "VR Mode (Quest Controllers):"
echo "  • Right Joystick → Move Arm (X/Y/Z)"
echo "  • Right Grip (Hold) → Clutch (Direct Hand Tracking)"
echo "  • Right Trigger → Close Gripper"
echo "  • B Button (Long Press) → Exit VR"
echo ""
echo "=================================================="

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "🛑 Shutting down..."
    kill $SERVER_PID 2>/dev/null || true
    echo "✅ Server stopped"
    exit 0
}

# Set trap to cleanup on Ctrl+C
trap cleanup SIGINT SIGTERM

echo ""
echo "Press Ctrl+C to stop the server"
echo ""

# Keep script running
wait $SERVER_PID
