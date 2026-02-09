#!/bin/bash
# VR Control Startup Script for REALMAN RM75B in MuJoCo
# This script starts the teleoperation server with VR support for Quest 3S

set -e

echo "🤖 Starting REALMAN RM75B VR Teleoperation System"
echo "=================================================="
echo ""

# Configuration
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROBOT_XML="${SCRIPT_DIR}/robots/rm75b_vr.xml"
EE_SITE="ee_site"
PORT=8000
WEB_PORT=8080

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
echo "   Robot Model: REALMAN RM75B (7-DOF)"
echo "   XML File: $ROBOT_XML"
echo "   End Effector: $EE_SITE"
echo "   Server Port: $PORT"
echo "   Web UI Port: $WEB_PORT"
echo "   Local IP: $LOCAL_IP"
echo ""

echo -e "${YELLOW}🔧 Starting MuJoCo Backend...${NC}"

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
echo "2. 🌐 Navigate to: http://${LOCAL_IP}:${PORT}/web/"
echo "3. 🔐 Login with:"
echo "   Username: operator"
echo "   Password: operator"
echo "4. 🎮 Click 'Enter VR Mode' button"
echo ""
echo "Alternative (USB Tethering):"
echo "1. Connect Quest to Mac via USB cable"
echo "2. Run: adb reverse tcp:$PORT tcp:$PORT"
echo "3. Navigate to: http://localhost:$PORT/web/"
echo ""
echo "=================================================="
echo -e "${BLUE}📊 Server URLs:${NC}"
echo "=================================================="
echo ""
echo "Web Interface:  http://localhost:$PORT/web/"
echo "                http://${LOCAL_IP}:$PORT/web/"
echo ""
echo "API Endpoint:   http://localhost:$PORT/api/v1/"
echo "Statistics:     http://localhost:$PORT/api/v1/statistics"
echo "Video Stream:   http://localhost:$PORT/api/v1/video/mjpeg"
echo ""
echo "=================================================="
echo -e "${GREEN}🎮 CONTROLS${NC}"
echo "=================================================="
echo ""
echo "VR Mode (Quest Controllers):"
echo "  • Right Controller Position → Robot End Effector"
echo "  • Right Trigger → Close Gripper"
echo "  • B Button → Emergency Stop"
echo "  • Menu Button → Exit VR"
echo ""
echo "Keyboard Mode (2D Interface):"
echo "  • Arrow Keys → Move X/Y"
echo "  • Page Up/Down → Move Z"
echo "  • Q/E → Rotate Yaw"
echo "  • G → Toggle Gripper"
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
