#!/bin/bash
# VR Control Startup Script for REAL SO-ARM101
# This script starts the teleoperation server with VR support for Quest 3S
# and connects to the physical robot.

set -e

echo "🤖 Starting REAL SO-ARM101 VR Teleoperation System"
echo "=================================================="
echo ""

# Configuration
# Default port for SO-ARM101 on macOS
DEFAULT_PORT="/dev/tty.usbmodem5B3E1224691"
USB_PORT=${1:-$DEFAULT_PORT}
PORT=8000

# Color codes for terminal output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if USB device exists
if [ ! -e "$USB_PORT" ]; then
    echo -e "${YELLOW}⚠️  Warning: USB port $USB_PORT not found!${NC}"
    echo "Available ports:"
    ls /dev/tty* | grep -E "USB|ACM|usbmodem" || echo "  No USB devices found"
    echo ""
    # We don't exit here because the user might have provided a symlink or 
    # the backend might have better detection.
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
echo "   Robot Model: SO-ARM101 (Real Hardware)"
echo "   USB Port: $USB_PORT"
echo "   Server Port: $PORT"
echo "   Local IP: $LOCAL_IP"
echo ""

echo -e "${YELLOW}🔧 Starting SO-ARM Backend...${NC}"

# Start the server with soarm backend
python3 run_server.py \
    --backend soarm \
    --soarm-port "$USB_PORT" \
    --port $PORT &

SERVER_PID=$!

# Wait for server to start
echo "Waiting for server to initialize..."
sleep 5

# Check if server is running
if ! ps -p $SERVER_PID > /dev/null; then
    echo "❌ Error: Server failed to start. Check if the robot is connected and the port is correct."
    exit 1
fi

echo -e "${GREEN}✅ Server started successfully!${NC}"
echo ""
echo "=================================================="
echo -e "${GREEN}🥽 VR SETUP INSTRUCTIONS${NC}"
echo "=================================================="
echo ""
echo "On your Meta Quest 3S:"
echo "1. 📱 Open Browser app in Quest"
echo "2. 🌐 Navigate to: http://${LOCAL_IP}:${PORT}/web/vr.html?urdf=so101"
echo "3. 🔐 Login if prompted (default: operator/operator)"
echo "4. 🎮 Click 'Enter VR Mode' button"
echo ""
echo "Alternative (USB Tethering):"
echo "1. Connect Quest to Mac via USB cable"
echo "2. Run: adb reverse tcp:$PORT tcp:$PORT"
echo "3. Navigate to: http://localhost:$PORT/web/vr.html?urdf=so101"
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
echo "  • Grip Button (Hold) → Clutch (Direct Hand Tracking)"
echo "  • Right Trigger → Close Gripper"
echo "  • B Button → Emergency Stop"
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
