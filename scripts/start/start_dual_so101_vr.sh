#!/bin/bash
# VR Control Startup Script for DUAL SO-ARM101
# Controls two follower arms simultaneously from Quest 3S

set -e

echo "🤖 Starting DUAL SO-ARM101 VR Teleoperation System"
echo "=================================================="
echo ""

# Configuration - Default ports (change these if needed)
DEFAULT_LEFT="/dev/tty.usbmodem5B3E1187881"
DEFAULT_RIGHT="/dev/tty.usbmodem5B3E1224691"

LEFT_PORT=${1:-$DEFAULT_LEFT}
RIGHT_PORT=${2:-$DEFAULT_RIGHT}
PORT=8000

# Color codes
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Scan for ports if defaults not found
echo "🔍 Scanning for robot ports..."
ls /dev/tty.usbmodem* 2>/dev/null || echo "No usbmodem devices found."
echo ""

# Get local IP
if [[ "$OSTYPE" == "darwin"* ]]; then
    LOCAL_IP=$(ifconfig | grep "inet " | grep -v 127.0.0.1 | awk '{print $2}' | head -n 1)
else
    LOCAL_IP=$(hostname -I | awk '{print $1}')
fi

echo -e "${BLUE}📋 Configuration:${NC}"
echo "   Left Arm Port:  $LEFT_PORT"
echo "   Right Arm Port: $RIGHT_PORT"
echo "   Server Port:    $PORT"
echo "   Local IP:       $LOCAL_IP"
echo ""

export TELEOP_BACKEND=so101_dual
export TELEOP_LEFT_PORT="$LEFT_PORT"
export TELEOP_RIGHT_PORT="$RIGHT_PORT"
export TELEOP_PORT=$PORT

echo -e "${YELLOW}🔧 Starting Dual SO-ARM Backend...${NC}"

python3 run_server.py \
    --backend so101_dual \
    --port $PORT &

SERVER_PID=$!

echo "Waiting for server to initialize..."
sleep 6

if ! ps -p $SERVER_PID > /dev/null; then
    echo "❌ Error: Server failed to start."
    exit 1
fi

echo -e "${GREEN}✅ Dual Server started successfully!${NC}"
echo ""
echo "=================================================="
echo -e "${GREEN}🥽 VR SETUP (DUAL ARM)${NC}"
echo "=================================================="
echo "On your Meta Quest 3S:"
echo "1. 📱 Open Browser"
echo "2. 🌐 Navigate to: http://${LOCAL_IP}:${PORT}/web/vr.html?urdf=so101_dual"
echo "3. 🎮 Left Hand Controller  → Controls Left Arm"
echo "4. 🎮 Right Hand Controller → Controls Right Arm"
echo ""
echo "=================================================="

cleanup() {
    echo ""
    echo "🛑 Shutting down..."
    kill $SERVER_PID 2>/dev/null || true
    exit 0
}

trap cleanup SIGINT SIGTERM
wait $SERVER_PID
