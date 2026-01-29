#!/bin/bash
# VR Teleoperation Quick Start Script

echo "🥽 VR Teleoperation Setup for Quest 3S"
echo "======================================"
echo ""

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "${YELLOW}Step 1: Check Prerequisites${NC}"
echo "- Quest 3S charged and powered on"
echo "- Quest connected to same WiFi as Mac"
echo "- Developer mode enabled on Quest"
echo ""
read -p "Prerequisites met? (y/n) " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
    echo "Please complete prerequisites first!"
    exit 1
fi

echo ""
echo "${GREEN}✓ Prerequisites OK${NC}"
echo ""

echo "${YELLOW}Step 2: Get Mac IP Address${NC}"
MAC_IP=$(ifconfig | grep "inet " | grep -v 127.0.0.1 | awk '{print $2}' | head -n 1)
echo "Your Mac IP: ${GREEN}$MAC_IP${NC}"
echo ""

echo "${YELLOW}Step 3: Start Web Server${NC}"
echo "Starting simple HTTP server on port 8080..."
cd "$(dirname "$0")/../client/web"
python3 -m http.server 8080 &
WEB_PID=$!
echo "Web server PID: $WEB_PID"
echo ""
sleep 2

echo "${GREEN}✓ Web server started${NC}"
echo ""

echo "${YELLOW}Step 4: Start Teleoperation Server${NC}"
echo "Starting FastAPI server..."
cd "$(dirname "$0")/.."

# Check if server is already running
if lsof -Pi :8000 -sTCP:LISTEN -t >/dev/null ; then
    echo "Server already running on port 8000"
else
    # Start server in background
    python3 run_server.py --backend mock --port 8000 &
    SERVER_PID=$!
    echo "Server PID: $SERVER_PID"
    sleep 3
fi

echo "${GREEN}✓ Teleoperation server started${NC}"
echo ""

echo "======================================"
echo "${GREEN}🚀 READY FOR VR!${NC}"
echo "======================================"
echo ""
echo "On your Quest 3S:"
echo "1. Open the Browser app"
echo "2. Navigate to: ${GREEN}http://$MAC_IP:8080/vr.html${NC}"
echo "3. Enter server URL: ${GREEN}http://$MAC_IP:8000${NC}"
echo "4. Click 'Connect to Server'"
echo "5. Click 'Enter VR Mode'"
echo ""
echo "Press Ctrl+C to stop all servers"
echo ""

# Keep script running
trap "echo 'Stopping servers...'; kill $WEB_PID 2>/dev/null; exit" INT
wait
