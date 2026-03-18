#!/bin/bash
# Quick Local Test Script - Simplest way to verify system
# One-click test for SO-ARM101 setup

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}"
echo "╔═══════════════════════════════════════════════════════╗"
echo "║        🚀 SO-ARM101 Quick Local Test                  ║"
echo "╚═══════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo ""

# Step 1: Hardware detection
echo -e "${GREEN}[1/4] Running hardware detection...${NC}"
python3 detect_hardware.py

# Ask to continue
echo ""
read -p "Hardware detection complete. Continue to start system? (y/N): " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Cancelled"
    exit 0
fi

# Step 2: Select USB port
echo ""
echo -e "${GREEN}[2/4] Select robot arm port${NC}"
echo ""

# Auto-detect ports
PORTS=($(ls /dev/tty* 2>/dev/null | grep -E "USB|ACM" || echo ""))

if [ ${#PORTS[@]} -eq 0 ]; then
    echo -e "${YELLOW}No USB ports detected${NC}"
    read -p "Enter port manually (e.g. /dev/ttyUSB0): " USB_PORT
elif [ ${#PORTS[@]} -eq 1 ]; then
    USB_PORT=${PORTS[0]}
    echo "Auto-selected: $USB_PORT"
else
    echo "Multiple ports detected:"
    for i in "${!PORTS[@]}"; do
        echo "  $((i+1))) ${PORTS[$i]}"
    done
    echo ""
    read -p "Select (1-${#PORTS[@]}) [1]: " PORT_CHOICE
    PORT_CHOICE=${PORT_CHOICE:-1}
    USB_PORT=${PORTS[$((PORT_CHOICE-1))]}
    echo "Selected: $USB_PORT"
fi

# Step 3: Test connection
echo ""
echo -e "${GREEN}[3/4] Testing robot arm connection...${NC}"
echo ""

# Modify test script port
if [ -f "test_soarm_integration.py" ]; then
    sed -i.bak "s|port='.*'|port='$USB_PORT'|" test_soarm_integration.py
    python3 test_soarm_integration.py

    if [ $? -ne 0 ]; then
        echo ""
        echo -e "${YELLOW}⚠️  Connection test failed${NC}"
        read -p "Continue to start server anyway? (y/N): " -n 1 -r
        echo ""
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
else
    echo -e "${YELLOW}Test script not found, skipping connection test${NC}"
fi

# Step 4: Start server
echo ""
echo -e "${GREEN}[4/4] Starting server...${NC}"
echo ""

export LEFT_ARM_PORT="$USB_PORT"
export MAIN_SERVER_PORT=8000

# Start server
python3 run_server.py \
    --backend soarm \
    --soarm-port "$USB_PORT" \
    --host 0.0.0.0 \
    --port 8000 > server.log 2>&1 &

SERVER_PID=$!

# Wait for startup
sleep 3

# Check if successful
if ! ps -p $SERVER_PID > /dev/null 2>&1; then
    echo -e "${RED}❌ Server failed to start${NC}"
    echo "View logs:"
    tail server.log
    exit 1
fi

# Get IP
LOCAL_IP=$(hostname -I | awk '{print $1}' || echo "localhost")

# Success message
echo ""
echo -e "${GREEN}╔═══════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║              ✅ Startup Successful!                    ║${NC}"
echo -e "${GREEN}╚═══════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}Access URLs:${NC}"
echo "  📱 Local: http://localhost:8000"
echo "  🌐 Network: http://$LOCAL_IP:8000"
echo "  🥽 VR Mode: http://$LOCAL_IP:8000/vr.html"
echo ""
echo -e "${BLUE}Controls:${NC}"
echo "  1. Open browser at above URL"
echo "  2. Click 'Connect' to connect robot"
echo "  3. Press '1' to activate safety gate"
echo "  4. Use W/A/S/D/Q/E to control movement"
echo "  5. Use G/H to control gripper"
echo ""
echo -e "${YELLOW}Tip: Press Ctrl+C to stop server${NC}"
echo ""

# Ask to open browser
if command -v xdg-open &> /dev/null; then
    read -p "Open browser automatically? (y/N): " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        xdg-open "http://localhost:8000" 2>/dev/null &
    fi
fi

# Keep running
trap "
    echo ''
    echo -e '${YELLOW}Stopping server...${NC}'
    kill $SERVER_PID 2>/dev/null || true
    echo -e '${GREEN}✓ Stopped${NC}'
    exit 0
" INT TERM

echo -e "${GREEN}Server running...${NC}"
echo ""

# Show live logs
tail -f server.log 2>/dev/null &
TAIL_PID=$!

wait $SERVER_PID

# Cleanup
kill $TAIL_PID 2>/dev/null || true
echo ""
echo -e "${BLUE}Server stopped${NC}"
