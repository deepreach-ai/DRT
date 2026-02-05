#!/bin/bash
# start_vr_simulation.sh
# One-click script to start the full VR-Isaac Sim pipeline

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}==============================================${NC}"
echo -e "${BLUE}   🚀 Starting VR Teleoperation System       ${NC}"
echo -e "${BLUE}==============================================${NC}"

# Check for arguments
ROBOT_NAME=${1:-"realman_rm65"}  # Default to realman if not specified
USD_PATH="assets/usd/${ROBOT_NAME}.usd"
EE_FRAME="link_6"

if [ "$ROBOT_NAME" == "lingyu_robot" ]; then
    USD_PATH="assets/usd/lingyu_robot.usd"
    EE_FRAME="end_effector"
elif [ "$ROBOT_NAME" == "realman_rm65" ]; then
    # Correct filename for Realman based on Isaac Sim assets
    USD_PATH="assets/usd/RM65-6F.usd" 
fi

echo -e "${GREEN}1. Starting Teleop Server (Port 8000)...${NC}"
# Start server in background
python3 run_server.py --host 0.0.0.0 --backend isaac > server.log 2>&1 &
SERVER_PID=$!
echo "   Server PID: $SERVER_PID"
sleep 3

echo -e "${GREEN}2. Starting Isaac Sim Client (Headless + Livestream)...${NC}"
echo "   Robot: $ROBOT_NAME"
echo "   USD: $USD_PATH"

# Function to find Isaac python
find_isaac_python() {
    # Check ISAAC_SIM_PATH env var first
    if [ ! -z "$ISAAC_SIM_PATH" ] && [ -f "$ISAAC_SIM_PATH/python.sh" ]; then
        echo "$ISAAC_SIM_PATH/python.sh"
        return
    fi

    if [ -f "isaac-sim/python.sh" ]; then
        echo "isaac-sim/python.sh"
    elif [ -f "$HOME/.local/share/ov/pkg/isaac_sim-4.0.0/python.sh" ]; then
        echo "$HOME/.local/share/ov/pkg/isaac_sim-4.0.0/python.sh"
    elif [ -f "$HOME/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh" ]; then
        echo "$HOME/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh"
    else
        # Try generic find if directory exists
        if [ -d "$HOME/.local/share/ov/pkg" ]; then
            find $HOME/.local/share/ov/pkg -name "python.sh" | head -n 1
        fi
    fi
}

ISAAC_PYTHON=$(find_isaac_python)

if [ -z "$ISAAC_PYTHON" ]; then
    echo "❌ Error: Could not find Isaac Sim python.sh"
    kill $SERVER_PID
    exit 1
fi

echo "   Using: $ISAAC_PYTHON"

# Run Isaac Sim
$ISAAC_PYTHON isaac_sim_client.py \
    --robot_usd "$USD_PATH" \
    --ee_frame "$EE_FRAME" \
    --host localhost \
    --headless \
    --enable-livestream

# Cleanup on exit
kill $SERVER_PID
echo -e "${BLUE}System stopped.${NC}"
