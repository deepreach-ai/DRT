#!/bin/bash
# Quick Start Script for Teleop System Validation
# Run this to test the system locally

set -e  # Exit on error

TELEOP_DIR="/Users/ziguo/teleop_system"
cd "$TELEOP_DIR"

echo "======================================"
echo "  TELEOP SYSTEM QUICK VALIDATION"
echo "======================================"
echo ""

# Step 1: Check prerequisites
echo "Step 1: Running prerequisite checks..."
python3 validate_prereqs.py
if [ $? -ne 0 ]; then
    echo "❌ Prerequisite checks failed. Please fix issues above."
    exit 1
fi

echo ""
echo "======================================"
echo "  Ready to start validation tests!"
echo "======================================"
echo ""
echo "Choose a test to run:"
echo ""
echo "1) Test Mock Backend (no robot needed)"
echo "2) Test MuJoCo Backend (SO-101 robot simulation)"
echo "3) Test Web UI + Mock Backend"
echo "4) Test Web UI + MuJoCo Backend"
echo "5) Test Keyboard Client"
echo "6) Run all API tests"
echo "7) Exit"
echo ""
read -p "Enter choice [1-7]: " choice

case $choice in
    1)
        echo ""
        echo "Starting Mock Backend Server..."
        echo "Press Ctrl+C to stop"
        echo ""
        python run_server.py --backend mock
        ;;
    2)
        echo ""
        echo "Starting MuJoCo Backend Server..."
        echo "Using robot model: robots/so101.xml"
        echo "Press Ctrl+C to stop"
        echo ""
        python run_server.py --backend mujoco \
            --mujoco-xml robots/so101.xml \
            --mujoco-ee gripperframe
        ;;
    3)
        echo ""
        echo "Starting Mock Backend + Web UI..."
        echo "This will open 2 terminals"
        echo ""
        echo "Terminal 1: Teleop Server (Mock)"
        osascript -e 'tell app "Terminal" to do script "cd '"$TELEOP_DIR"' && python run_server.py --backend mock"'
        sleep 3
        echo "Terminal 2: Web UI Server"
        osascript -e 'tell app "Terminal" to do script "cd '"$TELEOP_DIR"' && python client/web_server.py"'
        sleep 2
        echo ""
        echo "✅ Servers started!"
        echo "📱 Open browser: http://localhost:8080"
        echo ""
        ;;
    4)
        echo ""
        echo "Starting MuJoCo Backend + Web UI..."
        echo "This will open 2 terminals"
        echo ""
        echo "Terminal 1: Teleop Server (MuJoCo)"
        osascript -e 'tell app "Terminal" to do script "cd '"$TELEOP_DIR"' && python run_server.py --backend mujoco --mujoco-xml robots/so101.xml --mujoco-ee gripperframe"'
        sleep 3
        echo "Terminal 2: Web UI Server"
        osascript -e 'tell app "Terminal" to do script "cd '"$TELEOP_DIR"' && python client/web_server.py"'
        sleep 2
        echo ""
        echo "✅ Servers started!"
        echo "📱 Open browser: http://localhost:8080"
        echo ""
        ;;
    5)
        echo ""
        echo "Make sure teleop server is running first!"
        echo "Then run keyboard client..."
        echo "Press Ctrl+C to stop"
        echo ""
        python client/keyboard_client.py
        ;;
    6)
        echo ""
        echo "Running API tests..."
        echo ""
        
        # Check if server is running
        if ! curl -s http://localhost:8000/health > /dev/null; then
            echo "❌ Server not running on port 8000"
            echo "Start server first: python run_server.py --backend mock"
            exit 1
        fi
        
        echo "Test 1: Health check"
        curl -s http://localhost:8000/health | python3 -m json.tool
        echo ""
        
        echo "Test 2: Statistics"
        curl -s http://localhost:8000/api/v1/statistics | python3 -m json.tool
        echo ""
        
        echo "Test 3: Activate safety"
        curl -s -X POST http://localhost:8000/api/v1/safety/activate | python3 -m json.tool
        echo ""
        
        echo "Test 4: Send delta command"
        curl -s -X POST http://localhost:8000/api/v1/command/delta \
            -H "Content-Type: application/json" \
            -d '{"dx":0.01,"dy":0,"dz":0,"droll":0,"dpitch":0,"dyaw":0,"max_velocity":0.1,"max_angular_velocity":0.5,"timestamp":0,"client_id":"test"}' \
            | python3 -m json.tool
        echo ""
        
        echo "✅ All API tests completed!"
        ;;
    7)
        echo "Goodbye!"
        exit 0
        ;;
    *)
        echo "Invalid choice"
        exit 1
        ;;
esac
