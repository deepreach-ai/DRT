#!/bin/bash
# Quick start script for SO-ARM101 local testing

set -e

echo "================================"
echo "SO-ARM101 Local Server Startup"
echo "================================"
echo ""

# Default values
DEFAULT_PORT="/dev/ttyUSB0"
USB_PORT=${1:-$DEFAULT_PORT}
HOST="0.0.0.0"
SERVER_PORT="8000"

echo "Configuration:"
echo "  USB Port: $USB_PORT"
echo "  Server Host: $HOST"
echo "  Server Port: $SERVER_PORT"
echo ""

# Check if USB device exists
if [ ! -e "$USB_PORT" ]; then
    echo "⚠️  Warning: USB port $USB_PORT not found!"
    echo ""
    echo "Available ports:"
    ls /dev/tty* | grep -E "USB|ACM" || echo "  No USB devices found"
    echo ""
    read -p "Do you want to continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Aborted."
        exit 1
    fi
fi

# Check USB permissions
if [ -e "$USB_PORT" ] && [ ! -r "$USB_PORT" ]; then
    echo "⚠️  Warning: No read permission for $USB_PORT"
    echo "Run: sudo chmod 666 $USB_PORT"
    echo "Or: sudo usermod -a -G dialout $USER (then re-login)"
    echo ""
    exit 1
fi

# Check if port is already in use
if lsof -Pi :$SERVER_PORT -sTCP:LISTEN -t >/dev/null 2>&1 ; then
    echo "❌ Error: Port $SERVER_PORT is already in use"
    echo "Stop the existing server first."
    exit 1
fi

echo "Starting SO-ARM101 server..."
echo ""

# Set environment variables
export TELEOP_BACKEND=soarm
export TELEOP_SOARM_PORT=$USB_PORT
export TELEOP_PORT=$SERVER_PORT

# Start server
python run_server.py \
    --backend soarm \
    --soarm-port "$USB_PORT" \
    --host "$HOST" \
    --port "$SERVER_PORT"
