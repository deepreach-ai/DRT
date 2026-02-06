#!/bin/bash
# Quick deployment script for teleoperation server
# Usage: ./quick_deploy.sh [server_ip]

set -e  # Exit on error

echo "🚀 Teleoperation System - Quick Deploy"
echo "======================================"

# Check if server IP provided
if [ -z "$1" ]; then
    echo "Usage: ./quick_deploy.sh <server_ip>"
    echo "Example: ./quick_deploy.sh 54.123.45.67"
    exit 1
fi

SERVER_IP=$1
SERVER_USER=${SERVER_USER:-ubuntu}  # Default to ubuntu, can override with env var
PROJECT_NAME="drt"

echo ""
echo "📋 Configuration:"
echo "  Server IP: $SERVER_IP"
echo "  Server User: $SERVER_USER"
echo "  Project: $PROJECT_NAME"
echo ""

# Step 1: Package the code
echo "📦 Step 1: Packaging code..."
cd ~/drt
tar -czf /tmp/drt.tar.gz \
    --exclude='.git' \
    --exclude='.venv' \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    --exclude='.DS_Store' \
    .

echo "✓ Code packaged"

# Step 2: Upload to server
echo ""
echo "📤 Step 2: Uploading to server..."
scp /tmp/drt.tar.gz $SERVER_USER@$SERVER_IP:/tmp/
echo "✓ Uploaded"

# Step 3: Deploy on server
echo ""
echo "🔧 Step 3: Deploying on server..."
ssh $SERVER_USER@$SERVER_IP << 'ENDSSH'
    set -e
    
    echo "  • Installing system dependencies..."
    sudo apt-get update -qq
    sudo apt-get install -y -qq python3 python3-pip python3-venv git
    
    echo "  • Creating project directory..."
    mkdir -p ~/drt
    cd ~/drt
    
    echo "  • Extracting code..."
    tar -xzf /tmp/drt.tar.gz
    
    echo "  • Setting up virtual environment..."
    python3 -m venv venv
    source venv/bin/activate
    
    echo "  • Installing Python dependencies..."
    pip install -q --upgrade pip
    pip install -q -r server/requirements.txt
    pip install -q -r client/requirements.txt
    
    echo "  • Creating systemd services..."
    
    # Teleop server service
    sudo tee /etc/systemd/system/teleop-server.service > /dev/null << 'EOF'
[Unit]
Description=Teleoperation Server
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$HOME/drt
Environment="PATH=$HOME/drt/venv/bin"
ExecStart=$HOME/drt/venv/bin/python run_server.py --backend mock --host 0.0.0.0 --port 8000
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

    # Web UI service
    sudo tee /etc/systemd/system/teleop-web.service > /dev/null << 'EOF'
[Unit]
Description=Teleoperation Web UI
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$HOME/drt
Environment="PATH=$HOME/drt/venv/bin"
ExecStart=$HOME/drt/venv/bin/python client/web_server.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

    echo "  • Reloading systemd..."
    sudo systemctl daemon-reload
    
    echo "  • Enabling services..."
    sudo systemctl enable teleop-server
    sudo systemctl enable teleop-web
    
    echo "  • Starting services..."
    sudo systemctl restart teleop-server
    sudo systemctl restart teleop-web
    
    echo "  • Waiting for services to start..."
    sleep 3
    
    echo ""
    echo "  ✅ Deployment complete!"
    echo ""
    echo "  Service status:"
    sudo systemctl status teleop-server --no-pager -l | head -n 5
    sudo systemctl status teleop-web --no-pager -l | head -n 5
ENDSSH

echo ""
echo "✅ Deployment successful!"
echo ""
echo "🌐 Access URLs:"
echo "  Web UI:   http://$SERVER_IP:8080"
echo "  API:      http://$SERVER_IP:8000"
echo "  API Docs: http://$SERVER_IP:8000/docs"
echo ""
echo "🔍 Useful commands:"
echo "  Check server logs:    ssh $SERVER_USER@$SERVER_IP 'sudo journalctl -u teleop-server -f'"
echo "  Check web logs:       ssh $SERVER_USER@$SERVER_IP 'sudo journalctl -u teleop-web -f'"
echo "  Restart server:       ssh $SERVER_USER@$SERVER_IP 'sudo systemctl restart teleop-server'"
echo "  Server status:        ssh $SERVER_USER@$SERVER_IP 'sudo systemctl status teleop-server'"
echo ""
echo "🧪 Test latency:"
echo "  python client/latency_test_client.py --server ws://$SERVER_IP:8000/ws/v1/teleop"
echo ""
