#!/bin/bash
set -e

# Configuration
# Replace these or set them as environment variables
EC2_HOST="${EC2_HOST:-}"
SSH_KEY="${SSH_KEY:-}"
REMOTE_USER="ubuntu"
REMOTE_DIR="~/drt"

# Help message
if [ -z "$EC2_HOST" ] || [ -z "$SSH_KEY" ]; then
    echo "Usage: EC2_HOST=44.255.70.45 SSH_KEY=~/.ssh/1234.pem ./deploy_remote.sh"
    echo "Or edit the script to set defaults."
    exit 1
fi

echo "🚀 Starting deployment to $EC2_HOST..."

# 1. Prepare archive excluding venv, git, etc.
echo "📦 Packaging project..."
tar --exclude='venv' --exclude='.git' --exclude='__pycache__' --exclude='.DS_Store' -czf drt_deploy.tar.gz -C .. .

# 2. Copy archive and setup script
echo "pw Uploading files..."
scp -i "$SSH_KEY" -o StrictHostKeyChecking=no drt_deploy.tar.gz "$REMOTE_USER@$EC2_HOST:~/"
scp -i "$SSH_KEY" -o StrictHostKeyChecking=no ec2_setup.sh "$REMOTE_USER@$EC2_HOST:~/"

# 3. Execute remote setup and deployment
echo "🔧 Executing remote setup..."
ssh -i "$SSH_KEY" -o StrictHostKeyChecking=no "$REMOTE_USER@$EC2_HOST" << EOF
    # Unpack
    mkdir -p $REMOTE_DIR
    tar -xzf ~/drt_deploy.tar.gz -C $REMOTE_DIR
    
    # Make setup script executable
    chmod +x ~/ec2_setup.sh
    
    # Run setup (installs Docker if needed)
    # We run this with bash specifically
    ./ec2_setup.sh
    
    # Start application
    cd $REMOTE_DIR/deployment
    
    echo "🐳 Building and starting containers..."
    # Check if we need sudo for docker (if setup just ran, group might not be active in this session)
    if groups | grep -q docker; then
        docker compose up -d --build
    else
        sudo docker compose up -d --build
    fi
    
    echo "✅ Deployment complete! Server should be running at http://$EC2_HOST:8000"
EOF

# Cleanup
rm drt_deploy.tar.gz
echo "🎉 Done!"
