#!/bin/bash
# Upload MuJoCo assets to AWS
SERVER_IP=$1
if [ -z "$SERVER_IP" ]; then
    echo "Usage: ./upload_assets.sh <AWS_IP>"
    exit 1
fi

echo "📦 Uploading MuJoCo assets to $SERVER_IP..."

# Ensure target directory exists
ssh -o StrictHostKeyChecking=no ubuntu@$SERVER_IP "mkdir -p ~/drt/robots"

# Upload XMLs and assets folder
scp -o StrictHostKeyChecking=no -r robots/so101.xml robots/teleop_scene.xml robots/assets/ ubuntu@$SERVER_IP:~/drt/robots/

echo "✅ Assets uploaded successfully."
