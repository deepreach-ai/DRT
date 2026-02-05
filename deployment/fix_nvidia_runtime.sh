#!/bin/bash
set -e

echo "🔧 Fixing NVIDIA Docker Runtime..."

# 1. Configure NVIDIA Runtime for Docker
echo "⚙️  Configuring docker daemon..."
sudo nvidia-ctk runtime configure --runtime=docker

# 2. Restart Docker Daemon
echo "🔄 Restarting docker..."
sudo systemctl restart docker

# 3. Verify
echo "🔍 Verifying..."
if docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi &>/dev/null; then
    echo "✅ NVIDIA Runtime verified!"
else
    # Fallback check - sometimes --runtime=nvidia isn't needed if it's default, 
    # but the error explicitly said runtime 'nvidia' was missing.
    if docker run --rm --gpus all ubuntu nvidia-smi &>/dev/null; then
         echo "✅ GPU Access verified (without explicit runtime flag)!"
    else
         echo "❌ Still failing. Please check 'sudo systemctl status docker' and 'nvidia-smi'."
         exit 1
    fi
fi

echo "🚀 You can now run: docker compose -f docker-compose.full.yml up -d"
