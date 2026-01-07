#!/bin/bash
# EC2部署脚本 - 同时启动Teleop Server和Isaac Sim

set -e

echo "🚀 Starting Teleoperation System with Isaac Sim..."

# 检查NVIDIA Docker支持
if ! docker run --rm --gpus all nvidia/cuda:12.1.0-base-ubuntu22.04 nvidia-smi &>/dev/null; then
    echo "❌ NVIDIA Docker support not available"
    echo "Please run: sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker"
    exit 1
fi

# 设置环境变量
export TELEOP_BACKEND=isaac
export TELEOP_ISAAC_HOST=0.0.0.0
export TELEOP_ISAAC_PORT=9000

# 进入deployment目录
cd "$(dirname "$0")"

# 拉取Isaac Sim镜像（如果不存在）
if [[ "$(docker images -q nvcr.io/nvidia/isaac-sim:4.0.0 2> /dev/null)" == "" ]]; then
    echo "📥 Pulling Isaac Sim Docker image (this may take 10-20 minutes)..."
    docker pull nvcr.io/nvidia/isaac-sim:4.0.0
fi

# 停止旧容器
echo "🛑 Stopping old containers..."
docker-compose -f docker-compose.full.yml down 2>/dev/null || true

# 启动服务
echo "▶️  Starting services..."
docker-compose -f docker-compose.full.yml up -d

# 等待服务启动
echo "⏳ Waiting for services to start..."
sleep 5

# 检查状态
echo ""
echo "📊 Service Status:"
docker-compose -f docker-compose.full.yml ps

echo ""
echo "✅ System started!"
echo ""
echo "🔗 Access points:"
echo "  - Teleop API: http://localhost:8000/docs"
echo "  - Health Check: http://localhost:8000/"
echo "  - Status: http://localhost:8000/api/v1/status"
echo ""
echo "📝 View logs:"
echo "  - Teleop Server: docker logs -f teleop-server"
echo "  - Isaac Sim: docker logs -f isaac-sim"
echo ""
echo "🛑 To stop: docker-compose -f docker-compose.full.yml down"
