#!/bin/bash
# Isaac Sim with Native Streaming (更稳定)

set -e

echo "🚀 Starting Isaac Sim with Native Streaming..."

# 设置环境变量
export OMNI_SERVER="${OMNI_SERVER:-omniverse://localhost}"
export PRIVACY_CONSENT=1
export ACCEPT_EULA=Y

# 检测公网IP
PUBLIC_IP=$(curl -s http://169.254.169.254/latest/meta-data/public-ipv4 2>/dev/null || echo "")

echo "📡 Public IP: ${PUBLIC_IP:-Not detected}"

# 启动Isaac Sim with native streaming
/isaac-sim/isaac-sim.sh \
    --/app/window/hideUi=true \
    --/app/livestream/enabled=true \
    --/app/livestream/proto=ws \
    --/app/livestream/port=8899 \
    --/app/livestream/websocket_port=8011 \
    --no-window \
    --/renderer/enabled=rtx \
    --allow-root \
    --ext-folder /isaac-sim/exts \
    --enable omni.kit.livestream.native \
    --enable omni.services.transport.server.http \
    /workspace/run_isaac_client.py \
        --host "${ISAAC_SIM_HOST:-localhost}" \
        --port "${ISAAC_SIM_PORT:-9000}" \
        --headless
