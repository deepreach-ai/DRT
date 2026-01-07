#!/bin/bash
# 验证Isaac Sim WebRTC配置

set -e

echo "🔍 Isaac Sim WebRTC诊断工具"
echo "================================"
echo ""

# 检查容器运行状态
echo "1️⃣ 检查容器状态..."
if docker ps | grep -q isaac-sim; then
    echo "  ✅ isaac-sim容器正在运行"
else
    echo "  ❌ isaac-sim容器未运行"
    echo "  请运行: docker-compose -f docker-compose.full.yml up -d"
    exit 1
fi

echo ""

# 检查端口监听
echo "2️⃣ 检查端口监听..."
PORTS=("8899" "8011" "47998" "49100" "8000" "9000")
for port in "${PORTS[@]}"; do
    if netstat -tuln 2>/dev/null | grep -q ":$port "; then
        echo "  ✅ 端口 $port 正在监听"
    else
        echo "  ⚠️  端口 $port 未监听"
    fi
done

echo ""

# 获取公网IP
echo "3️⃣ 获取EC2公网IP..."
PUBLIC_IP=$(curl -s http://169.254.169.254/latest/meta-data/public-ipv4 2>/dev/null || echo "未检测到")
echo "  公网IP: $PUBLIC_IP"

echo ""

# 测试本地连接
echo "4️⃣ 测试本地WebRTC连接..."
if curl -s -o /dev/null -w "%{http_code}" http://localhost:8899 | grep -q "200\|404\|302"; then
    echo "  ✅ 本地8899端口可访问"
else
    echo "  ❌ 本地8899端口无法访问"
fi

echo ""

# 检查Isaac Sim日志
echo "5️⃣ 最近的Isaac Sim日志 (最后20行):"
echo "-----------------------------------"
docker logs --tail 20 isaac-sim 2>&1 | grep -i -E "livestream|webrtc|streaming|8899|error|warning" || echo "  无相关日志"
echo "-----------------------------------"

echo ""

# 显示访问URL
echo "6️⃣ WebRTC访问地址:"
echo "-----------------------------------"
if [ "$PUBLIC_IP" != "未检测到" ]; then
    echo "  🌐 外部访问: http://$PUBLIC_IP:8899"
    echo "  🎥 WebRTC客户端: http://$PUBLIC_IP:8899/streaming/webrtc-client/"
fi
echo "  🏠 本地访问: http://localhost:8899"
echo "  🎥 本地WebRTC: http://localhost:8899/streaming/webrtc-client/"
echo "-----------------------------------"

echo ""

# 提供故障排除建议
echo "7️⃣ 故障排除建议:"
echo "-----------------------------------"
echo "  如果无法访问:"
echo "  1. 检查AWS安全组是否开放8899端口"
echo "  2. 检查Isaac Sim日志: docker logs -f isaac-sim"
echo "  3. 重启容器: docker-compose -f docker-compose.full.yml restart isaac-sim"
echo "  4. 查看完整日志: docker logs isaac-sim | grep -i livestream"
echo ""
echo "  如果页面空白或无视频:"
echo "  1. 确保使用Chrome或Edge浏览器"
echo "  2. 允许WebRTC权限（摄像头/音频）"
echo "  3. 检查浏览器控制台错误 (F12)"
echo "  4. 尝试刷新页面或清除缓存"
echo "-----------------------------------"

echo ""
echo "✅ 诊断完成"
