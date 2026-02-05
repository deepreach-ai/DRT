#!/bin/bash
# 完整本地系统启动脚本
# Complete Local System Startup Script
# 支持: 双臂SO-ARM101 + 多相机 + VR

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 配置默认值
LEFT_ARM_PORT="${LEFT_ARM_PORT:-/dev/ttyUSB0}"
RIGHT_ARM_PORT="${RIGHT_ARM_PORT:-/dev/ttyUSB1}"
SERVER_HOST="${SERVER_HOST:-0.0.0.0}"
LEFT_ARM_SERVER_PORT="${LEFT_ARM_SERVER_PORT:-8001}"
RIGHT_ARM_SERVER_PORT="${RIGHT_ARM_SERVER_PORT:-8002}"
MAIN_SERVER_PORT="${MAIN_SERVER_PORT:-8000}"

echo -e "${BLUE}"
echo "╔════════════════════════════════════════════════════════╗"
echo "║   🚀 SO-ARM101 完整系统启动 / Complete System Startup  ║"
echo "╚════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# 函数: 打印步骤
print_step() {
    echo -e "${GREEN}[步骤 $1/$2]${NC} $3"
}

# 函数: 打印错误
print_error() {
    echo -e "${RED}❌ 错误:${NC} $1"
}

# 函数: 打印警告
print_warning() {
    echo -e "${YELLOW}⚠️  警告:${NC} $1"
}

# 函数: 检查端口是否被占用
check_port() {
    if lsof -Pi :$1 -sTCP:LISTEN -t >/dev/null 2>&1 ; then
        return 0  # 端口被占用
    else
        return 1  # 端口空闲
    fi
}

# 步骤1: 硬件检测
print_step 1 6 "硬件检测..."
echo ""

# 检测串口设备
echo -e "${BLUE}检测机械臂...${NC}"
if [ ! -e "$LEFT_ARM_PORT" ]; then
    print_warning "左臂端口 $LEFT_ARM_PORT 不存在"
    echo "可用端口:"
    ls /dev/tty* 2>/dev/null | grep -E "USB|ACM" || echo "  无"
fi

if [ ! -e "$RIGHT_ARM_PORT" ]; then
    print_warning "右臂端口 $RIGHT_ARM_PORT 不存在"
    echo "提示: 如果只有一个臂，设置 RIGHT_ARM_PORT=none"
fi

# 检测相机
echo -e "${BLUE}检测相机...${NC}"
video_devices=$(ls /dev/video* 2>/dev/null | wc -l || echo "0")
echo "  找到 $video_devices 个视频设备"

# 步骤2: 检查权限
print_step 2 6 "检查权限..."
echo ""

if [ -e "$LEFT_ARM_PORT" ] && [ ! -r "$LEFT_ARM_PORT" ]; then
    print_error "无法读取 $LEFT_ARM_PORT"
    echo "请运行: sudo chmod 666 $LEFT_ARM_PORT"
    echo "或: sudo usermod -a -G dialout $USER (然后重新登录)"
    exit 1
fi

# 步骤3: 检查端口占用
print_step 3 6 "检查端口占用..."
echo ""

if check_port $MAIN_SERVER_PORT; then
    print_error "端口 $MAIN_SERVER_PORT 已被占用"
    echo "请停止占用该端口的进程，或使用其他端口:"
    echo "  export MAIN_SERVER_PORT=8080"
    exit 1
fi

# 步骤4: 询问启动模式
print_step 4 6 "选择启动模式..."
echo ""

echo "请选择启动模式:"
echo "  1) 单臂模式 (推荐先测试)"
echo "  2) 双臂模式 (需要两个SO-ARM101)"
echo "  3) 仿真模式 (Mock backend，无需硬件)"
echo ""
read -p "请输入选项 (1/2/3) [1]: " MODE_CHOICE
MODE_CHOICE=${MODE_CHOICE:-1}

case $MODE_CHOICE in
    1)
        echo -e "${GREEN}✓${NC} 单臂模式"
        MODE="single"
        ;;
    2)
        echo -e "${GREEN}✓${NC} 双臂模式"
        MODE="dual"
        ;;
    3)
        echo -e "${GREEN}✓${NC} 仿真模式"
        MODE="mock"
        ;;
    *)
        print_error "无效选项"
        exit 1
        ;;
esac

# 步骤5: 启动服务器
print_step 5 6 "启动服务器..."
echo ""

# 清理旧的日志
rm -f server.log server_left.log server_right.log

case $MODE in
    "single")
        echo -e "${BLUE}启动单臂服务器...${NC}"
        echo "  端口: $LEFT_ARM_PORT"
        echo "  服务器端口: $MAIN_SERVER_PORT"

        python3 run_server.py \
            --backend soarm \
            --soarm-port "$LEFT_ARM_PORT" \
            --host "$SERVER_HOST" \
            --port "$MAIN_SERVER_PORT" \
            > server.log 2>&1 &

        SERVER_PID=$!
        echo "  PID: $SERVER_PID"
        ;;

    "dual")
        echo -e "${BLUE}启动双臂服务器...${NC}"
        echo "  左臂: $LEFT_ARM_PORT -> :$LEFT_ARM_SERVER_PORT"
        echo "  右臂: $RIGHT_ARM_PORT -> :$RIGHT_ARM_SERVER_PORT"

        # 启动左臂服务器
        python3 run_server.py \
            --backend soarm \
            --soarm-port "$LEFT_ARM_PORT" \
            --host "$SERVER_HOST" \
            --port "$LEFT_ARM_SERVER_PORT" \
            > server_left.log 2>&1 &
        LEFT_PID=$!
        echo "  左臂 PID: $LEFT_PID"

        sleep 2

        # 启动右臂服务器
        python3 run_server.py \
            --backend soarm \
            --soarm-port "$RIGHT_ARM_PORT" \
            --host "$SERVER_HOST" \
            --port "$RIGHT_ARM_SERVER_PORT" \
            > server_right.log 2>&1 &
        RIGHT_PID=$!
        echo "  右臂 PID: $RIGHT_PID"

        SERVER_PID="$LEFT_PID,$RIGHT_PID"
        ;;

    "mock")
        echo -e "${BLUE}启动仿真服务器...${NC}"
        echo "  模式: Mock (无需硬件)"
        echo "  服务器端口: $MAIN_SERVER_PORT"

        python3 run_server.py \
            --backend mock \
            --host "$SERVER_HOST" \
            --port "$MAIN_SERVER_PORT" \
            > server.log 2>&1 &

        SERVER_PID=$!
        echo "  PID: $SERVER_PID"
        ;;
esac

# 等待服务器启动
echo ""
echo -e "${BLUE}等待服务器启动...${NC}"
sleep 3

# 步骤6: 验证启动
print_step 6 6 "验证启动状态..."
echo ""

# 检查进程是否运行
if [ "$MODE" = "dual" ]; then
    if ! ps -p $LEFT_PID > /dev/null 2>&1; then
        print_error "左臂服务器启动失败"
        echo "查看日志: tail server_left.log"
        exit 1
    fi
    if ! ps -p $RIGHT_PID > /dev/null 2>&1; then
        print_error "右臂服务器启动失败"
        echo "查看日志: tail server_right.log"
        exit 1
    fi
    echo -e "${GREEN}✓${NC} 双臂服务器运行中"
else
    if ! ps -p $SERVER_PID > /dev/null 2>&1; then
        print_error "服务器启动失败"
        echo "查看日志: tail server.log"
        exit 1
    fi
    echo -e "${GREEN}✓${NC} 服务器运行中"
fi

# 获取本机IP
LOCAL_IP=$(hostname -I | awk '{print $1}' || echo "localhost")

echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║              ✅ 系统启动成功！                          ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}访问方式:${NC}"
echo ""

if [ "$MODE" = "dual" ]; then
    echo "  📱 本机浏览器:"
    echo "     左臂: http://localhost:$LEFT_ARM_SERVER_PORT"
    echo "     右臂: http://localhost:$RIGHT_ARM_SERVER_PORT"
    echo ""
    echo "  🌐 其他设备 (Quest 3S等):"
    echo "     左臂: http://$LOCAL_IP:$LEFT_ARM_SERVER_PORT"
    echo "     右臂: http://$LOCAL_IP:$RIGHT_ARM_SERVER_PORT"
    echo ""
    echo "  🥽 VR模式:"
    echo "     左臂: http://$LOCAL_IP:$LEFT_ARM_SERVER_PORT/vr.html"
    echo "     右臂: http://$LOCAL_IP:$RIGHT_ARM_SERVER_PORT/vr.html"
else
    echo "  📱 本机浏览器:"
    echo "     http://localhost:$MAIN_SERVER_PORT"
    echo ""
    echo "  🌐 其他设备 (Quest 3S等):"
    echo "     http://$LOCAL_IP:$MAIN_SERVER_PORT"
    echo ""
    echo "  🥽 VR模式:"
    echo "     http://$LOCAL_IP:$MAIN_SERVER_PORT/vr.html"
    echo ""
    echo "  📊 API文档:"
    echo "     http://localhost:$MAIN_SERVER_PORT/docs"
fi

echo ""
echo -e "${BLUE}控制说明:${NC}"
echo "  1. 打开浏览器访问上述地址"
echo "  2. 点击 'Connect' 按钮"
echo "  3. 按键盘 '1' 激活安全门"
echo "  4. 使用键盘或虚拟摇杆控制:"
echo "     W/S: 前后  |  A/D: 左右  |  Q/E: 上下"
echo "     G/H: 夹爪开关"
echo ""

echo -e "${BLUE}日志查看:${NC}"
if [ "$MODE" = "dual" ]; then
    echo "  tail -f server_left.log   # 左臂日志"
    echo "  tail -f server_right.log  # 右臂日志"
else
    echo "  tail -f server.log"
fi
echo ""

echo -e "${BLUE}停止服务器:${NC}"
echo "  按 Ctrl+C"
echo ""

# 如果使用ngrok，提示设置
if command -v ngrok &> /dev/null; then
    echo -e "${YELLOW}💡 提示: 如需从外网访问，可以使用 ngrok:${NC}"
    echo "     ngrok http $MAIN_SERVER_PORT"
    echo ""
fi

echo -e "${BLUE}═════════════════════════════════════════════════════════${NC}"
echo ""

# 等待用户中断
trap "
    echo ''
    echo -e '${YELLOW}正在停止服务器...${NC}'
    if [ '$MODE' = 'dual' ]; then
        kill $LEFT_PID $RIGHT_PID 2>/dev/null || true
    else
        kill $SERVER_PID 2>/dev/null || true
    fi
    echo -e '${GREEN}✓ 已停止${NC}'
    exit 0
" INT TERM

# 保持脚本运行
echo -e "${GREEN}服务器运行中... 按 Ctrl+C 停止${NC}"
echo ""

# 持续监控服务器状态
while true; do
    sleep 5

    if [ "$MODE" = "dual" ]; then
        if ! ps -p $LEFT_PID > /dev/null 2>&1; then
            print_error "左臂服务器已停止"
            echo "查看日志: tail server_left.log"
            break
        fi
        if ! ps -p $RIGHT_PID > /dev/null 2>&1; then
            print_error "右臂服务器已停止"
            echo "查看日志: tail server_right.log"
            break
        fi
    else
        if ! ps -p $SERVER_PID > /dev/null 2>&1; then
            print_error "服务器已停止"
            echo "查看日志: tail server.log"
            break
        fi
    fi
done

# 清理
if [ "$MODE" = "dual" ]; then
    kill $LEFT_PID $RIGHT_PID 2>/dev/null || true
else
    kill $SERVER_PID 2>/dev/null || true
fi

echo ""
echo -e "${BLUE}系统已停止${NC}"
