# SO-ARM101 部署指南 / SO-ARM101 Deployment Guide

## 目录 / Table of Contents

1. [本地测试 / Local Testing](#1-本地测试--local-testing)
2. [AWS部署 / AWS Deployment](#2-aws部署--aws-deployment)
3. [美国连接测试 / USA Connection Test](#3-美国连接测试--usa-connection-test)
4. [故障排除 / Troubleshooting](#4-故障排除--troubleshooting)

---

## 1. 本地测试 / Local Testing

### 1.1 检查USB连接 / Check USB Connection

```bash
# 查找SO-ARM101的USB端口 / Find SO-ARM101 USB port
ls /dev/tty* | grep -E "USB|ACM"

# 常见端口名称 / Common port names:
# - Linux: /dev/ttyUSB0, /dev/ttyACM0
# - macOS: /dev/tty.usbmodem*
# - Windows: COM3, COM4, etc.
```

### 1.2 测试机械臂连接 / Test Robot Connection

```bash
cd ~/Teleop_platform

# 使用正确的端口测试连接 / Test connection with correct port
python test_soarm_integration.py
```

**如果需要修改端口 / If you need to change the port:**
```bash
# 编辑test_soarm_integration.py，修改第24行 / Edit test_soarm_integration.py, line 24
port='/dev/ttyUSB0',  # 改为你的实际端口 / Change to your actual port
```

### 1.3 启动本地服务器 / Start Local Server

```bash
# 方法1：使用命令行参数 / Method 1: Using command line args
python run_server.py --backend soarm --soarm-port /dev/ttyUSB0 --host 0.0.0.0 --port 8000

# 方法2：使用环境变量 / Method 2: Using environment variables
export TELEOP_BACKEND=soarm
export TELEOP_SOARM_PORT=/dev/ttyUSB0
export TELEOP_PORT=8000
python run_server.py

# 方法3：使用快速启动脚本 / Method 3: Using quick start script
./start_soarm_local.sh
```

### 1.4 测试本地控制 / Test Local Control

打开浏览器 / Open browser:
```
http://localhost:8000
```

或使用Web UI / Or use Web UI:
```
http://localhost:8080
```

**测试步骤 / Test steps:**
1. 点击"Connect" / Click "Connect"
2. 激活安全门 (按键1) / Activate safety gate (press key 1)
3. 使用键盘或虚拟摇杆控制 / Use keyboard or virtual joystick to control
4. 观察机械臂是否响应 / Check if robot responds

---

## 2. AWS部署 / AWS Deployment

### 2.1 前提条件 / Prerequisites

- [ ] SO-ARM101机械臂连接到部署机器 / SO-ARM101 connected to deployment machine
- [ ] USB端口权限配置 / USB port permissions configured
- [ ] Python 3.8+ 已安装 / Python 3.8+ installed
- [ ] 网络端口8000开放 / Network port 8000 open

### 2.2 配置USB权限 / Configure USB Permissions

```bash
# 在部署机器上 / On deployment machine
# 查找USB设备 / Find USB device
lsusb

# 添加用户到dialout组 / Add user to dialout group
sudo usermod -a -G dialout $USER

# 或者设置udev规则 / Or set udev rules
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", MODE="0666"' | sudo tee /etc/udev/rules.d/99-soarm.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# 重新登录以应用组变化 / Re-login to apply group changes
```

### 2.3 快速部署到AWS / Quick Deploy to AWS

#### 选项A：自动部署脚本 / Option A: Automated Script

```bash
# 创建部署脚本 / Create deployment script
cat > deploy_soarm_aws.sh << 'EOF'
#!/bin/bash
set -e

echo "================================"
echo "SO-ARM101 AWS Deployment Script"
echo "================================"

# 检查参数 / Check arguments
if [ -z "$1" ]; then
    echo "Usage: ./deploy_soarm_aws.sh <AWS_IP> [USB_PORT]"
    echo "Example: ./deploy_soarm_aws.sh 54.123.45.67 /dev/ttyUSB0"
    exit 1
fi

AWS_IP=$1
USB_PORT=${2:-/dev/ttyUSB0}
SSH_KEY=~/.ssh/teleop-aws.pem

echo "AWS IP: $AWS_IP"
echo "USB Port: $USB_PORT"
echo ""

# 打包代码 / Package code
echo "1. Packaging code..."
tar -czf teleop_deploy.tar.gz \
    --exclude='.git' \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    --exclude='venv' \
    server/ client/ run_server.py requirements.txt

# 上传到AWS / Upload to AWS
echo "2. Uploading to AWS..."
scp -i $SSH_KEY teleop_deploy.tar.gz ubuntu@$AWS_IP:~/

# 安装和配置 / Install and configure
echo "3. Setting up on AWS..."
ssh -i $SSH_KEY ubuntu@$AWS_IP << ENDSSH
    # 解压 / Extract
    tar -xzf teleop_deploy.tar.gz

    # 安装依赖 / Install dependencies
    sudo apt-get update
    sudo apt-get install -y python3-pip python3-venv

    # 创建虚拟环境 / Create venv
    python3 -m venv venv
    source venv/bin/activate

    # 安装Python包 / Install packages
    pip install -r requirements.txt
    pip install -r server/requirements.txt

    # 配置USB权限 / Configure USB permissions
    sudo usermod -a -G dialout ubuntu

    # 创建systemd服务 / Create systemd service
    sudo tee /etc/systemd/system/teleop-soarm.service > /dev/null << 'EOFSERVICE'
[Unit]
Description=Teleoperation Server - SO-ARM101
After=network.target

[Service]
Type=simple
User=ubuntu
WorkingDirectory=/home/ubuntu
Environment="PATH=/home/ubuntu/venv/bin"
Environment="TELEOP_BACKEND=soarm"
Environment="TELEOP_SOARM_PORT=$USB_PORT"
Environment="TELEOP_PORT=8000"
ExecStart=/home/ubuntu/venv/bin/python run_server.py --backend soarm --soarm-port $USB_PORT --host 0.0.0.0 --port 8000
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOFSERVICE

    # 启动服务 / Start service
    sudo systemctl daemon-reload
    sudo systemctl enable teleop-soarm
    sudo systemctl start teleop-soarm

    echo "✓ Deployment complete!"
ENDSSH

# 清理 / Cleanup
rm teleop_deploy.tar.gz

echo ""
echo "================================"
echo "✓ Deployment successful!"
echo "================================"
echo ""
echo "Service status:"
ssh -i $SSH_KEY ubuntu@$AWS_IP 'sudo systemctl status teleop-soarm --no-pager'
echo ""
echo "Access points:"
echo "  API: http://$AWS_IP:8000/docs"
echo "  WebSocket: ws://$AWS_IP:8000/ws/v1/teleop"
echo ""
echo "View logs:"
echo "  ssh -i $SSH_KEY ubuntu@$AWS_IP 'sudo journalctl -u teleop-soarm -f'"
EOF

chmod +x deploy_soarm_aws.sh

# 执行部署 / Execute deployment
./deploy_soarm_aws.sh 44.254.63.252 /dev/ttyUSB0
```

#### 选项B：手动部署 / Option B: Manual Deployment

```bash
# 1. SSH连接到AWS / SSH to AWS
ssh -i ~/.ssh/teleop-aws.pem ubuntu@44.254.63.252

# 2. 安装依赖 / Install dependencies
sudo apt-get update
sudo apt-get install -y python3-pip python3-venv git

# 3. 克隆或上传代码 / Clone or upload code
git clone YOUR_REPO teleop_platform
# OR
# scp -r ~/Teleop_platform ubuntu@44.254.63.252:~/

# 4. 进入项目目录 / Enter project directory
cd ~/teleop_platform

# 5. 创建虚拟环境 / Create virtual environment
python3 -m venv venv
source venv/bin/activate

# 6. 安装Python包 / Install Python packages
pip install -r server/requirements.txt

# 7. 配置USB权限 / Configure USB permissions
sudo usermod -a -G dialout $USER
# 重新登录 / Re-login

# 8. 查找USB端口 / Find USB port
ls /dev/tty* | grep -E "USB|ACM"

# 9. 测试连接 / Test connection
python test_soarm_integration.py

# 10. 启动服务器 / Start server
python run_server.py --backend soarm --soarm-port /dev/ttyUSB0 --host 0.0.0.0 --port 8000
```

### 2.4 配置为系统服务 / Configure as System Service

```bash
# 创建systemd服务文件 / Create systemd service file
sudo nano /etc/systemd/system/teleop-soarm.service
```

**服务文件内容 / Service file content:**
```ini
[Unit]
Description=Teleoperation Server - SO-ARM101
After=network.target

[Service]
Type=simple
User=ubuntu
WorkingDirectory=/home/ubuntu/teleop_platform
Environment="PATH=/home/ubuntu/teleop_platform/venv/bin"
Environment="TELEOP_BACKEND=soarm"
Environment="TELEOP_SOARM_PORT=/dev/ttyUSB0"
Environment="TELEOP_PORT=8000"
ExecStart=/home/ubuntu/teleop_platform/venv/bin/python run_server.py --backend soarm --soarm-port /dev/ttyUSB0 --host 0.0.0.0 --port 8000
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

**启动服务 / Start service:**
```bash
sudo systemctl daemon-reload
sudo systemctl enable teleop-soarm
sudo systemctl start teleop-soarm

# 检查状态 / Check status
sudo systemctl status teleop-soarm

# 查看日志 / View logs
sudo journalctl -u teleop-soarm -f
```

---

## 3. 美国连接测试 / USA Connection Test

### 3.1 基础连接测试 / Basic Connection Test

```bash
# 测试网络延迟 / Test network latency
ping 44.254.63.252

# 测试端口连通性 / Test port connectivity
nc -zv 44.254.63.252 8000

# 或使用telnet / Or use telnet
telnet 44.254.63.252 8000
```

### 3.2 WebSocket连接测试 / WebSocket Connection Test

**使用Python客户端 / Using Python client:**

```bash
cd ~/Teleop_platform/client

# 安装客户端依赖 / Install client dependencies
pip install websockets requests

# 运行延迟测试 / Run latency test
python latency_test_client.py \
    --server ws://44.254.63.252:8000/ws/v1/teleop?token=YOUR_TOKEN \
    --samples 200
```

**使用Web浏览器 / Using Web browser:**

```
http://44.254.63.252:8000
```

### 3.3 VR头显连接测试 / VR Headset Connection Test

**Quest 3设置 / Quest 3 Setup:**

1. 确保Quest 3和电脑在同一网络 / Ensure Quest 3 and PC are on same network
2. 在Quest浏览器中打开 / Open in Quest browser:
   ```
   http://44.254.63.252:8000
   ```
3. 进入VR模式 / Enter VR mode
4. 测试控制延迟 / Test control latency

### 3.4 性能监控 / Performance Monitoring

```bash
# SSH到AWS服务器 / SSH to AWS server
ssh -i ~/.ssh/teleop-aws.pem ubuntu@44.254.63.252

# 监控CPU和内存使用 / Monitor CPU and memory usage
top

# 监控服务状态 / Monitor service status
watch -n 1 'systemctl status teleop-soarm --no-pager | tail -20'

# 实时查看日志 / View logs in real-time
sudo journalctl -u teleop-soarm -f
```

---

## 4. 故障排除 / Troubleshooting

### 4.1 USB连接问题 / USB Connection Issues

**问题：找不到USB设备 / Issue: USB device not found**

```bash
# 检查USB设备 / Check USB devices
lsusb
dmesg | tail -20

# 检查权限 / Check permissions
ls -l /dev/ttyUSB0

# 添加权限 / Add permissions
sudo chmod 666 /dev/ttyUSB0
# OR
sudo usermod -a -G dialout $USER
```

**问题：权限被拒绝 / Issue: Permission denied**

```bash
# 临时解决 / Temporary fix
sudo chmod 666 /dev/ttyUSB0

# 永久解决 / Permanent fix
sudo tee /etc/udev/rules.d/99-usb-serial.rules << EOF
SUBSYSTEM=="tty", MODE="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 4.2 服务启动失败 / Service Start Failure

```bash
# 查看详细日志 / View detailed logs
sudo journalctl -u teleop-soarm -n 100 --no-pager

# 检查配置 / Check configuration
systemctl cat teleop-soarm

# 手动运行以查看错误 / Run manually to see errors
cd ~/teleop_platform
source venv/bin/activate
python run_server.py --backend soarm --soarm-port /dev/ttyUSB0
```

### 4.3 连接超时 / Connection Timeout

**检查防火墙 / Check firewall:**

```bash
# 检查AWS安全组 / Check AWS Security Group
# 确保端口8000已开放 / Ensure port 8000 is open

# 检查Ubuntu防火墙 / Check Ubuntu firewall
sudo ufw status
sudo ufw allow 8000/tcp
sudo ufw reload
```

### 4.4 机械臂不响应 / Robot Not Responding

```bash
# 1. 检查服务状态 / Check service status
sudo systemctl status teleop-soarm

# 2. 检查USB连接 / Check USB connection
ls -l /dev/ttyUSB*

# 3. 重启服务 / Restart service
sudo systemctl restart teleop-soarm

# 4. 检查日志中的错误 / Check logs for errors
sudo journalctl -u teleop-soarm -n 50

# 5. 测试直接连接 / Test direct connection
cd ~/teleop_platform
source venv/bin/activate
python test_soarm_integration.py
```

### 4.5 高延迟问题 / High Latency Issues

**诊断步骤 / Diagnostic steps:**

```bash
# 1. 测试网络延迟 / Test network latency
ping -c 100 44.254.63.252

# 2. 测试带宽 / Test bandwidth
# 使用iperf3或speedtest-cli

# 3. 检查服务器负载 / Check server load
ssh -i ~/.ssh/teleop-aws.pem ubuntu@44.254.63.252 'top -bn1 | head -20'

# 4. 优化设置 / Optimize settings
# - 减少视频分辨率 / Reduce video resolution
# - 降低控制频率 / Lower control frequency
# - 使用更近的AWS区域 / Use closer AWS region
```

---

## 5. 常用命令速查 / Quick Command Reference

### 启动/停止服务 / Start/Stop Service

```bash
# 启动 / Start
sudo systemctl start teleop-soarm

# 停止 / Stop
sudo systemctl stop teleop-soarm

# 重启 / Restart
sudo systemctl restart teleop-soarm

# 查看状态 / Check status
sudo systemctl status teleop-soarm

# 查看日志 / View logs
sudo journalctl -u teleop-soarm -f
```

### 更新代码 / Update Code

```bash
# SSH到服务器 / SSH to server
ssh -i ~/.ssh/teleop-aws.pem ubuntu@44.254.63.252

# 更新代码 / Update code
cd ~/teleop_platform
git pull  # OR re-upload files

# 重启服务 / Restart service
sudo systemctl restart teleop-soarm
```

### 修改配置 / Change Configuration

```bash
# 修改USB端口 / Change USB port
sudo systemctl edit teleop-soarm --full
# 修改 TELEOP_SOARM_PORT 环境变量 / Edit TELEOP_SOARM_PORT env var

# 重新加载配置 / Reload configuration
sudo systemctl daemon-reload
sudo systemctl restart teleop-soarm
```

---

## 6. 验收清单 / Acceptance Checklist

部署完成后，验证以下项目 / After deployment, verify:

- [ ] USB设备可以被识别 / USB device is recognized
- [ ] 服务可以正常启动 / Service starts successfully
- [ ] 可以从本地连接到服务器 / Can connect from local machine
- [ ] WebSocket连接正常 / WebSocket connection works
- [ ] 可以控制机械臂移动 / Can control robot movement
- [ ] 视频流正常显示 / Video stream displays correctly
- [ ] 延迟低于100ms (本地) / Latency < 100ms (local)
- [ ] 延迟低于150ms (跨国) / Latency < 150ms (cross-border)
- [ ] 服务开机自启动 / Service auto-starts on boot
- [ ] 日志正常记录 / Logs are being recorded
- [ ] 安全门功能正常 / Safety gate functions properly

---

## 7. 下一步 / Next Steps

完成部署后 / After successful deployment:

1. ✅ 记录延迟测试结果 / Record latency test results
2. ✅ 向经理报告部署状态 / Report deployment status to manager
3. ✅ 提供访问凭证和URL / Provide access credentials and URLs
4. ✅ 安排远程演示 / Schedule remote demo
5. ✅ 收集反馈并优化 / Collect feedback and optimize

---

## 联系信息 / Contact Info

如有问题，请联系 / For issues, contact:
- 技术支持 / Tech Support: [Your Email]
- 文档更新 / Doc Updates: [Repository URL]

最后更新 / Last Updated: 2026-02-05
