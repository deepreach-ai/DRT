# SO-ARM101 快速开始指南 / Quick Start Guide

## 🚀 5分钟快速启动 / 5-Minute Quick Start

### 前提条件 / Prerequisites

1. SO-ARM101机械臂已连接USB / SO-ARM101 connected via USB
2. Python 3.8+ 已安装 / Python 3.8+ installed
3. LeRobot已安装 / LeRobot installed

### 快速测试 / Quick Test

```bash
cd ~/Teleop_platform

# 1. 查找USB端口 / Find USB port
ls /dev/tty* | grep -E "USB|ACM"
# 输出示例 / Example output: /dev/ttyUSB0

# 2. 测试连接 / Test connection (可选 / Optional)
python test_soarm_integration.py

# 3. 启动服务器 / Start server
./start_soarm_local.sh /dev/ttyUSB0
# 或 / OR
python run_server.py --backend soarm --soarm-port /dev/ttyUSB0

# 4. 打开浏览器测试 / Open browser to test
# http://localhost:8000
```

---

## 📋 详细步骤 / Detailed Steps

### 步骤1：检查硬件连接 / Step 1: Check Hardware

```bash
# 检查USB设备 / Check USB devices
lsusb

# 查看串口设备 / List serial devices
ls -l /dev/tty* | grep -E "USB|ACM"

# 检查权限 / Check permissions
ls -l /dev/ttyUSB0  # 替换为你的端口 / Replace with your port
```

**如果权限不足 / If permission denied:**
```bash
# 方法1：添加到dialout组 (推荐) / Method 1: Add to dialout group (recommended)
sudo usermod -a -G dialout $USER
# 然后重新登录 / Then re-login

# 方法2：临时赋权 / Method 2: Temporary fix
sudo chmod 666 /dev/ttyUSB0
```

### 步骤2：测试机械臂通信 / Step 2: Test Robot Communication

```bash
cd ~/Teleop_platform

# 如果test_soarm_integration.py中的端口不对，先修改它 / Fix port if needed
# 编辑文件第24行 / Edit line 24:
# port='/dev/ttyUSB0',  # 改为你的端口 / Change to your port

python test_soarm_integration.py
```

**预期输出 / Expected output:**
```
🤖 SO-ARM Backend Integration Test

============================================================
Testing SO-ARM Backend Connection
============================================================

1. Creating SO-ARM backend...
✓ Backend created: so101_test

2. Connecting to robot...
[FlexibleSO101Follower] Scanning for motors on /dev/ttyUSB0...
  - Found shoulder_pan (ID 1)
  - Found shoulder_lift (ID 2)
  ...
✓ Connection successful!

3. Getting robot status...
Status: {...}

✅ ALL TESTS PASSED!
```

### 步骤3：启动本地服务器 / Step 3: Start Local Server

**方法A：使用快速启动脚本 (推荐) / Method A: Quick start script (recommended)**

```bash
./start_soarm_local.sh /dev/ttyUSB0
```

**方法B：直接运行 / Method B: Direct run**

```bash
python run_server.py \
    --backend soarm \
    --soarm-port /dev/ttyUSB0 \
    --host 0.0.0.0 \
    --port 8000
```

**方法C：使用环境变量 / Method C: Using environment variables**

```bash
export TELEOP_BACKEND=soarm
export TELEOP_SOARM_PORT=/dev/ttyUSB0
export TELEOP_PORT=8000
python run_server.py
```

### 步骤4：测试Web控制 / Step 4: Test Web Control

1. **打开浏览器 / Open browser:**
   ```
   http://localhost:8000
   ```

2. **登录 (如果启用了认证) / Login (if auth enabled):**
   - Username: `admin`
   - Password: `admin`

3. **连接机械臂 / Connect to robot:**
   - 点击 "Connect" 按钮 / Click "Connect" button
   - 等待连接成功 / Wait for connection success

4. **激活安全门 / Activate safety gate:**
   - 按键盘 `1` 键 / Press keyboard key `1`
   - 或点击界面上的"Activate Safety" / Or click "Activate Safety" in UI

5. **控制机械臂 / Control robot:**
   - 使用键盘控制 / Use keyboard:
     - `W/S`: 前进/后退 / Forward/Backward
     - `A/D`: 左右 / Left/Right
     - `Q/E`: 上下 / Up/Down
     - `↑/↓/←/→`: 旋转 / Rotate
     - `G/H`: 夹爪开合 / Gripper open/close
   - 或使用虚拟摇杆 / Or use virtual joystick

### 步骤5：监控状态 / Step 5: Monitor Status

**在另一个终端 / In another terminal:**

```bash
# 实时查看日志 / View logs in real-time
tail -f server.log

# 或使用curl查看状态 / Or check status with curl
curl http://localhost:8000/api/v1/status | python -m json.tool
```

---

## 🌐 AWS部署 / AWS Deployment

### 快速部署到AWS / Quick Deploy to AWS

```bash
# 1. 确保本地测试成功 / Ensure local testing works
./start_soarm_local.sh /dev/ttyUSB0

# 2. 停止本地服务器 / Stop local server (Ctrl+C)

# 3. 运行AWS部署脚本 / Run AWS deployment script
# 参考 SOARM_DEPLOYMENT_GUIDE.md 中的脚本 / See script in SOARM_DEPLOYMENT_GUIDE.md

# 4. 或使用手动部署 / Or manual deployment
# 详见 SOARM_DEPLOYMENT_GUIDE.md
```

---

## ⚡ 常见问题 / Common Issues

### 问题1：找不到USB设备 / Issue 1: USB device not found

```bash
# 检查设备是否连接 / Check if device connected
lsusb

# 查看系统日志 / Check system log
dmesg | tail -20

# 可能的端口名称 / Possible port names:
# Linux: /dev/ttyUSB0, /dev/ttyACM0, /dev/ttyUSB1
# macOS: /dev/tty.usbmodem*, /dev/cu.usbmodem*
```

### 问题2：权限被拒绝 / Issue 2: Permission denied

```bash
# 永久解决方案 / Permanent solution:
sudo usermod -a -G dialout $USER
# 重新登录 / Re-login

# 临时解决方案 / Temporary solution:
sudo chmod 666 /dev/ttyUSB0
```

### 问题3：电机连接失败 / Issue 3: Motor connection failed

```bash
# 1. 检查电源 / Check power
# 确保机械臂已上电 / Ensure robot is powered on

# 2. 检查波特率 / Check baud rate
# LeRobot默认使用1000000 / LeRobot uses 1000000 by default

# 3. 重新插拔USB / Reconnect USB
# 拔出并重新插入USB线 / Unplug and replug USB cable

# 4. 重启服务 / Restart service
# Ctrl+C 停止服务器 / Stop server with Ctrl+C
# 重新运行启动脚本 / Re-run start script
```

### 问题4：端口已被占用 / Issue 4: Port already in use

```bash
# 查找占用端口的进程 / Find process using port
lsof -i :8000

# 停止进程 / Kill process
kill <PID>

# 或使用其他端口 / Or use different port
python run_server.py --backend soarm --soarm-port /dev/ttyUSB0 --port 8001
```

### 问题5：机械臂不响应 / Issue 5: Robot not responding

```bash
# 1. 检查安全门是否激活 / Check if safety gate is active
# 按键盘'1'激活 / Press '1' to activate

# 2. 检查连接状态 / Check connection status
curl http://localhost:8000/api/v1/status

# 3. 查看日志错误 / Check logs for errors
tail -50 server.log

# 4. 重启服务 / Restart service
# Ctrl+C 然后重新启动 / Ctrl+C then restart
```

---

## 📊 控制键位图 / Control Layout

### 键盘控制 / Keyboard Control

```
位置控制 / Position Control:
┌─────┬─────┬─────┐
│  Q  │  W  │  E  │  Q: 上升 / Up
│ Up  │ Fwd │ Up  │  W: 前进 / Forward
├─────┼─────┼─────┤  E: 下降 / Down
│  A  │  S  │  D  │  A: 左移 / Left
│Left │ Back│Right│  S: 后退 / Backward
└─────┴─────┴─────┘  D: 右移 / Right

方向控制 / Orientation Control:
┌─────┬─────┬─────┐
│     │  ↑  │     │  ↑: Pitch Up
│     │Roll+│     │  ↓: Pitch Down
├─────┼─────┼─────┤  ←: Yaw Left
│  ←  │  ↓  │  →  │  →: Yaw Right
│Yaw-L│Roll-│Yaw-R│
└─────┴─────┴─────┘

夹爪控制 / Gripper Control:
  G: 打开 / Open
  H: 关闭 / Close

安全控制 / Safety Control:
  1: 激活安全门 / Activate Safety Gate
  0: 重置 / Reset
```

---

## 📈 下一步 / Next Steps

1. ✅ 完成本地测试 / Complete local testing
2. ✅ 部署到AWS / Deploy to AWS
3. ✅ 测试跨境延迟 / Test cross-border latency
4. ✅ 向经理展示 / Demo to manager
5. ✅ 收集反馈并优化 / Collect feedback and optimize

---

## 📚 相关文档 / Related Documentation

- **详细部署指南 / Detailed Deployment:** `SOARM_DEPLOYMENT_GUIDE.md`
- **完整项目文档 / Full Documentation:** `README.md`
- **键盘控制指南 / Keyboard Controls:** `KEYBOARD_CONTROLS_GUIDE.md`
- **AWS部署指南 / AWS Deployment:** `AWS_DEPLOYMENT_GUIDE.md`

---

## 🔧 技术支持 / Technical Support

遇到问题? / Having issues?

1. 查看故障排除部分 / Check troubleshooting section above
2. 查看详细日志 / Check detailed logs: `tail -f server.log`
3. 参考完整部署指南 / Refer to full deployment guide
4. 联系技术支持 / Contact technical support

---

**最后更新 / Last Updated:** 2026-02-05
**版本 / Version:** 1.0.0
