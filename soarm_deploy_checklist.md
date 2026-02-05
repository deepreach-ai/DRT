# SO-ARM101 部署检查清单 / Deployment Checklist

## 📋 部署前检查 / Pre-Deployment Checklist

### 硬件准备 / Hardware Preparation
- [ ] SO-ARM101机械臂已通电 / SO-ARM101 powered on
- [ ] USB连接稳定 / USB connection stable
- [ ] 电机正常响应 / Motors responding normally
- [ ] 夹爪功能正常 / Gripper functioning
- [ ] (可选) 摄像头已连接 / (Optional) Cameras connected

### 软件准备 / Software Preparation
- [ ] Python 3.8+ 已安装 / Python 3.8+ installed
- [ ] LeRobot已安装 / LeRobot installed
- [ ] 依赖包已安装 / Dependencies installed
  ```bash
  pip install -r server/requirements.txt
  ```
- [ ] USB端口权限已配置 / USB port permissions configured
  ```bash
  sudo usermod -a -G dialout $USER
  ```

### 本地测试 / Local Testing
- [ ] USB设备可识别 / USB device detected
  ```bash
  ls /dev/tty* | grep -E "USB|ACM"
  ```
- [ ] 连接测试通过 / Connection test passed
  ```bash
  python test_soarm_integration.py
  ```
- [ ] 本地服务器启动成功 / Local server starts
  ```bash
  ./start_soarm_local.sh /dev/ttyUSB0
  ```
- [ ] Web界面可访问 / Web UI accessible
  ```
  http://localhost:8000
  ```
- [ ] 机械臂控制正常 / Robot control works
  - [ ] 安全门激活 / Safety gate activation
  - [ ] 位置控制 / Position control
  - [ ] 方向控制 / Orientation control
  - [ ] 夹爪控制 / Gripper control

---

## 🌐 AWS部署检查 / AWS Deployment Checklist

### AWS基础设施 / AWS Infrastructure
- [ ] EC2实例已创建 / EC2 instance created
  - 推荐: t3.medium (2 vCPU, 4GB RAM)
  - OS: Ubuntu 22.04 LTS
- [ ] 安全组配置正确 / Security group configured
  - [ ] SSH (22) - 限制为你的IP / SSH (22) - Your IP only
  - [ ] API (8000) - 开放 / API (8000) - Open
  - [ ] Web (8080) - 开放 (如需要) / Web (8080) - Open (if needed)
- [ ] SSH密钥已下载 / SSH key downloaded
  ```bash
  chmod 400 ~/.ssh/teleop-aws.pem
  ```
- [ ] 可以SSH连接 / Can SSH connect
  ```bash
  ssh -i ~/.ssh/teleop-aws.pem ubuntu@YOUR_AWS_IP
  ```

### 服务器配置 / Server Configuration
- [ ] 代码已上传 / Code uploaded
- [ ] 依赖已安装 / Dependencies installed
- [ ] USB设备已识别 / USB device detected on server
- [ ] USB权限已配置 / USB permissions configured
- [ ] systemd服务已创建 / systemd service created
- [ ] 服务已启动 / Service started
  ```bash
  sudo systemctl status teleop-soarm
  ```
- [ ] 服务开机自启 / Service auto-starts
  ```bash
  sudo systemctl enable teleop-soarm
  ```

### 网络测试 / Network Testing
- [ ] 可以ping通服务器 / Can ping server
  ```bash
  ping YOUR_AWS_IP
  ```
- [ ] API端口可访问 / API port accessible
  ```bash
  curl http://YOUR_AWS_IP:8000/api/v1/status
  ```
- [ ] WebSocket连接正常 / WebSocket works
  ```bash
  wscat -c ws://YOUR_AWS_IP:8000/ws/v1/teleop
  ```

### 功能测试 / Functional Testing
- [ ] Web UI可访问 / Web UI accessible
  ```
  http://YOUR_AWS_IP:8000
  ```
- [ ] 可以连接机械臂 / Can connect to robot
- [ ] 机械臂响应控制命令 / Robot responds to commands
- [ ] 视频流正常 (如有摄像头) / Video stream works (if cameras)
- [ ] 安全门功能正常 / Safety gate functions
- [ ] 延迟测试通过 / Latency test passed
  - 目标: <100ms (本地), <150ms (跨境)
  - Target: <100ms (local), <150ms (cross-border)

---

## 🇺🇸 美国测试检查 / USA Testing Checklist

### 网络连接 / Network Connection
- [ ] 从美国可以ping通服务器 / Can ping from USA
- [ ] 延迟可接受 / Latency acceptable
  ```bash
  ping -c 100 YOUR_AWS_IP
  ```

### 功能测试 / Functional Testing
- [ ] 可以访问Web界面 / Can access web UI
- [ ] 可以连接机械臂 / Can connect to robot
- [ ] 控制延迟可接受 / Control latency acceptable
- [ ] 视频流畅 / Video smooth
- [ ] 无明显卡顿 / No significant lag

### 性能测试 / Performance Testing
- [ ] 运行延迟测试 / Run latency test
  ```bash
  python client/latency_test_client.py \
      --server ws://YOUR_AWS_IP:8000/ws/v1/teleop \
      --samples 200
  ```
- [ ] 记录测试结果 / Record test results
  - 平均延迟 / Average latency: _____ms
  - 最大延迟 / Max latency: _____ms
  - 丢包率 / Packet loss: _____%

---

## 📊 部署后验证 / Post-Deployment Verification

### 系统稳定性 / System Stability
- [ ] 服务持续运行24小时无故障 / Service runs 24h without issues
- [ ] 日志无严重错误 / No critical errors in logs
  ```bash
  sudo journalctl -u teleop-soarm --since "24 hours ago" | grep -i error
  ```
- [ ] 重启后服务自动恢复 / Service recovers after reboot
  ```bash
  sudo reboot
  # 等待重启完成后检查 / After reboot
  sudo systemctl status teleop-soarm
  ```

### 性能监控 / Performance Monitoring
- [ ] CPU使用率正常 (<70%) / CPU usage normal (<70%)
- [ ] 内存使用率正常 (<80%) / Memory usage normal (<80%)
- [ ] 磁盘空间充足 (>10GB可用) / Disk space sufficient (>10GB free)
- [ ] 网络带宽充足 / Network bandwidth sufficient

### 安全检查 / Security Check
- [ ] 只有必要端口开放 / Only necessary ports open
- [ ] SSH密钥认证正常 / SSH key auth works
- [ ] 防火墙规则正确 / Firewall rules correct
- [ ] (可选) HTTPS已配置 / (Optional) HTTPS configured

---

## 🎯 经理验收检查 / Manager Acceptance Checklist

### 基础功能 / Basic Functions
- [ ] 可以从美国访问系统 / Can access from USA
- [ ] 可以看到实时视频 / Can see live video
- [ ] 可以控制机械臂移动 / Can control robot movement
- [ ] 控制响应及时 / Control responds promptly

### 性能指标 / Performance Metrics
- [ ] 延迟 <150ms / Latency <150ms
- [ ] 视频帧率 ≥20fps / Video framerate ≥20fps
- [ ] 控制频率 ≥20Hz / Control frequency ≥20Hz
- [ ] 无明显延迟感 / No noticeable lag

### 用户体验 / User Experience
- [ ] 界面直观易用 / UI intuitive
- [ ] 连接稳定 / Connection stable
- [ ] 操作流畅 / Operation smooth
- [ ] 错误提示清晰 / Error messages clear

### 文档完整性 / Documentation
- [ ] 部署文档完整 / Deployment docs complete
- [ ] 使用说明清晰 / User guide clear
- [ ] 故障排除指南有用 / Troubleshooting guide helpful
- [ ] 代码注释充分 / Code well commented

---

## ✅ 最终签字 / Final Sign-off

### 开发团队 / Development Team
- [ ] 本地测试通过 / Local testing passed
- [ ] AWS部署完成 / AWS deployment complete
- [ ] 文档已更新 / Documentation updated
- [ ] 代码已提交 / Code committed

**签字 / Sign-off:** _______________ **日期 / Date:** _______________

### 经理验收 / Manager Acceptance
- [ ] 功能满足需求 / Functions meet requirements
- [ ] 性能达到标准 / Performance meets standards
- [ ] 可以投入使用 / Ready for production

**签字 / Sign-off:** _______________ **日期 / Date:** _______________

---

## 📝 备注 / Notes

记录任何问题、建议或需要跟进的事项 / Record any issues, suggestions, or follow-ups:

```
[日期 / Date] [问题 / Issue] [状态 / Status] [备注 / Notes]
_________________________________________________________________
_________________________________________________________________
_________________________________________________________________
```

---

## 🔗 快速链接 / Quick Links

- **服务器地址 / Server URL:** `http://YOUR_AWS_IP:8000`
- **API文档 / API Docs:** `http://YOUR_AWS_IP:8000/docs`
- **WebSocket:** `ws://YOUR_AWS_IP:8000/ws/v1/teleop`
- **SSH命令 / SSH Command:**
  ```bash
  ssh -i ~/.ssh/teleop-aws.pem ubuntu@YOUR_AWS_IP
  ```
- **日志查看 / View Logs:**
  ```bash
  sudo journalctl -u teleop-soarm -f
  ```
- **重启服务 / Restart Service:**
  ```bash
  sudo systemctl restart teleop-soarm
  ```

---

**最后更新 / Last Updated:** 2026-02-05
