# SO-ARM101 部署检查清单 / Deployment Checklist

## 📋 部署前检查 / Pre-Deployment Checklist

### 硬件准备 / Hardware Preparation
- [ ] SO-ARM101机械臂已通电 / SO-ARM101 powered on
- [ ] USB连接稳定 / USB connection stable
- [ ] 电机正常响应 / Motors responding normally
- [ ] 夹爪功能正常 / Gripper functioning
- [ ] (可选) 摄像头已连接 / (Optional) Cameras connected

### 软件准备 / Software Preparation
- [ ] Python 3.10+ 已安装 / Python 3.10+ installed
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
- [ ] 本地服务器启动成功 / Local server starts
  ```bash
  python3 run_server.py --backend soarm --soarm-port /dev/ttyUSB0
  ```
- [ ] Web界面可访问 / Web UI accessible
  ```
  http://localhost:8000/web/
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
- [ ] SSH密钥已下载 / SSH key downloaded
  ```bash
  chmod 400 ~/.ssh/teleop-aws.pem
  ```
- [ ] 可以SSH连接 / Can SSH connect
  ```bash
  ssh -i ~/.ssh/teleop-aws.pem ubuntu@<YOUR_SERVER_IP>
  ```

### 服务器配置 / Server Configuration
- [ ] 代码已上传 / Code uploaded
- [ ] 依赖已安装 / Dependencies installed
- [ ] systemd服务已创建并启动 / systemd service created and started
  ```bash
  sudo systemctl status teleop-soarm
  sudo systemctl enable teleop-soarm
  ```

### 网络测试 / Network Testing
- [ ] API端口可访问 / API port accessible
  ```bash
  curl http://<YOUR_SERVER_IP>:8000/api/v1/statistics
  ```
- [ ] WebSocket连接正常 / WebSocket works
  ```bash
  wscat -c ws://<YOUR_SERVER_IP>:8000/ws/v1/teleop
  ```

### 功能测试 / Functional Testing
- [ ] Web UI可访问 / Web UI accessible
  ```
  http://<YOUR_SERVER_IP>:8000/web/
  ```
- [ ] 机械臂响应控制命令 / Robot responds to commands
- [ ] 安全门功能正常 / Safety gate functions
- [ ] 延迟测试通过 / Latency test passed
  - 目标: <100ms (本地), <150ms (跨境)
  - Target: <100ms (local), <150ms (cross-border)

---

## 📊 部署后验证 / Post-Deployment Verification

### 系统稳定性 / System Stability
- [ ] 服务持续运行无故障 / Service runs without issues
- [ ] 日志无严重错误 / No critical errors in logs
  ```bash
  sudo journalctl -u teleop-soarm --since "24 hours ago" | grep -i error
  ```
- [ ] 重启后服务自动恢复 / Service recovers after reboot

### 性能监控 / Performance Monitoring
- [ ] CPU使用率正常 (<70%) / CPU usage normal (<70%)
- [ ] 内存使用率正常 (<80%) / Memory usage normal (<80%)
- [ ] 磁盘空间充足 (>10GB可用) / Disk space sufficient (>10GB free)

### 安全检查 / Security Check
- [ ] 只有必要端口开放 / Only necessary ports open
- [ ] SSH密钥认证正常 / SSH key auth works
- [ ] (可选) HTTPS已配置 / (Optional) HTTPS configured

---

## 🎯 验收检查 / Acceptance Checklist

### 性能指标 / Performance Metrics
- [ ] 延迟 <150ms / Latency <150ms
- [ ] 视频帧率 ≥20fps / Video framerate ≥20fps
- [ ] 控制频率 ≥20Hz / Control frequency ≥20Hz

### 延迟测试 / Latency Test
```bash
python3 scripts/utils/measure_latency.py --host <YOUR_SERVER_IP> --port 8000
```
记录结果 / Record results:
- 平均延迟 / Average latency: _____ms
- 最大延迟 / Max latency: _____ms

---

## 🔗 快速命令参考 / Quick Command Reference

```bash
# SSH 连接 / SSH connect
ssh -i ~/.ssh/teleop-aws.pem ubuntu@<YOUR_SERVER_IP>

# 查看日志 / View logs
sudo journalctl -u teleop-soarm -f

# 重启服务 / Restart service
sudo systemctl restart teleop-soarm

# 健康检查 / Health check
curl http://<YOUR_SERVER_IP>:8000/api/v1/statistics | python3 -m json.tool
```

---

**最后更新 / Last Updated:** 2026-03-18
