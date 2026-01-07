# 🚀 DRT 快速上手指南（QUICK START）

欢迎使用 DRT（分布式机器人远程操控）系统！本指南帮助你以最简单的方式启动仿真（MuJoCo）、真实机械臂与 Web/VR 控制。

---

## 1. 安装

### 环境要求
- Python 3.10+
- 可选硬件：SO-ARM101、Realman、Quest 3/3S、Webcam/RealSense

### 安装步骤
```bash
git clone https://github.com/deepreach-ai/DRT.git
cd DRT
pip install -r requirements.txt
```

---

## 2. 启动服务（单一服务器，端口 8000）
使用集成的 FastAPI 服务器，既提供 Web UI，又提供 API。

### 启动 Mock 后端（无需硬件/仿真）
```bash
python run_server.py --backend mock
```

### 启动 MuJoCo 仿真（SO-101 示例）
```bash
python run_server.py --backend mujoco \
  --mujoco-xml robots/so101.xml \
  --mujoco-ee gripperframe
```

### 启动 Isaac 后端
```bash
python run_server.py --backend isaac
```

---

## 3. 打开 Web 客户端并登录
在浏览器访问：
```
http://localhost:8000/web/
```
登录（默认）：
- 用户名：operator
- 密码：operator
提示：无需填写 Base URL，或设置为 `http://localhost:8000`。

---

## 4. 快速验证 API
```bash
curl http://localhost:8000/api/v1/statistics | python -m json.tool
```
输出应包含：
- backend: mujoco/mock/isaac
- status: connected
- 位置/姿态统计信息等

---

## 5. 键盘控制
- Web UI：支持 XYZ 平移 + Yaw 旋转
- Python 客户端：支持完整 6-DoF（Pitch/Roll/Yaw + XYZ）

参考文档与命令：
- 文档：[Keyboard Controls](docs/KEYBOARD_CONTROLS.md)
- Python 客户端：
```bash
python client/keyboard_client.py
```

---

## 6. VR（Quest 3/3S）
- 打开 Web UI 页面后点击 “Enter VR Mode”
- 如需 HTTPS/SSL 以支持 WebXR，参考 [VR 设置](docs/VR_SETUP.md)
- 远程访问可配合 ngrok/反向代理进行 HTTPS 暴露，详见 [LOCAL_VALIDATION_GUIDE](docs/LOCAL_VALIDATION_GUIDE.md)

---

## 7. 真实机械臂（SO-ARM101 示例）
```bash
python run_server.py --backend soarm --soarm-port /dev/ttyUSB0
```
更多硬件配置与部署参考：[SOARM_SETUP](docs/SOARM_SETUP.md)、[SOARM_DEPLOYMENT_GUIDE](docs/SOARM_DEPLOYMENT_GUIDE.md)

---

## 8. 视频流
支持多源视频（Webcam、RealSense、Isaac Sim）。Web UI 会显示占位或 MJPEG/RTC 视频。高级配置参考相关文档。

---

## 9. 常见问题（Troubleshooting）
- 页面 404：访问 `http://localhost:8000/web/`
- 登录失败：使用 `operator/operator`；Base URL 留空或设为 `http://localhost:8000`
- WebSocket 失败：确保服务已启动，且 Base URL 正确
- 远程访问：检查云端安全组端口（默认 8000），参考部署文档

更多问题与进阶指南请见 [docs/](docs/)。
