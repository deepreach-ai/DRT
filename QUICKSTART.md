# 🚀 DRT Quick Start Guide

Welcome to DRT (Distributed Robot Teleoperation). This guide helps you quickly run simulation (MuJoCo), real robot control, and Web/VR clients.

---

## 1. Installation

### Requirements
- Python 3.10+
- Optional hardware: SO-ARM101, Realman, Quest 3/3S, Webcam/RealSense

### Steps
```bash
git clone https://github.com/deepreach-ai/DRT.git
cd DRT
pip install -r requirements.txt
```

---

## 2. Start the Server (single server, port 8000)
FastAPI server provides both Web UI and REST APIs.

### Mock backend (no hardware/sim required)
```bash
python run_server.py --backend mock
```

### MuJoCo simulation (SO-101 example)
```bash
python run_server.py --backend mujoco \
  --mujoco-xml robots/so101/so101.xml \
  --mujoco-ee gripperframe
```

### Isaac backend
```bash
python run_server.py --backend isaac
```

---

## 3. Open Web Client and Login
Visit:
```
http://localhost:8000/web/
```
Default login:
- Username: operator
- Password: operator
Tip: Leave Base URL empty, or set to `http://localhost:8000`.

---

## 4. Quick API Verification
```bash
curl http://localhost:8000/api/v1/statistics | python -m json.tool
```
Output should include:
- backend: mujoco/mock/isaac
- status: connected
- pose/position statistics, etc.

---

## 5. Keyboard Control
- Web UI: supports XYZ translation + Yaw rotation
- Python client: full 6-DoF (Pitch/Roll/Yaw + XYZ)

References:
- Docs: [Keyboard Controls](docs/KEYBOARD_CONTROLS.md)
- Python client:
```bash
python client/keyboard_client.py
```

---

## 6. VR (Quest 3/3S)
- In Web UI, click “Enter VR Mode”
- For WebXR (HTTPS/SSL), see [VR Setup](docs/VR_SETUP.md)
- For remote HTTPS exposure via ngrok/reverse proxy, see [LOCAL_VALIDATION_GUIDE](docs/LOCAL_VALIDATION_GUIDE.md)

---

## 7. Real Robot (SO-ARM101 example)
```bash
python run_server.py --backend soarm --soarm-port /dev/ttyUSB0
```
More setup and deployment: [SOARM_SETUP](docs/SOARM_SETUP.md), [SOARM_DEPLOYMENT_GUIDE](docs/SOARM_DEPLOYMENT_GUIDE.md)

---

## 8. Video Streams
Supports multiple sources (Webcam, RealSense, Isaac Sim). Web UI shows placeholders or MJPEG/RTC video. See docs for advanced configs.

---

## 9. Troubleshooting
- 404 page: visit `http://localhost:8000/web/`
- Login failed: use `operator/operator`; Base URL empty or `http://localhost:8000`
- WebSocket failed: ensure server is running, Base URL is correct
- Remote access: verify security group port (default 8000), see deployment docs

More guides in [docs/](docs/).
