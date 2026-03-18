# Quick Start Guide

Welcome to DRT (DeepReach Teleoperation). This guide helps you quickly run simulation (MuJoCo), real robot control, and Web/VR clients with the SO-ARM101.

---

## 1. Installation

### Requirements
- Python 3.10+
- Optional hardware: SO-ARM101 arm, Meta Quest 3/3S, Webcam/RealSense

### Steps
```bash
git clone https://github.com/deepreach-ai/DRT.git
cd DRT
pip install -r server/requirements.txt
```

---

## 2. Start the Server (single server, port 8000)
FastAPI server provides both Web UI and REST APIs.

### Mock backend (no hardware/sim required)
```bash
python3 run_server.py --backend mock
```

### MuJoCo simulation (SO-ARM101)
```bash
python3 run_server.py --backend mujoco \
  --mujoco-xml robots/so101/so101.xml \
  --mujoco-ee gripperframe
```

### Isaac Sim backend
```bash
python3 run_server.py --backend isaac
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
curl http://localhost:8000/api/v1/statistics | python3 -m json.tool
```
Output should include:
- backend: mujoco/mock/isaac/soarm
- status: connected
- pose/position statistics

---

## 5. Keyboard Control
- Web UI: supports XYZ translation + Yaw rotation
- Python client: full 6-DoF (Pitch/Roll/Yaw + XYZ)

References:
- Docs: [Keyboard Controls](docs/KEYBOARD_CONTROLS.md)
- Python client:
```bash
python3 client/keyboard_client.py
```

---

## 6. VR (Quest 3/3S)

### Single Arm
```
http://<host-ip>:8000/web/vr.html?urdf=so101
```

### Dual Arm
```
http://<host-ip>:8000/web/vr.html?urdf=so101_dual
```

In Web UI, click "Enter VR Mode". For WebXR HTTPS setup, see [VR Setup](docs/VR_SETUP.md).

### Wired (lowest latency)
```bash
adb reverse tcp:8000 tcp:8000
# then open http://localhost:8000/web/vr.html?urdf=so101 on headset
```

---

## 7. Real Robot (SO-ARM101)
```bash
# Single arm
python3 run_server.py --backend soarm --soarm-port /dev/ttyUSB0

# Dual arm
export TELEOP_BACKEND=so101_dual
export TELEOP_LEFT_PORT=/dev/tty.usbmodemLEFT
export TELEOP_RIGHT_PORT=/dev/tty.usbmodemRIGHT
python3 server/teleop_server.py --port 8000
```

More setup: [SOARM_SETUP](docs/SOARM_SETUP.md)

---

## 8. Video Streams
Supports Webcam, RealSense, and Isaac Sim. Web UI shows live MJPEG streams.
- Generic: `/api/v1/video/mjpeg?token=...`
- Per camera: `/api/v1/video/{left|right|depth}/mjpeg?token=...`

Note: login first to receive an auth token.

---

## 9. Troubleshooting
- **404 page:** visit `http://localhost:8000/web/`
- **Login failed:** use `operator/operator`; Base URL empty or `http://localhost:8000`
- **WebSocket failed:** ensure server is running, Base URL is correct
- **Remote access:** verify port 8000 is open, see [AWS Deployment](docs/AWS_DEPLOYMENT_GUIDE.md)

More guides in [docs/](docs/).
