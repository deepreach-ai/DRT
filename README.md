# DeepReach Teleoperation Platform

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

DeepReach Teleoperation Platform is a universal, cloud-native teleoperation system designed to bridge the gap between simulation and real-world robotic manipulation. It supports heterogeneous embodiments (6-DoF/7-DoF), multiple input methods (VR, Keyboard, Joystick), and seamless switching between local and cloud environments.

![Interface Demo](docs/demo.gif)

## 🌟 Key Features

### 🎮 Multi-Modal Inputs
*   **VR Control:** Full 6-DoF control with stereoscopic vision (Quest 3/3S via WebXR).
*   **Keyboard & Mouse:** Accessible browser-based control for quick testing.
*   **Joystick:** Xbox/Gamepad support for intuitive operation.

### 🤖 Universal Embodiment Support
*   **Heterogeneous Robots:** Supports 6-DoF (e.g., SO-ARM101) and 7-DoF (e.g., Realman RM65) arms.
*   **Unified Interface:** Abstracted backend allows controlling different robots with the same client.
*   **Supported Hardware:**
    *   **SO-ARM101** (6-DoF)
    *   **Realman RM65/RM75** (7-DoF)
    *   **Realman RM75B** (7-DOF, VR-Optimized)
    *   **Lingyu** (URDF support)

### 🌍 Simulation & Real World
*   **MuJoCo (Step 1):** Fast, local physics simulation for rapid development and testing.
*   **Isaac Sim (Step 2):** High-fidelity, photorealistic simulation (Local or Cloud via Omniverse Streaming).
*   **Sim-to-Real:** Identical control stack for simulation and physical hardware.

### 📱 Cross-Platform
*   **Server:** Linux (Ubuntu), macOS.
*   **Client:** Any web browser (Desktop, Android, VR headsets).
*   **Android Support:** Control via mobile browser or dedicated app (planned).

## 🚀 Scenarios
*   **Research:** Universal platform for teleoperation data collection.
*   **Logistics:** Box sorting and handling (e.g., Amazon return processing).
*   **Remote Operation:** Low-latency control over public internet.

## ⚡ Quick Start

For detailed installation and usage instructions, see the [Quick Start Guide](QUICKSTART.md).

### 1. Install
```bash
git clone https://github.com/deepreach-ai/DRT.git
cd DRT
pip install -r requirements.txt
```

### 2. Start the Unified FastAPI Server
Runs Web UI and API on a single port.
```bash
# Mock backend (no hardware)
python run_server.py --backend mock

# MuJoCo simulation (SO-101)
python run_server.py --backend mujoco \
  --mujoco-xml robots/so101.xml \
  --mujoco-ee gripperframe

# MuJoCo simulation (RM75B VR-optimized)
python run_server.py --backend mujoco \
  --mujoco-xml robots/rm75b_vr_v2.xml \
  --mujoco-ee ee_site

# MuJoCo simulation (SO-ARM101 VR-optimized)
python run_server.py --backend mujoco \
  --mujoco-xml robots/so101.xml \
  --mujoco-ee gripperframe

# Isaac Sim backend
python run_server.py --backend isaac
```

### 3. Open the Web Client
Visit:
```
http://localhost:8000/web/
```
Login (default):
- Username: operator
- Password: operator

### 4. Server Health Check
```bash
curl http://localhost:8000/api/v1/statistics | python -m json.tool
```
Expected fields include:
- backend: mujoco/mock/isaac/soarm/so101_dual
- status: connected
- current_position / orientation statistics

### 5. Keyboard Control
- Web UI supports XYZ translation + yaw rotation
- Python client supports full 6-DoF

References:
- Docs: [Keyboard Controls](docs/KEYBOARD_CONTROLS.md)
- Python client: `python client/keyboard_client.py`

## 🥽 VR Teleoperation (Meta Quest 3)

The VR client uses WebXR and sends 50 Hz control deltas over WebSocket.

### Modes
- Single Arm (SO-ARM101): `?urdf=so101`
- Dual Arm: `?urdf=so101_dual`
- RM75B: `?urdf=RM75-B`

### Connect Over Wi‑Fi
1. Put the Quest 3 and your computer on the same network.
2. Open the Quest Browser.
3. Navigate to:
   - `http://<host-ip>:8000/web/vr.html?urdf=so101`
4. Login (operator/operator), click “Enter VR Mode”.

### Wired USB (ADB Reverse)
Lowest latency and easy local access:
```bash
adb devices                # verify headset connection
adb reverse tcp:8000 tcp:8000
```
Open on the headset:
```
http://localhost:8000/web/vr.html?urdf=so101
```

### HTTPS/WSS (Optional Secure Context)
Some headset/browser setups prefer HTTPS for WebXR:
```bash
./generate_cert.sh
python run_server.py --backend soarm --soarm-port /dev/tty.usbmodemXXXX \
  --port 8443 --ssl-key key.pem --ssl-cert cert.pem
adb reverse tcp:8443 tcp:8443
```
Open:
```
https://localhost:8443/web/vr.html?urdf=so101
```
Accept the self-signed certificate warning.

### Dual-Arm Launch
```bash
export TELEOP_BACKEND=so101_dual
export TELEOP_LEFT_PORT=/dev/tty.usbmodemLEFT
export TELEOP_RIGHT_PORT=/dev/tty.usbmodemRIGHT
python server/teleop_server.py --port 8000
```
Open:
```
http://<host-ip>:8000/web/vr.html?urdf=so101_dual
```

### Controls
- Grip (hold): clutch for direct 6‑DoF hand deltas
- Right joystick: rate-based movement when clutch not held
- Right trigger: gripper open/close
- B button: exit VR

### Video Streams
- Generic MJPEG:
  - `/api/v1/video/mjpeg?token=...`
- Per camera MJPEG:
  - `/api/v1/video/{camera_name}/mjpeg?token=...` where camera_name is `left`, `right`, `depth` if supported.
Note: login first to receive an auth token used by the video endpoints.

### Startup Scripts
- RM75B: `./start_rm75b_vr.sh`
- SO-ARM101 (sim): `./start_so101_vr_sim.sh`
- SO-ARM101 (real): `./start_so101_vr_real.sh`
- Dual SO-ARM101: `./start_dual_so101_vr.sh`

## 📚 Documentation

Detailed guides can be found in the `docs/` directory:

*   **Setup:** [VR Setup](docs/VR_SETUP.md), [SO-ARM Setup](docs/SOARM_SETUP.md), [Ngrok (Remote Access)](docs/NGROK_SETUP.md)
*   **Operation:** [Keyboard Controls](docs/KEYBOARD_CONTROLS.md), [Quest 3 Sync Guide](docs/QUEST3_SYNC_GUIDE.md)
*   **Deployment:** [AWS Deployment](docs/AWS_DEPLOYMENT_GUIDE.md), [Isaac Sim Workflow](docs/ISAAC_SIM_WORKFLOW.md)
*   **Validation:** [Latency Testing](docs/LATENCY_TEST_GUIDE.md), [Local Validation](docs/LOCAL_VALIDATION_GUIDE.md)

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgements

*   **NVIDIA:** For Isaac Sim and investment support.
*   **OpenTelevision:** For inspiration on teleoperation frameworks.
*   **Realman & Lingyu:** For hardware support.
