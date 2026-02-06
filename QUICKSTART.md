# 🤖 DRT Quick Start Guide

Welcome to the DRT (Distributed Robot Teleoperation) System! This guide will help you get started with Simulation (MuJoCo), Real Robot Control, and VR.

---

## 🚀 1. Installation

### Prerequisites
*   Python 3.10+
*   Node.js & npm (for Web UI)
*   **Hardware (Optional):** SO-ARM101, Realman Robot, Quest 3, Webcam/RealSense

### Setup Environment
```bash
# 1. Clone the repository
git clone <repo_url>
cd drt

# 2. Install Python dependencies
pip install -r requirements.txt
```

---

## 🎮 2. Running in Simulation (MuJoCo)

Best for testing the UI and network without hardware.

### Start the Server
```bash
# Requires MuJoCo installed (pip install mujoco)
python server/teleop_server.py --backend mujoco
```

### Start the Web UI
Open a new terminal:
```bash
cd client/web
# Open index.html in your browser directly, OR serve it:
python -m http.server 3000
# Visit: http://localhost:3000
```
*   **Controls:**
    *   `W/S`: Forward/Backward (X)
    *   `A/D`: Left/Right (Y)
    *   `Q/E`: Up/Down (Z)
    *   `J/L`: Yaw Rotation
    *   See [Keyboard Controls](docs/KEYBOARD_CONTROLS.md) for full details.

---

## 🥽 3. Running in VR (Quest 3)

Control the robot using VR controllers with stereoscopic vision.

1.  **Start Server:** `python server/teleop_server.py --backend mujoco` (or real robot)
2.  **Start Web Server:** `python -m http.server 3000` (in `client/web`)
3.  **Connect Quest:** Open Browser in Quest and navigate to `http://YOUR_COMPUTER_IP:3000/vr.html`
4.  **Enter VR:** Click "Enter VR" button.

For detailed VR setup (including USB tethering for low latency), see [VR Setup Guide](docs/VR_SETUP.md).

---

## 🦾 4. Running Real Robot

### SO-ARM101
```bash
python server/teleop_server.py --backend soarm --robot-port /dev/tty.usbmodem...
```

### Realman (Coming Soon)
Support for Realman RM65/75 is integrated.
```bash
python server/teleop_server.py --backend realman --robot-ip 192.168.1.10
```

For detailed hardware setup, see [Real Robot Setup](docs/SOARM_SETUP.md).

---

## 📹 5. Video Streaming

The system supports multiple video sources (Webcam, RealSense, Isaac Sim).
*   **Local:** Webcams are auto-detected.
*   **Remote:** MJPEG streams supported.

---

## 🛠️ Troubleshooting

| Issue | Solution |
| :--- | :--- |
| **"Missing motors" error** | Check USB connection and power. |
| **Robot moves sluggishly** | Check `max_velocity` limits. |
| **VR Not Connecting** | Ensure Quest is on same WiFi or use USB tethering. |

See [docs/](docs/) for more troubleshooting guides.
