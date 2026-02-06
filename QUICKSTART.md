# 🤖 Teleop System Quick Start Guide

Welcome to the Teleoperation System! This guide will help you get started with Simulation (MuJoCo), Real Robot Control (SO-ARM101), and Video Streaming.

---

## 🚀 1. Installation

### Prerequisites
*   Python 3.10+
*   Node.js & npm (for Web UI)
*   **Hardware (Optional):** SO-ARM101 Robot Arm, Webcam/RealSense

### Setup Environment
```bash
# 1. Clone the repository
git clone <repo_url>
cd teleop_system

# 2. Install Python dependencies
pip install -r requirements.txt

# 3. Install LeRobot (Required for Real Robot)
# Follow instructions at: https://github.com/huggingface/lerobot
# Or typically: pip install lerobot
```

---

## 🎮 2. Running in Simulation (Mock/MuJoCo)

Best for testing the UI and network without hardware.

### Start the Server (Mock Mode)
```bash
# Runs a mock backend that simulates a robot responding to commands
python server/teleop_server.py --backend mock
```

### Start the Server (MuJoCo Physics)
```bash
# Requires MuJoCo installed
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
    *   `Space`: Toggle Gripper

---

## 🦾 3. Running Real Robot (SO-ARM101)

Control the physical SO-ARM101 robot arm.

### Hardware Setup
1.  Connect SO-ARM101 via USB.
2.  Find the port (e.g., `/dev/tty.usbmodem...` on Mac or `/dev/ttyUSB0` on Linux).
3.  Ensure 12V power is connected.

### Start Server with Robot
```bash
# Replace with your actual port
python server/teleop_server.py --backend soarm --robot-port /dev/tty.usbmodem5B3E1187881
```

*   **Safety Note:** The robot will sync to its current position on startup.
*   **Emergency Stop:** Press `Ctrl+C` in the terminal or close the browser tab to stop.

---

## 📹 4. Video Streaming

The system supports multiple video sources.

### Local Webcam (USB)
The `soarm` backend automatically detects USB webcams (ID 0-9).
*   Just start the server with `--backend soarm`.
*   The video will appear in the Web UI automatically.

### External Source (e.g., Isaac Sim / OBS)
You can push MJPEG frames to the server via HTTP:
```bash
# Example: Send a test image
curl -X POST -H "Content-Type: image/jpeg" --data-binary @test.jpg http://localhost:8000/api/v1/video/ingest
```

---

## 🛠️ Troubleshooting

| Issue | Solution |
| :--- | :--- |
| **"Missing motors" error** | The system now auto-adapts. Check USB connection and power. |
| **Robot moves sluggishly** | We increased speed limits. Check if `max_velocity` in client is set too low. |
| **Wrist spins 180°** | Anti-flip guard is active. Try moving to a less extreme position. |
| **Port access denied** | Run with `sudo` or check user permissions (`dialout` group). |

---

## 📂 Project Structure
*   `server/`: Python FastAPI backend & Robot Logic
    *   `teleop_server.py`: Main entry point
    *   `backends/soarm_backend.py`: Real robot driver
*   `client/web/`: HTML/JS Frontend
*   `configs/`: Robot URDFs and settings

Happy Teleoperating! 🕹️
