# 🤖 Isaac Sim Remote Workflow Guide

This guide explains how to set up the **Open-TeleVision style workflow** on your remote PC with Isaac Sim.

## 📋 Prerequisites on Remote PC

1.  **NVIDIA Isaac Sim** installed (4.0+ recommended)
2.  **Git** installed
3.  **Python 3.10** (bundled with Isaac Sim)
4.  **Public IP** or **VPN** access to the machine

---

## 🚀 Step 1: Clone & Setup

On your **Remote PC**:

```bash
# 1. Clone your repo
git clone https://github.com/YOUR_USERNAME/teleop_system.git
cd teleop_system

# 2. Install dependencies (using Isaac Sim's python)
# Note: Use the python.sh provided by Isaac Sim
~/.local/share/ov/pkg/isaac_sim-*/python.sh -m pip install -r requirements.txt
```

---

## 🔄 Step 2: Convert Assets (One-Time)

We need to convert the URDFs (Lingyu/Realman) to USD format so Isaac Sim can use them.

```bash
# 1. Convert Lingyu Robot
~/.local/share/ov/pkg/isaac_sim-*/python.sh convert_to_usd.py \
    robots/lingyu_robot/urdf/urdf20251126.urdf \
    assets/usd/lingyu_robot.usd

# 2. Convert Realman Robot
~/.local/share/ov/pkg/isaac_sim-*/python.sh convert_to_usd.py \
    robots/realman_65/RM65-6F.urdf \
    assets/usd/realman_rm65.usd
```

*Note: Ensure `assets/usd/` directory exists first.*

---

## 📡 Step 3: Start the Teleop Server

The server acts as the bridge between your VR headset and Isaac Sim.

```bash
# Run the server (can be on the same Remote PC or a different cloud instance)
# If running on the same PC as Isaac Sim:
~/.local/share/ov/pkg/isaac_sim-*/python.sh run_server.py --host 0.0.0.0 --backend isaac
```

*   **Port 8000**: Web UI / VR Interface
*   **Port 9000**: TCP Connection for Isaac Sim

---

## 🎮 Step 4: Start Isaac Sim Simulation

Now launch the simulation client which connects to the server.

```bash
# Start Lingyu Robot in Headless Mode (for remote servers)
~/.local/share/ov/pkg/isaac_sim-*/python.sh isaac_sim_client.py \
    --robot_usd assets/usd/lingyu_robot.usd \
    --ee_frame end_effector \
    --host localhost \
    --headless \
    --enable-livestream

# OR Start Realman Robot
~/.local/share/ov/pkg/isaac_sim-*/python.sh isaac_sim_client.py \
    --robot_usd assets/usd/realman_rm65.usd \
    --ee_frame link_6 \
    --host localhost \
    --headless \
    --enable-livestream
```

**What happens:**
1.  Isaac Sim launches (headless).
2.  It loads the robot USD.
3.  It connects to `localhost:9000` (the Teleop Server).
4.  It starts the **WebRTC Livestream**.

---

## 👓 Step 5: Connect VR (Operator)

1.  **Video Stream**:
    *   Download **Omniverse Streaming Client** on your local machine (or iPad).
    *   Connect to the **Remote PC's IP**.
    *   You will see the robot in high quality.

2.  **Controls**:
    *   Open `http://<REMOTE_IP>:8000/vr.html` on your Quest 3.
    *   Enter VR mode.
    *   Your hand movements will be sent to the server → Isaac Sim.

---

## 🐛 Troubleshooting

*   **"Connection Refused"**: Check if `run_server.py` is running and port 9000 is open.
*   **"Asset not found"**: Double check the `convert_to_usd.py` step output.
*   **Slow Video**: Ensure you are using the native Omniverse Streaming Client, not just screen sharing.
