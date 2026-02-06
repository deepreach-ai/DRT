# Release Notes - DRT v0.1.0

We are excited to announce the first release of **DRT (Distributed Robot Teleoperation)**, formerly known as Teleop System.

## 🚀 Highlights

*   **Rebranding**: Project officially renamed to **DRT**.
*   **Multi-Modal Inputs**: Full support for VR (Quest 3/3S), Keyboard, and Joystick control.
*   **Heterogeneous Robot Support**: Unified control interface for 6-DoF (SO-ARM101) and 7-DoF (Realman, Lingyu) robots.
*   **Simulation First**: Seamless workflow starting from MuJoCo physics to high-fidelity Isaac Sim.
*   **Cloud Ready**: One-click deployment scripts for AWS EC2.

## 📦 New Features

### VR Interface
*   WebXR-based immersive control.
*   Stereoscopic video streaming (Left/Right eye support).
*   Low-latency data transmission via WebSocket.

### Hardware Integration
*   **Realman RM65/75**: Driver integration for industrial-grade arms.
*   **SO-ARM101**: Continued support for low-cost 6-DoF arms.
*   **Lingyu**: URDF support added.

### Documentation
*   Restructured `docs/` folder for better navigation.
*   New [Quick Start Guide](QUICKSTART.md).
*   Updated deployment scripts for DRT naming convention.

## 🛠️ Changes & Fixes

*   Renamed package configuration to `drt`.
*   Updated deployment scripts (`quick_deploy.sh`, `deploy_remote.sh`) to use `drt` directory structure.
*   Improved WebSocket stability for remote connections.
*   Optimized MJPEG streaming for lower latency.

## ⬇️ Installation

```bash
git clone https://github.com/your-org/drt.git
cd drt
pip install -r requirements.txt
python server/teleop_server.py --backend mujoco
```

## 🤝 Contributors

Thanks to the entire Teleop Team for making this release possible!
