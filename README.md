# DRT: Distributed Robot Teleoperation Platform

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

DRT is a universal, cloud-native teleoperation system designed to bridge the gap between simulation and real-world robotic manipulation. It supports heterogeneous embodiments (6-DoF/7-DoF), multiple input methods (VR, Keyboard, Joystick), and seamless switching between local and cloud environments.

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

For detailed installation and usage instructions, please see the [Quick Start Guide](QUICKSTART.md).

### 1. 安装
```bash
git clone https://github.com/deepreach-ai/DRT.git
cd DRT
pip install -r requirements.txt
```

### 2. 启动服务（单一服务器）
推荐使用集成的 FastAPI 服务器（端口 8000），同时提供 Web UI 与 API。
```bash
# Mock 后端（无需仿真/硬件）
python run_server.py --backend mock

# MuJoCo 仿真（SO-101 示例）
python run_server.py --backend mujoco \
  --mujoco-xml robots/so101.xml \
  --mujoco-ee gripperframe

# MuJoCo 仿真（RM75B VR 优化版）
python run_server.py --backend mujoco \
  --mujoco-xml robots/rm75b_vr_v2.xml \
  --mujoco-ee ee_site

# MuJoCo 仿真（SO-ARM101 VR 优化版）
python run_server.py --backend mujoco \
  --mujoco-xml robots/so101.xml \
  --mujoco-ee gripperframe

# Isaac 后端
python run_server.py --backend isaac
```

### 3. 打开 Web 客户端
在浏览器访问：
```
http://localhost:8000/web/
```
登录（默认）：
- 用户名：operator
- 密码：operator
提示：无需填写 Base URL，或设置为 `http://localhost:8000`。

### 4. 快速验证
```bash
# 服务器健康检查
curl http://localhost:8000/api/v1/statistics | python -m json.tool
```
期望包含：
- backend: mujoco/mock/isaac
- status: connected
- current_position / orientation 等统计信息

### 5. 键盘控制
- Web UI：支持 XYZ 平移 + Yaw 旋转（更易用）
- Python 客户端：支持完整 6-DoF（Pitch/Roll/Yaw + XYZ）

参考：
- 文档：[Keyboard Controls](docs/KEYBOARD_CONTROLS.md)
- Python 客户端：`python client/keyboard_client.py`

## 🥽 VR Teleoperation (Meta Quest 3S)

To achieve low-latency control with the REALMAN RM75B or SO-ARM101:

### 1. Optimize for Latency
The system is pre-configured for **50Hz control loops**. For the best experience:
- Use **USB ADB Reverse** for the lowest latency: `adb reverse tcp:8000 tcp:8000`.
- Or use **ngrok** for remote access: `ngrok http 8000`.

### 2. Enter VR Mode
1.  Open Meta Quest Browser.
2.  Navigate to the VR interface:
    - **RM75B**: `https://<ngrok-id>.ngrok-free.app/web/vr.html?urdf=RM75-B`
    - **SO-ARM101**: `https://<ngrok-id>.ngrok-free.app/web/vr.html?urdf=so101`
3.  Click **"Enter VR Mode"**.
4.  **Controls**: 
    - **Right Joystick**: Rate-based movement (constant speed).
    - **Grip Button (Hold)**: Clutch mode (direct 6-DOF hand tracking).
    - **Trigger**: Gripper control.

### 3. Startup Scripts
For convenience, you can use the provided startup scripts:
- **RM75B**: `./start_rm75b_vr.sh`
- **SO-ARM101**: `./start_so101_vr.sh`

## 📚 Documentation

Detailed guides can be found in the `docs/` directory:

*   **Setup:** [VR Setup](docs/VR_SETUP.md), [Real Robot Setup](docs/SOARM_SETUP.md), [Ngrok (Remote Access)](docs/NGROK_SETUP.md)
*   **Operation:** [Keyboard Controls](docs/KEYBOARD_CONTROLS.md), [Quest 3 Sync](docs/QUEST3_SYNC.md)
*   **Deployment:** [AWS Deployment](docs/AWS_DEPLOYMENT_GUIDE.md), [Isaac Sim Workflow](docs/ISAAC_SIM_WORKFLOW.md)
*   **Validation:** [Latency Testing](docs/LATENCY_TEST_GUIDE.md), [Local Validation](docs/LOCAL_VALIDATION_GUIDE.md)

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgements

*   **NVIDIA:** For Isaac Sim and investment support.
*   **OpenTelevision:** For inspiration on teleoperation frameworks.
*   **Realman & Lingyu:** For hardware support.
