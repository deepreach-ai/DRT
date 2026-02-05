# Universal Teleoperation System - Implementation Guide
## Open-TeleVision Style: Control Multiple Robots via VR/Keyboard in Sim & Real

**Date**: 2026-02-04
**Status**: Core features implemented - Ready for testing!

---

## 🎯 What You Now Have

Your `~/teleop_system` now supports **Open-TeleVision style workflows**:

1. ✅ **Config-driven multi-robot support**
2. ✅ **Unified backend switcher** (sim ↔ real)
3. ✅ **Quest 3 VR controller**
4. ✅ **Isaac Sim + Real robot** with same interface
5. ✅ **MIT License** (already had it!)

---

## 📁 New Files Added

```
teleop_system/
├── server/
│   ├── robot_config.py          ✅ NEW - Config loader system
│   ├── unified_backend.py       ✅ NEW - Switch between sim/real
│
├── configs/robots/
│   ├── realman_rm65.yaml        ✅ NEW - REALMAN with sim+real
│   ├── lingyu_robot_new.yaml   ✅ NEW - 灵域 updated
│   ├── so101_new.yaml           ✅ NEW - SO-101 updated
│
└── client/vr/
    └── quest3_controller.py     ✅ NEW - Quest 3 VR control
```

---

## 🚀 Quick Start Guide

### Step 1: Test Config System (2 minutes)

```bash
cd ~/teleop_system

# Test config loading (using the test script)
python test_new_features.py
```

### Step 2: Test Unified Backend in Simulation (5 minutes)

```bash
# Test with REALMAN in Isaac Sim
# (Note: This requires Isaac Sim to be running if you want to connect, 
# but you can instantiate the backend to test config loading)
python -c "from server.unified_backend import UnifiedRobotBackend; robot = UnifiedRobotBackend('configs/robots/realman_rm65.yaml', mode='simulation'); print('Backend created!')"
```

### Step 3: Test VR Controller (10 minutes)

**Prerequisites:**
- Quest 3 headset
- Same WiFi network as your computer

```bash
# Terminal 1: Start VR controller
python client/vr/quest3_controller.py --ngrok
```

---

## 🔄 Key Workflow: Sim to Real

This is the **killer feature** - test safely in sim, deploy to real:

```python
from server.unified_backend import UnifiedRobotBackend

# 1. Start with simulation
robot = UnifiedRobotBackend("configs/robots/realman_rm65.yaml", mode="simulation")
robot.connect()

# 2. Test with VR in Isaac Sim
# ... move robot around, test everything ...
# ... verify safety systems work ...
# ... collect training data ...

# 3. Everything works? Switch to real robot!
robot.switch_mode("real")

# 4. Now controlling physical REALMAN with same VR interface!
# Same code, same VR controls, same everything!
```

---

## 🔧 Configuration Details

### Robot Config Format

Each robot needs ONE config file with both sim and real backends:

```yaml
robot_name: "ROBOT_NAME"
dof: 6  # or 7

# Simulation backend (Isaac Sim)
isaac_sim:
  usd_path: "assets/usd/robot.usd"
  ee_frame: "tool0"
  prim_path: "/World/Robot"

# Real robot backend
real_robot:
  backend_type: "robot_sdk"  # e.g., "realman_sdk"
  connection:
    host: "192.168.1.100"
    port: 8080

# Workspace limits (used for both sim and real)
workspace_limits:
  x: [-0.65, 0.65]
  y: [-0.65, 0.65]
  z: [0.0, 1.0]

# Controller settings
controller:
  max_velocity: 0.5
  control_rate_hz: 100
```
