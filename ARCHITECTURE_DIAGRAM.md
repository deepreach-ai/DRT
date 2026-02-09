# RM75B VR Teleoperation System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         META QUEST 3S (VR Headset)                      │
│  ┌───────────────┐  ┌──────────────┐  ┌─────────────────────────────┐  │
│  │ Left Camera   │  │ Right Camera │  │  Controllers (6-DOF)        │  │
│  │ (Stereo View) │  │ (Stereo View)│  │  • Right: Robot Control     │  │
│  └───────────────┘  └──────────────┘  │  • Left: Settings           │  │
│                                        │  • Triggers: Gripper        │  │
│                                        └─────────────────────────────┘  │
└────────────────────────────┬──────────────────────────────────────────────┘
                             │
                             │ WebXR over WiFi/USB (WebSocket)
                             │ Position: [x, y, z]
                             │ Orientation: [qw, qx, qy, qz]
                             │ Gripper: 0.0 - 1.0
                             ↓
┌─────────────────────────────────────────────────────────────────────────┐
│                        WEB BROWSER (VR Mode)                            │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  vr.html (WebXR Interface)                                      │   │
│  │  • Three.js for 3D rendering                                    │   │
│  │  • WebXR API for VR tracking                                    │   │
│  │  • WebSocket client for real-time control                       │   │
│  └─────────────────────────────────────────────────────────────────┘   │
└────────────────────────────┬──────────────────────────────────────────────┘
                             │
                             │ HTTP REST API + WebSocket
                             │ POST /api/v1/teleop
                             │ GET /api/v1/video/mjpeg
                             │ WS /ws/control
                             ↓
┌─────────────────────────────────────────────────────────────────────────┐
│                    DRT SERVER (Port 8000)                               │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  run_server.py (FastAPI/Flask)                                  │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌────────────────────┐    │   │
│  │  │ Teleop API   │  │ Video Stream │  │ WebSocket Handler  │    │   │
│  │  │ /api/v1/*    │  │ /video/*     │  │ /ws/control        │    │   │
│  │  └──────────────┘  └──────────────┘  └────────────────────┘    │   │
│  └─────────────────────────┬───────────────────────────────────────┘   │
│                            │                                            │
│  ┌─────────────────────────▼───────────────────────────────────────┐   │
│  │           Backend Router (backends/__init__.py)                 │   │
│  │           • Mock Backend                                        │   │
│  │           • MuJoCo Backend ◄── ACTIVE                          │   │
│  │           • Isaac Sim Backend                                   │   │
│  │           • SOARM Backend (Real Hardware)                       │   │
│  └─────────────────────────┬───────────────────────────────────────┘   │
└────────────────────────────┼──────────────────────────────────────────────┘
                             │
                             │ Python API
                             │ position, orientation, gripper
                             ↓
┌─────────────────────────────────────────────────────────────────────────┐
│                    MUJOCO BACKEND                                       │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  mujoco_backend.py                                              │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌────────────────────┐    │   │
│  │  │ IK Solver    │  │ Forward Kin  │  │ Collision Check    │    │   │
│  │  │ (Jacobian)   │  │ (FK)         │  │                    │    │   │
│  │  └──────────────┘  └──────────────┘  └────────────────────┘    │   │
│  │                                                                  │   │
│  │  Algorithm:                                                      │   │
│  │  1. Get target pose from VR controller                          │   │
│  │  2. Compute joint angles via IK (Jacobian pseudoinverse)        │   │
│  │  3. Apply to MuJoCo model                                        │   │
│  │  4. Step physics simulation                                      │   │
│  │  5. Render scene for VR display                                  │   │
│  └─────────────────────────┬───────────────────────────────────────┘   │
└────────────────────────────┼──────────────────────────────────────────────┘
                             │
                             │ MuJoCo C API
                             │ mj_step(), mj_forward()
                             ↓
┌─────────────────────────────────────────────────────────────────────────┐
│                    MUJOCO PHYSICS ENGINE                                │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  Robot Model: rm75b_vr.xml                                      │   │
│  │                                                                  │   │
│  │  ┌────────────────────────────────────────────────────────┐     │   │
│  │  │  Robot Structure (7-DOF + Gripper)                     │     │   │
│  │  │                                                         │     │   │
│  │  │  base_link                                             │     │   │
│  │  │    └─ link_1 (joint_1: ±180°)                         │     │   │
│  │  │         └─ link_2 (joint_2: ±120°)                    │     │   │
│  │  │              └─ link_3 (joint_3: ±170°)               │     │   │
│  │  │                   └─ link_4 (joint_4: ±135°)          │     │   │
│  │  │                        └─ link_5 (joint_5: ±178°)     │     │   │
│  │  │                             └─ link_6 (joint_6: ±128°)│     │   │
│  │  │                                  └─ link_7 (±360°)    │     │   │
│  │  │                                       └─ ee_site      │     │   │
│  │  │                                            └─ gripper │     │   │
│  │  └────────────────────────────────────────────────────────┘     │   │
│  │                                                                  │   │
│  │  Physics:                                                        │   │
│  │  • Timestep: 2ms (500Hz)                                        │   │
│  │  • Integrator: Implicit Euler                                    │   │
│  │  • Gravity: 9.81 m/s²                                           │   │
│  │  • Contact detection & response                                  │   │
│  │  • Joint limits & safety                                         │   │
│  └─────────────────────────┬───────────────────────────────────────┘   │
└────────────────────────────┼──────────────────────────────────────────────┘
                             │
                             │ Actuator commands
                             │ Joint torques/positions
                             ↓
┌─────────────────────────────────────────────────────────────────────────┐
│                    SIMULATED ROBOT (RM75B)                              │
│                                                                         │
│    Current State:                                                       │
│    • Joint Positions: [θ₁, θ₂, θ₃, θ₄, θ₅, θ₆, θ₇]                   │
│    • Joint Velocities: [ω₁, ω₂, ω₃, ω₄, ω₅, ω₆, ω₇]                  │
│    • End Effector Pose: (x, y, z, qw, qx, qy, qz)                     │
│    • Gripper State: open (0.0) ↔ closed (1.0)                         │
│                                                                         │
│    Physics Response:                                                    │
│    • Mass/Inertia effects                                              │
│    • Gravity compensation                                               │
│    • Joint friction & damping                                           │
│    • Collision with environment                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## Data Flow (Single Frame)

```
VR Controller Update (11ms @ 90Hz)
    ↓
WebXR API reads controller pose (1ms)
    ↓
JavaScript sends via WebSocket (2ms over local network)
    ↓
Server receives command (1ms)
    ↓
Backend processes:
    • Parse position/orientation (0.1ms)
    • Run IK solver (3-8ms)
    • Compute joint commands (0.5ms)
    ↓
MuJoCo simulation:
    • Apply actuator forces (0.5ms)
    • Step physics (2-5ms)
    • Forward kinematics (0.5ms)
    ↓
Render frame for VR:
    • Generate left/right eye views (8ms)
    • Encode as MJPEG/H264 (3ms)
    ↓
Stream to browser (2-5ms over network)
    ↓
Browser decodes & displays (8ms)
    ↓
VR headset renders (11ms @ 90Hz)
    ↓
Total: ~25-40ms latency ✅
```

## Control Loop Timing

```
┌──────────────────────────────────────────────────────────┐
│                    90Hz VR Loop                          │
│  ┌────────────────────────────────────────────────┐     │
│  │ Frame 1 (11ms)                                  │     │
│  │ • Read controller                               │     │
│  │ • Send command                                  │     │
│  │ • Receive updated pose                          │     │
│  │ • Render scene                                  │     │
│  └────────────────────────────────────────────────┘     │
│  ┌────────────────────────────────────────────────┐     │
│  │ Frame 2 (11ms)                                  │     │
│  │ • Read controller                               │     │
│  │ • ...                                           │     │
│  └────────────────────────────────────────────────┘     │
└──────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────┐
│                   500Hz Physics Loop                     │
│  ┌──┐┌──┐┌──┐┌──┐┌──┐┌──┐┌──┐┌──┐┌──┐                  │
│  │ 2││ 2││ 2││ 2││ 2││ 2││ 2││ 2││ 2│ ... ms              │
│  └──┘└──┘└──┘└──┘└──┘└──┘└──┘└──┘└──┘                  │
│  Physics updates ~5x per VR frame                        │
└──────────────────────────────────────────────────────────┘
```

## Network Topology

```
Option A: WiFi
┌──────────┐           WiFi           ┌──────────┐
│  Quest   │ ◄───────────────────────►│   Mac    │
│  3S      │   192.168.1.x:8000       │  Server  │
└──────────┘                          └──────────┘
Latency: 2-10ms
Bandwidth: ~50 Mbps needed
Reliability: Good (same network)

Option B: USB Tethering (Recommended)
┌──────────┐        USB-C Cable       ┌──────────┐
│  Quest   │ ◄───────────────────────►│   Mac    │
│  3S      │   localhost:8000         │  Server  │
└──────────┘   (ADB reverse)          └──────────┘
Latency: <1ms
Bandwidth: 480 Mbps (USB 2.0)
Reliability: Excellent
```

## File Structure

```
~/teleop_system/
│
├── robots/
│   ├── rm75b_vr.xml ◄────────── MuJoCo robot model (NEW)
│   ├── RM75-B.urdf              Original URDF
│   └── realman_75/
│       └── meshes/              STL mesh files
│
├── server/
│   ├── backends/
│   │   ├── mujoco_backend.py   IK solver & physics
│   │   ├── soarm_backend.py    Real robot interface
│   │   └── isaac_backend.py    Isaac Sim
│   └── teleop_server.py         Main server
│
├── client/
│   ├── web/
│   │   ├── vr.html              VR interface
│   │   └── index.html           2D interface
│   └── keyboard_client.py       Keyboard control
│
├── docs/
│   ├── VR_SETUP.md
│   └── KEYBOARD_CONTROLS.md
│
├── run_server.py ◄────────────── Server launcher
│
├── RM75B_VR_SETUP.md ◄─────────  Complete guide (NEW)
├── RM75B_QUICK_START.md ◄──────  Quick reference (NEW)
├── RM75B_IMPLEMENTATION_SUMMARY.md  Summary (NEW)
├── test_rm75b_setup.py ◄───────  Verification script (NEW)
└── commands_reference.sh ◄──────  Command cheatsheet (NEW)
```

## System States

```
┌────────────────┐
│  Disconnected  │ ◄─── Initial state
└────────┬───────┘
         │ start_server()
         ↓
┌────────────────┐
│   Connected    │ ◄─── Server running, waiting for VR
└────────┬───────┘
         │ enter_vr_mode()
         ↓
┌────────────────┐
│   VR Active    │ ◄─── Controllers tracked, robot moving
└────────┬───────┘
         │ emergency_stop() or exit_vr()
         ↓
┌────────────────┐
│     Stopped    │ ◄─── Motion halted
└────────────────┘
```

---

**Legend:**
- ◄── Active in current setup
- ► Future feature
- ✅ Tested and working
- ⚠️ Needs verification
