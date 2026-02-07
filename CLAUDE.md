# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

DRT (Distributed Robot Teleoperation Platform) is a cloud-native teleoperation system supporting heterogeneous robots (6-DoF/7-DoF), multiple input methods (VR, keyboard, joystick), and seamless switching between simulation and real hardware.

## Build & Run Commands

### Python Server

```bash
# Install dependencies
pip install -r server/requirements.txt

# Run with MuJoCo simulation (primary dev mode)
python server/teleop_server.py --backend mujoco

# Run with mock backend (no-op, for testing)
python server/teleop_server.py --backend mock

# Run with real SO-ARM101 hardware
python server/teleop_server.py --backend soarm --robot-port /dev/tty.usbmodem...

# Run with Isaac Sim
python server/teleop_server.py --backend isaac --isaac-host 0.0.0.0 --isaac-port 9000
```

### Web Client

```bash
cd client/web && python -m http.server 3000
# Open http://localhost:3000
```

### Next.js App (teleops_standalone/)

```bash
cd teleops_standalone
npm install
npm run dev          # Development server
npm run build        # Production build
npm run lint         # ESLint
npm run db:generate  # Generate Drizzle migrations
npm run db:migrate   # Run Drizzle migrations
npm run db:push      # Push schema to database
```

### Docker

```bash
docker-compose up --build    # Single service (port 8000 HTTP, port 9000 Isaac Sim)
```

### Validation & Testing

```bash
python validate_prereqs.py     # Check system dependencies
python run_validation.py       # Full system validation
python detect_hardware.py      # Detect connected robots/cameras
python measure_latency.py      # Benchmark network latency
bash quick_local_test.sh       # Rapid local test
bash quick_validate.sh         # Quick validation suite
```

## Architecture

### System Topology

```
Input (VR/Keyboard/Joystick) → Client (Web/Python) → HTTP/WebSocket → Server (FastAPI)
    → SafetyGate → TeleoperationController → Backend → Robot (Sim or Real)
```

### Server (`server/`)

The server is a FastAPI application (`teleop_server.py`) that processes teleoperation commands through a safety-first pipeline:

- **models.py** — Pydantic data models: `DeltaCommand`, `JointCommand`, `RobotState`, `TeleopStatus`
- **robot_backend.py** — Abstract `RobotBackend` base class + `BackendFactory` (factory pattern for backend selection)
- **control_logic.py** — `TeleoperationController`: processes delta commands, enforces workspace limits and velocity limiting
- **safety_gate.py** — `SafetyGate` (deadman switch with timeout) + `VelocityLimiter`
- **robot_config.py** — `RobotConfig` (YAML-based) + `RobotRegistry` (auto-discovers from `configs/robots/`)
- **web_support.py** — `AuthManager` (token auth), `SessionRecorder` (records sessions to disk), MJPEG streaming
- **vr_video_endpoints.py** — Stereoscopic video streaming for VR headsets

### Backends (`server/backends/`)

All backends implement `RobotBackend`. Swap via `--backend` flag:

| Backend | File | Purpose |
|---------|------|---------|
| `mock` | `mock_backend.py` | No-op testing |
| `mujoco` | `mujoco_backend.py` | Local physics simulation with IK solver |
| `isaac` | `isaac_backend.py` | NVIDIA Isaac Sim via TCP (port 9000) |
| `soarm` | `soarm_backend.py` | Real SO-ARM101 via USB serial (feetech-servo-sdk) |

### Clients (`client/`)

- **web/** — Browser UI (HTML/JS): main console (`index.html`), WebXR VR interface (`vr.html`), 3D visualization (`robot_viz.js`)
- **keyboard_client.py** — Terminal keyboard control (WASD position, IJKL rotation)
- **leader/leader_teleop.py** — Leader-follower dual-arm teleoperation
- **vr/quest3_controller.py** — Quest 3 VR controller integration
- **latency_test_client.py** — Round-trip latency measurement

### Next.js Dashboard (`teleops_standalone/`)

Standalone web app (Next.js 15, TypeScript, React 18) with:
- Clerk authentication
- Drizzle ORM + PostgreSQL (tables: `tx_robots`, `tx_teleop_sessions`, `tx_operator_profiles`, `tx_admin_users`)
- Tailwind CSS + Radix UI components
- Pages: landing, dashboard, admin (robot management), teleop control room (`teleop/[id]/`)

### Robot Configs (`configs/robots/`)

YAML files defining per-robot workspace limits, velocity constraints, Isaac Sim USD paths, and real-robot connection parameters. The `RobotRegistry` auto-discovers these at startup.

## Key Design Decisions

- **HTTP REST over ROS2** for client-server communication (simplicity over ROS ecosystem)
- **Delta commands** (incremental position changes) rather than absolute targets for safety
- **Deadman switch** requires continuous heartbeat to maintain control; timeout triggers safety stop
- **Abstract backend factory** enables identical control stack across simulation and real hardware
- **Workspace bounding box** enforced in `TeleoperationController` prevents out-of-bounds motion

## Environment Variables

| Variable | Purpose |
|----------|---------|
| `TELEOP_BACKEND` | Backend type: mock, isaac, mujoco, soarm |
| `TELEOP_ISAAC_HOST` / `TELEOP_ISAAC_PORT` | Isaac Sim connection |
| `TELEOP_AUTH_DISABLED` | Bypass authentication |
| `TELEOP_RECORDINGS_DIR` | Session recording output directory |
