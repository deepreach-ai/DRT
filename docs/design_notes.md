# Teleoperation Design Notes

## Key Tradeoffs

### 1. Incremental (Delta) vs. Absolute Control
**Decision:** Implemented **Incremental (Delta) Control**.
- **Reasoning:** In a teleoperation scenario over the internet (AWS), network latency and jitter are inevitable.
- **Tradeoff:**
  - *Pros:* Safer for packet loss. If a packet is lost, the robot just stops moving (due to the safety gate/deadman switch) rather than jumping to an old absolute position that might arrive late. It also feels more natural for joystick/keyboard control where inputs are typically rates or nudges.
  - *Cons:* No "absolute" positioning guarantees from the client side. Drift can occur between the client's mental model and the robot's actual state, though visual feedback (video stream) mitigates this.

### 2. FastAPI (HTTP) vs. Pure WebSockets vs. gRPC
**Decision:** Hybrid approach using **FastAPI (HTTP)** for commands/status and **WebSockets** for high-frequency streams.
- **Reasoning:**
  - HTTP is stateless, easy to debug, and works through any firewall/load balancer. Perfect for initial setup, status checks, and low-frequency commands.
  - WebSockets are essential for the < 50ms latency loop required for smooth teleoperation.
  - gRPC was considered but adds complexity to the client (need to compile protos) which goes against the "Minimal Client" requirement.
- **Tradeoff:** Slightly more overhead than raw UDP, but much better reliability and ease of deployment on AWS.

### 3. Backend Abstraction
**Decision:** Abstract `RobotBackend` class with `Mock` and `IsaacSim` implementations.
- **Reasoning:** Allows development and testing of the control stack without needing the heavy Isaac Sim environment running.
- **Extensibility:** The `BackendFactory` pattern makes it trivial to add a `RealRobotBackend` (e.g., using `franka_ros` or `ur_rtde`) without touching the core server logic.

## Scaling to Real Robots

To move from Isaac Sim to a physical robot (e.g., UR5, Franka Emika):

1.  **Safety Layer Hardening:**
    -   The current software "Safety Gate" is good, but real robots need hardware-level E-Stops.
    -   Implement "Smooth Stop" trajectory generation rather than sudden zero-velocity commands to protect gearboxes.

2.  **Network Transport:**
    -   Switch the video stream (currently assumed to be out-of-band or handled by Isaac) to WebRTC for sub-500ms latency video.
    -   Implement a jitter buffer on the robot side to smooth out uneven packet arrival times.

3.  **Deployment:**
    -   The Docker container can run directly on the robot's control box (if x86) or on a site-local edge server to minimize last-mile latency.

## Future Improvements (With More Time)

1.  **User Authentication:** Currently, the API is open. I would add JWT-based auth to `teleop_server.py` to prevent unauthorized control.
2.  **Visual Feedback:** Integrate a WebRTC video signaling server directly into the FastAPI app so the client gets video + control in one window.
3.  **Predictive Display:** On the client side, render a "ghost" robot that moves instantly to provide immediate visual feedback while the video stream lags behind.
