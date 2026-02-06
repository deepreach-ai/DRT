# Design Notes

## 1. Key Tradeoffs

### A. HTTP vs. WebSocket vs. TCP
- **Decision**: Used HTTP (FastAPI) for the Client-Server interface and raw TCP for the Server-Robot interface.
- **Rationale**:
  - **Client-Server (HTTP)**: Simplifies client implementation (stateless, standard REST API). While WebSockets offer lower latency, HTTP is sufficient for non-haptic teleoperation (~20Hz) and easier to debug/load balance.
  - **Server-Robot (TCP)**: Raw TCP was chosen for communication with the Isaac Sim container to minimize overhead and allow for a simple custom protocol without the weight of ROS 2 bridges for this specific minimal scope.
- **Alternative**: Using ROS 2 for everything.
  - *Tradeoff*: ROS 2 provides better ecosystem integration but adds significant complexity (DDS configuration, middleware overhead) which is overkill for a minimal take-home assignment.

### B. Delta Control vs. Absolute Pose
- **Decision**: Implemented Incremental (Delta) Control.
- **Rationale**: Safer for teleoperation. If the network lags, the robot stops (due to deadman switch) rather than jumping to an old absolute timestamped pose. It also feels more natural for keyboard/gamepad control.

### C. Backend Abstraction
- **Decision**: Python `abc` (Abstract Base Class) with a Factory Pattern.
- **Rationale**: Allows clean separation. The core logic doesn't know if it's talking to Isaac Sim, a Mock, or a Real Robot. This makes testing easier (using Mock) and future hardware integration seamless.

## 2. Scalability to Real Robots

To migrate this system to a real Franka Emika Panda:

1. **Create a `RealFrankaBackend` class**:
   - Inherit from `RobotBackend`.
   - Implement `send_target_pose()` using `franka_ros` or `libfranka`.
2. **Update Safety Limits**:
   - Real robots need stricter collision checking (Self-Collision + Environment).
   - Currently, we use a simple Bounding Box. For real robots, we would integrate a proper planning scene (e.g., MoveIt) or Signed Distance Fields (SDF).
3. **Network Security**:
   - The current setup allows unencrypted TCP/HTTP. For real robots, we would need VPNs (WireGuard) or TLS/mTLS encryption to prevent unauthorized control.

## 3. Future Improvements (With More Time)

1. **WebRTC for Video**:
   - Currently using Native Streaming (Omniverse Client). Integrating WebRTC directly into a web frontend would remove the need for a separate client app.
2. **Visual Servoing / Haptic Feedback**:
   - Sending force feedback to the client (if using a haptic device).
3. **Container Orchestration**:
   - Move from Docker Compose to Kubernetes (EKS) for better auto-scaling and resilience.
4. **Testing**:
   - Add Integration Tests that spin up the Mock Backend and verify control loop frequency and safety gate triggering.
