"""
Realman RM75B Robot Backend

Communicates with the Realman RM75B (7-DOF) over Ethernet using the
Realman Python SDK (robotic_arm v1.1.4+).

Motion pipeline (seeded general IK, single-step mode):
  pose target
    → rm_algo_inverse_kinematics(q_in=last_joints, flag=1/euler)
      (single-step mode set once at connect via rm_algo_set_redundant_parameter_traversal_mode)
    → rm_movej(solved_joints_deg, v, 0, 0, 0)
  Fallback on IK failure: rm_movej_p (firmware IK, unseeded)

Using the general seeded IK (not the arm-angle variant) allows the solver to
adjust the 7th redundant DoF freely, so all joints participate in every motion.
Single-step mode is tuned for continuous near-pose control and runs fast.

Home move:
  On connect, the arm moves to a configurable home configuration before the
  teleoperation loop starts.  This seeds the IK from a mid-range joint config
  where all 7 joints are at non-trivial angles, preventing the solver from
  locking out proximal joints for the first session.

Install SDK:
    pip install robotic-arm

Robot default IP: 192.168.1.18, port: 8080

SDK key facts (verified against v1.1.4):
  - RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)                     -- thread mode arg
  - arm.rm_create_robot_arm(ip, port)                                  -- returns handle
  - arm.rm_get_current_arm_state()                                     -- (int, {"joint":[deg*7], "pose":[x,y,z,rx,ry,rz]})
  - arm.rm_algo_set_redundant_parameter_traversal_mode(False)          -- single-step IK mode for continuous control
  - arm.rm_algo_inverse_kinematics(params)                             -- (int, [joint_deg*7]), params has q_in seed
  - arm.rm_movej(joint_deg, v_pct, r, connect, block)                  -- joint-space move
  - arm.rm_movej_p(pose_6f, v_pct, r, connect, block)                  -- Cartesian fallback
  - arm.rm_set_gripper_position(pos, block, timeout)                   -- 1-1000
  - arm.rm_delete_robot_arm()                                          -- disconnect
"""
from typing import Tuple, Optional, Dict, Any, List
from ctypes import c_float
import numpy as np
import time
import threading
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot_backend import RobotBackend, BackendStatus

try:
    from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e
    from Robotic_Arm.rm_ctypes_wrap import rm_inverse_kinematics_params_t
    REALMAN_SDK_AVAILABLE = True
except ImportError:
    REALMAN_SDK_AVAILABLE = False
    rm_inverse_kinematics_params_t = None
    print("[RealmanBackend] Warning: Realman SDK not found. Install: pip install robotic-arm")

try:
    import transforms3d.euler as t3e
    TRANSFORMS3D_AVAILABLE = True
except ImportError:
    TRANSFORMS3D_AVAILABLE = False

_MAX_ROBOT_LINEAR_VEL = 0.1   # m/s — maps velocity_limit to 1-100% for SDK
_GRIPPER_OPEN   = 1
_GRIPPER_CLOSED = 1000

# Home configuration: joint angles in degrees that place all 7 joints at
# non-trivial mid-range angles so the IK seed engages the full kinematic chain.
# Tune for your specific RM75B mounting and workspace.
#   J1=0   shoulder pan      (straight forward)
#   J2=-30 shoulder lift     (slightly raised)
#   J3=0   upper-arm roll    (neutral)
#   J4=90  elbow flex        (90° bend)
#   J5=0   forearm roll      (neutral)
#   J6=45  wrist pitch       (45° down)
#   J7=0   wrist roll        (neutral)
_DEFAULT_HOME_JOINTS_DEG: List[float] = [0.0, -30.0, 0.0, 90.0, 0.0, 45.0, 0.0]
_HOME_VELOCITY_PCT = 20   # slow/safe homing speed (1-100%)


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def _quat_to_euler_xyz(quat_wxyz: np.ndarray) -> np.ndarray:
    """Quaternion [w,x,y,z] -> Euler [rx,ry,rz] radians, extrinsic XYZ."""
    if TRANSFORMS3D_AVAILABLE:
        w, x, y, z = quat_wxyz
        rx, ry, rz = t3e.quat2euler(np.array([w, x, y, z]), axes='sxyz')
        return np.array([rx, ry, rz])
    w, x, y, z = quat_wxyz
    rx = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    ry = np.arcsin(np.clip(2*(w*y - z*x), -1.0, 1.0))
    rz = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.array([rx, ry, rz])


def _euler_xyz_to_quat(euler_xyz: np.ndarray) -> np.ndarray:
    """Euler [rx,ry,rz] radians, extrinsic XYZ -> quaternion [w,x,y,z]."""
    if TRANSFORMS3D_AVAILABLE:
        rx, ry, rz = euler_xyz
        return t3e.euler2quat(rx, ry, rz, axes='sxyz')
    rx, ry, rz = euler_xyz
    cy, sy = np.cos(rz*0.5), np.sin(rz*0.5)
    cp, sp = np.cos(ry*0.5), np.sin(ry*0.5)
    cr, sr = np.cos(rx*0.5), np.sin(rx*0.5)
    return np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    ])


def _vel_to_percent(velocity_ms: float) -> int:
    """Convert m/s to 1-100 integer percentage for the SDK."""
    return max(1, min(100, int(velocity_ms / _MAX_ROBOT_LINEAR_VEL * 100)))


def _min_jerk_alpha(t: float, T: float) -> float:
    """
    Normalized minimum-jerk position scalar (0 → 1) at time t for
    trajectory duration T.  Uses the 5th-order polynomial:
        s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
    which gives zero velocity and acceleration at both endpoints.
    """
    if T <= 0.0 or t >= T:
        return 1.0
    tau = t / T
    return 10*tau**3 - 15*tau**4 + 6*tau**5


def _slerp_quat(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation between unit quaternions [w,x,y,z]."""
    dot = float(np.dot(q0, q1))
    if dot < 0.0:          # take the shorter arc
        q1 = -q1
        dot = -dot
    if dot > 0.9995:       # nearly identical — linear interpolation is fine
        result = q0 + t * (q1 - q0)
        n = np.linalg.norm(result)
        return result / n if n > 0 else q0
    theta_0 = np.arccos(np.clip(dot, -1.0, 1.0))
    theta   = theta_0 * t
    sin_0   = np.sin(theta_0)
    s0 = np.sin(theta_0 - theta) / sin_0
    s1 = np.sin(theta) / sin_0
    return s0 * q0 + s1 * q1


# ---------------------------------------------------------------------------
# Backend
# ---------------------------------------------------------------------------

class RealmanBackend(RobotBackend):
    """
    Backend for Realman RM75B (7-DOF) robot arm.

    Motion pipeline
    ---------------
    • send_target_pose() — called by the teleoperation server at ~20 Hz.
      Validates the new goal (jump-rejection) then stores it and returns
      immediately.  actual_commanded_position/orientation are updated here
      so the controller's virtual position stays synced to the goal.

    • _motion_loop() — background thread running at _MOTION_HZ (50 Hz).
      Computes min-jerk interpolated waypoints between trajectory start and
      current goal, then calls rm_algo_inverse_kinematics (seeded) → rm_movej.
      A min-change threshold suppresses SDK calls when the arm is stationary.

    Home move
    ---------
    On connect(), the arm moves to _home_joints_deg before the control loop
    starts.  This ensures all joints start from a mid-range configuration
    where the general seeded IK can engage all 7 DoF.

    Gripper
    -------
    rm_set_gripper_position (1-1000).  Commands are sent only when the
    requested position changes to avoid spamming the controller.
    """

    _POLL_HZ   = 5    # state-polling thread rate
    _MOTION_HZ = 100  # motion command rate — match VR 90 Hz to minimise lag

    # Jump-rejection thresholds (tracking-loss protection)
    _MAX_POS_JUMP = 0.05   # m  (5 cm)
    _MAX_ROT_JUMP = 0.50   # rad (~28°)

    # Min-change thresholds (oscillation suppression).
    # Tuned for 90 Hz VR sends: each frame delta is ~11 ms × hand velocity.
    # At 1 cm/s hand speed → 0.11 mm/frame; threshold at 0.05 mm keeps slow
    # motion responsive while filtering truly stationary noise.
    _MIN_POS_CHANGE = 0.00005  # m   (0.05 mm)
    _MIN_ROT_CHANGE = 0.00050  # rad (~0.03°)

    def __init__(
        self,
        host: str = "192.168.1.18",
        port: int = 8080,
        name: str = "realman_rm75b",
        home_on_connect: bool = True,
        home_joints: Optional[List[float]] = None,
    ):
        super().__init__(name)
        self.host = host
        self.port = int(port)
        self._arm: Optional[Any] = None
        # RLock: motion thread holds _lock while calling _maybe_send_gripper,
        # which also acquires _lock — re-entrancy is required.
        self._lock = threading.RLock()

        # ---- Home configuration -------------------------------------------
        self._home_on_connect  = home_on_connect
        self._home_joints_deg  = home_joints if home_joints is not None \
                                 else list(_DEFAULT_HOME_JOINTS_DEG)

        # ---- Goal (updated by send_target_pose) ----------------------------
        self._goal_pos:      Optional[np.ndarray] = None
        self._goal_ori:      Optional[np.ndarray] = None
        self._goal_velocity: float = 0.1
        self._goal_gripper:  float = -1.0
        self._goal_lock = threading.Lock()
        self._goal_last_updated: float = 0.0  # wall-clock time of last send_target_pose call

        # ---- Last position actually sent to SDK ----------------------------
        self._last_sent_pos: Optional[np.ndarray] = None
        self._last_sent_ori: Optional[np.ndarray] = None
        self._last_gripper_pos: int = -1

        # ---- Seeded IK state -----------------------------------------------
        # Joint angles in degrees from the last successfully sent command.
        # Passed as q_in to rm_algo_inverse_kinematics so the solver always
        # picks the nearest continuous solution.
        self._last_sent_joints_deg: Optional[list] = None
        self._ik_fail_count: int = 0

        # Exposed for controller sync in teleop_server.py
        self.actual_commanded_position:    Optional[np.ndarray] = None
        self.actual_commanded_orientation: Optional[np.ndarray] = None

        # ---- State cache (poll thread) -------------------------------------
        self._cached_position:    Optional[np.ndarray] = None
        self._cached_orientation: Optional[np.ndarray] = None
        self._cached_joints:      np.ndarray = np.zeros(7)
        self._cache_lock  = threading.Lock()
        self._poll_thread: Optional[threading.Thread] = None
        self._stop_poll   = threading.Event()

        # ---- Motion thread -------------------------------------------------
        self._motion_thread: Optional[threading.Thread] = None
        self._stop_motion   = threading.Event()

        self.command_count = 0

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        if not REALMAN_SDK_AVAILABLE:
            print("[RealmanBackend] Cannot connect: Realman SDK not installed.")
            self.status = BackendStatus.ERROR
            return False

        try:
            print(f"[RealmanBackend] Connecting to RM75B at {self.host}:{self.port} ...")
            self.status = BackendStatus.CONNECTING

            self._arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
            handle = self._arm.rm_create_robot_arm(self.host, self.port)

            if handle is None or handle.id <= 0:
                raise RuntimeError(f"rm_create_robot_arm returned invalid handle: {handle}")

            err, state = self._arm.rm_get_current_arm_state()
            if err != 0:
                raise RuntimeError(f"rm_get_current_arm_state returned error: {err}")

            # Set single-step IK mode: designed for continuous near-pose control,
            # auto-adjusts the 7th redundant DoF, fast computation.
            self._arm.rm_algo_set_redundant_parameter_traversal_mode(False)
            print("[RealmanBackend] IK mode: single-step (continuous control)")

            # Configure gripper travel range for the attached gripper (Changingtek or other).
            # rm_set_gripper_route persists across power cycles.  Setting 0..1000 ensures
            # the full position range is available; narrow factory defaults cause small
            # actuation on third-party grippers.
            gr_err = self._arm.rm_set_gripper_route(0, 1000)
            if gr_err != 0:
                print(f"[RealmanBackend] Warning: rm_set_gripper_route returned {gr_err} "
                      f"(gripper may not be connected yet)")
            else:
                print("[RealmanBackend] Gripper route set: 0..1000")

            # Log current gripper state for diagnostics
            gs_err, gs = self._arm.rm_get_gripper_state()
            if gs_err == 0:
                print(f"[RealmanBackend] Gripper state: {gs}")
            else:
                print(f"[RealmanBackend] Gripper state query returned {gs_err}")

            # Seed state from current robot pose before any home move
            self._seed_state_from_sdk(state)

            self.status = BackendStatus.CONNECTED
            self.last_update_time = time.time()
            print(f"[RealmanBackend] Connected. Initial pose: "
                  f"{[round(v, 4) for v in state['pose']]}")

            # Home move: go to mid-range config so all joints are engaged
            if self._home_on_connect:
                self._move_to_home()

            self._stop_poll.clear()
            self._poll_thread = threading.Thread(
                target=self._poll_state_loop, daemon=True, name="realman-poll"
            )
            self._poll_thread.start()

            self._stop_motion.clear()
            self._motion_thread = threading.Thread(
                target=self._motion_loop, daemon=True, name="realman-motion"
            )
            self._motion_thread.start()

            return True

        except Exception as e:
            print(f"[RealmanBackend] Connection failed: {e}")
            import traceback; traceback.print_exc()
            self._arm = None
            self.status = BackendStatus.ERROR
            return False

    def _seed_state_from_sdk(self, state: dict):
        """Seed all tracking state from a freshly-read SDK arm state dict."""
        pose = state["pose"]
        pos  = np.array(pose[:3])
        ori  = _euler_xyz_to_quat(np.array(pose[3:]))
        joints_deg = list(state["joint"])

        with self._cache_lock:
            self._cached_position    = pos.copy()
            self._cached_orientation = ori.copy()
            self._cached_joints      = np.deg2rad(joints_deg)

        self._last_sent_pos  = pos.copy()
        self._last_sent_ori  = ori.copy()
        self._goal_pos       = pos.copy()
        self._goal_ori       = ori.copy()
        self.actual_commanded_position    = pos.copy()
        self.actual_commanded_orientation = ori.copy()
        self._last_sent_joints_deg = joints_deg
        self._ik_fail_count = 0

    def _move_to_home(self) -> bool:
        """
        Blocking move to home configuration before teleoperation starts.
        Re-seeds all IK and trajectory state from the post-home pose.
        Returns True on success, False if the move failed (control still works
        from current pose in that case).
        """
        # Read current joints so we can show whether a real move is needed
        _err0, _s0 = self._arm.rm_get_current_arm_state()
        _cur_j = [round(v, 1) for v in _s0["joint"]] if _err0 == 0 else "?"
        print(f"[RealmanBackend] Current joints: {_cur_j}")
        print(f"[RealmanBackend] Moving to home: {self._home_joints_deg} "
              f"at {_HOME_VELOCITY_PCT}% speed ...")
        with self._lock:
            err = self._arm.rm_movej(
                self._home_joints_deg,
                _HOME_VELOCITY_PCT,
                0,   # blending radius
                0,   # trajectory_connect
                1,   # block=1: wait until done
            )
        if err != 0:
            print(f"[RealmanBackend] Home move failed (err={err}), "
                  f"starting from current pose")
            return False

        # Re-read state after home move and re-seed everything
        err2, state = self._arm.rm_get_current_arm_state()
        if err2 != 0:
            print(f"[RealmanBackend] Could not read state after home move (err={err2})")
            return False

        self._seed_state_from_sdk(state)
        print(f"[RealmanBackend] Home reached. Pose: "
              f"{[round(v, 4) for v in state['pose']]}")
        return True

    def _poll_state_loop(self):
        """Background thread: polls robot state at _POLL_HZ and caches it."""
        interval = 1.0 / self._POLL_HZ
        while not self._stop_poll.is_set():
            try:
                with self._lock:
                    err, state = self._arm.rm_get_current_arm_state()
                if err == 0:
                    pose = state["pose"]
                    with self._cache_lock:
                        self._cached_position    = np.array(pose[:3])
                        self._cached_orientation = _euler_xyz_to_quat(np.array(pose[3:]))
                        self._cached_joints      = np.deg2rad(state["joint"])
                    self.last_update_time = time.time()
            except Exception as e:
                print(f"[RealmanBackend] Poll error: {e}")
            self._stop_poll.wait(interval)

    def disconnect(self):
        self._stop_motion.set()
        self._stop_poll.set()
        if self._motion_thread and self._motion_thread.is_alive():
            self._motion_thread.join(timeout=2.0)
        if self._poll_thread and self._poll_thread.is_alive():
            self._poll_thread.join(timeout=2.0)
        try:
            if self._arm is not None:
                self._arm.rm_delete_robot_arm()
                print("[RealmanBackend] Disconnected from RM75B.")
        except Exception as e:
            print(f"[RealmanBackend] Error during disconnect: {e}")
        finally:
            self._arm = None
            self.status = BackendStatus.DISCONNECTED

    # ------------------------------------------------------------------
    # Motion interpolation thread
    # ------------------------------------------------------------------

    def _motion_loop(self):
        """Background thread: forwards latest goal to arm at _MOTION_HZ."""
        dt = 1.0 / self._MOTION_HZ
        while not self._stop_motion.is_set():
            t0 = time.time()
            try:
                self._motion_tick()
            except Exception as e:
                print(f"[RealmanBackend] Motion loop error: {e}")
            self._stop_motion.wait(max(0.0, dt - (time.time() - t0)))

    def _motion_tick(self):
        """Forward the latest goal directly to the arm at _MOTION_HZ.

        Uses connect=0 so every command is self-contained — no firmware queue
        builds up.  When the next command arrives before the previous move
        finishes, the firmware interrupts and redirects, giving real-time
        servo behaviour.

        Stale-goal guard: if no fresh send_target_pose call has arrived in
        >150 ms (grip released / heartbeat only), skip sending so the arm
        comes to rest at the last commanded position instead of re-sending
        stale targets indefinitely.
        """
        with self._goal_lock:
            if self._goal_pos is None:
                return
            # Grip released — stop as soon as goal is stale
            if time.time() - self._goal_last_updated > 0.15:
                return
            pos      = self._goal_pos.copy()
            ori      = self._goal_ori.copy()
            velocity = self._goal_velocity
            gripper  = self._goal_gripper

        # Min-change filter: suppress SDK call when arm is stationary
        if self._last_sent_pos is not None and self._last_sent_ori is not None:
            pos_delta = np.linalg.norm(pos - self._last_sent_pos)
            rot_delta = np.linalg.norm(
                _quat_to_euler_xyz(ori) - _quat_to_euler_xyz(self._last_sent_ori)
            )
            if pos_delta < self._MIN_POS_CHANGE and rot_delta < self._MIN_ROT_CHANGE:
                if gripper >= 0.0:
                    self._maybe_send_gripper(gripper)
                return

        self._send_pose_to_arm(pos, ori, velocity, gripper)
        self._last_sent_pos = pos.copy()
        self._last_sent_ori = ori.copy()

    def _send_pose_to_arm(self, position: np.ndarray, orientation: np.ndarray,
                           velocity_limit: float, gripper_state: float):
        """
        Send pose via seeded general IK → rm_movej.
        Fallback: rm_movej_p (firmware IK, unseeded) on IK failure.

        Uses rm_algo_inverse_kinematics with q_in seed (single-step mode set at
        connect).  The solver auto-adjusts the 7th redundant DoF to minimise
        joint movement from the current configuration, distributing motion across
        all 7 joints rather than saturating only the distal wrist joints.
        """
        euler  = _quat_to_euler_xyz(orientation)
        pose_6f = [
            float(position[0]), float(position[1]), float(position[2]),
            float(euler[0]),    float(euler[1]),    float(euler[2]),
        ]
        v_pct   = _vel_to_percent(velocity_limit)
        sent_ok = False

        # --- Primary path: seeded general IK → rm_movej ---------------------
        if (self._last_sent_joints_deg is not None and
                rm_inverse_kinematics_params_t is not None):
            try:
                params        = rm_inverse_kinematics_params_t()
                params.q_in   = (c_float * 7)(*self._last_sent_joints_deg)
                params.q_pose = pose_6f   # [x,y,z,rx,ry,rz] m+rad
                params.flag   = 1         # 1 = euler angles

                with self._lock:
                    ret, joints_deg = self._arm.rm_algo_inverse_kinematics(params)

                if ret == 0:
                    with self._lock:
                        # connect=0: each command is independent — no queue
                        # builds up.  The firmware interrupts the current move
                        # when a new target arrives, giving real-time tracking.
                        err = self._arm.rm_movej(joints_deg, v_pct, 0, 0, 0)
                    if err == 0:
                        self._last_sent_joints_deg = joints_deg
                        self._ik_fail_count = 0
                        sent_ok = True
                    else:
                        print(f"[RealmanBackend] rm_movej error {err}, "
                              f"falling back to rm_movej_p")
                else:
                    self._ik_fail_count += 1
                    if self._ik_fail_count % 10 == 1:
                        print(f"[RealmanBackend] IK ret={ret} "
                              f"(fail #{self._ik_fail_count}), "
                              f"falling back to rm_movej_p")
                    # After sustained failures, re-seed from live joint state
                    if self._ik_fail_count >= 20:
                        with self._cache_lock:
                            q_rad = self._cached_joints.copy()
                        self._last_sent_joints_deg = list(np.degrees(q_rad))
                        self._ik_fail_count = 0
                        print("[RealmanBackend] IK seed refreshed from live joint state")

            except Exception as e:
                print(f"[RealmanBackend] Seeded IK exception: {e}")

        # --- Fallback: rm_movej_p (firmware IK, unseeded) -------------------
        if not sent_ok:
            with self._lock:
                err = self._arm.rm_movej_p(pose_6f, v_pct, 0, 0, 0)  # connect=0
            if err != 0:
                print(f"[RealmanBackend] rm_movej_p error: {err}")
                return

        if gripper_state >= 0.0:
            self._maybe_send_gripper(gripper_state)
        self.last_update_time = time.time()

    # ------------------------------------------------------------------
    # Control (called by teleoperation server)
    # ------------------------------------------------------------------

    def send_target_pose(
        self,
        position: np.ndarray,
        orientation: np.ndarray,
        velocity_limit: float = 0.1,
        gripper_state: float = -1.0,
        handedness: str = "right",
    ) -> bool:
        if not self.is_connected():
            return False

        # Jump-rejection: compare new TARGET against previous TARGET (not
        # mid-trajectory waypoint).  Catches tracking-loss teleports without
        # rejecting legitimate fast motion where the arm is still catching up.
        with self._goal_lock:
            prev_goal_pos = self._goal_pos
            prev_goal_ori = self._goal_ori

        pos_jump = 0.0
        rot_jump = 0.0
        if prev_goal_pos is not None and prev_goal_ori is not None:
            pos_jump = np.linalg.norm(position - prev_goal_pos)
            rot_jump = np.linalg.norm(
                _quat_to_euler_xyz(orientation) - _quat_to_euler_xyz(prev_goal_ori)
            )
            if pos_jump > self._MAX_POS_JUMP or rot_jump > self._MAX_ROT_JUMP:
                print(f"[RealmanBackend] JUMP REJECTED: "
                      f"pos={pos_jump*100:.1f}cm rot={np.degrees(rot_jump):.1f}deg")
                return True   # controller syncs to old goal; motion thread continues

        # Always update the goal.  The motion thread's min-change filter
        # (_MIN_POS_CHANGE / _MIN_ROT_CHANGE) suppresses redundant SDK calls
        # when the position hasn't actually changed, so we don't need a
        # coarse goal_moved gate here — and removing it ensures that the very
        # first clutch-press frame always registers.
        with self._goal_lock:
            self._goal_pos          = position.copy()
            self._goal_ori          = orientation.copy()
            self._goal_velocity     = velocity_limit
            self._goal_gripper      = gripper_state
            self._goal_last_updated = time.time()

        # actual_commanded tracks the goal so the controller's virtual position
        # stays in sync with what we're commanding.
        self.actual_commanded_position    = position.copy()
        self.actual_commanded_orientation = orientation.copy()
        self.last_update_time = time.time()
        self.command_count += 1
        return True

    def _maybe_send_gripper(self, state: float):
        """Send gripper command only when the target position changed.

        trigger 0.0 → rm_set_gripper_release (fully open)
        trigger 1.0 → rm_set_gripper_position(1000) (fully closed)
        trigger 0.0..1.0 → rm_set_gripper_position(1..1000)

        rm_set_gripper_release is used at trigger=0 because it drives the gripper
        to its physical maximum-open position regardless of the configured route,
        which is more reliable than position=1 for third-party grippers.
        """
        new_pos = int(np.clip(state, 0.0, 1.0) * _GRIPPER_CLOSED)
        new_pos = max(_GRIPPER_OPEN, new_pos)
        if new_pos == self._last_gripper_pos:
            return
        print(f"[RealmanBackend] Gripper: {self._last_gripper_pos} → {new_pos} (trigger={state:.3f})")
        self._last_gripper_pos = new_pos
        try:
            if state <= 0.02:
                # Full open: use release command for maximum-open regardless of route config
                with self._lock:
                    err = self._arm.rm_set_gripper_release(500, False, 5)
                cmd = "rm_set_gripper_release"
            else:
                with self._lock:
                    err = self._arm.rm_set_gripper_position(new_pos, False, 5)
                cmd = "rm_set_gripper_position"
            if err != 0:
                print(f"[RealmanBackend] {cmd} error: {err}")
            else:
                print(f"[RealmanBackend] Gripper OK: pos={new_pos}")
        except Exception as e:
            print(f"[RealmanBackend] Gripper command failed: {e}")

    # ------------------------------------------------------------------
    # State reading
    # ------------------------------------------------------------------

    def get_current_pose(self, **kwargs) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Returns cached pose — non-blocking, safe to call from async context."""
        with self._cache_lock:
            pos = self._cached_position
            ori = self._cached_orientation
        if pos is None or ori is None:
            return None, None
        return pos.copy(), ori.copy()

    def get_joint_positions(self) -> Optional[np.ndarray]:
        with self._cache_lock:
            return self._cached_joints.copy()

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def get_status(self) -> Dict[str, Any]:
        return {
            "backend": "realman_rm75b",
            "status": self.status.value,
            "host": self.host,
            "port": self.port,
            "command_count": self.command_count,
            "last_update": self.last_update_time,
            "ik_fail_count": self._ik_fail_count,
            "home_joints": self._home_joints_deg,
            "actual_position": (
                self.actual_commanded_position.tolist()
                if self.actual_commanded_position is not None else None
            ),
        }
