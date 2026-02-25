# VR + SO-101 Fixes: Camera Routing, Motion Accumulation, Safety Gate

## Overview

Three bugs were fixed that blocked practical VR teleoperation of the SO-101 arm with a Meta Quest 3:

1. **Camera streams — all three VR panels showed the same stitched composite**
2. **Motion accumulation — arm became faster and unsynchronised over time ("runaway")**
3. **Safety gate — could silently fail to activate on a flaky ADB tunnel**

Plus several smaller fixes discovered during bring-up:

4. Calibration file not found on connect (`SafeFeetechMotorsBus has no calibration registered`)
5. Camera at `/dev/video0` was skipped by the scan loop
6. Left/right hand movement was spatially inverted
7. Fallback starting pose required wrist_flex outside physical range
8. Y button ("go back to initial state") had no effect

---

## Fix 1 — Camera Routing (`server/backends/soarm_backend.py`)

### Problem

The server exposes three separate MJPEG endpoints:

```
GET /api/v1/video/left/mjpeg
GET /api/v1/video/right/mjpeg
GET /api/v1/video/depth/mjpeg
```

Each calls `backend.render(camera="left" | "right" | "depth")`. Because the old
`render()` did not accept a `camera` keyword argument, Python raised `TypeError`,
the server caught it, and fell back to `render()` — returning the same full stitched
composite for all three endpoints. The VR headset showed three identical panels.

### Fix

`render()` now accepts an optional `camera` parameter:

| Endpoint `camera=` | Internal role | Physical device  |
|--------------------|---------------|------------------|
| `"left"`           | `side_1`      | First detected (e.g. `/dev/video0`) |
| `"right"`          | `side_2`      | Second detected (e.g. `/dev/video2`) |
| `"depth"`          | `top`         | Third detected (e.g. `/dev/video4`) |
| `None` (default)   | —             | All three stitched side-by-side |

When `camera` is provided the method returns only that camera's frame, resized to
the requested `(width, height)`. When `camera=None` the original stitched behaviour
is preserved.

### Camera Reconnection

A new `_read_camera()` helper is used in all camera reads. When `cap.read()` returns
`ret=False` (device dropped), it immediately releases the handle and tries to reopen
`cv2.VideoCapture(index)`. If the device came back the stream resumes; if not, `None`
is returned and the VR panel goes black rather than freezing on the last frame.

Camera device indices are now stored alongside the capture object so the reconnect
knows which `/dev/videoN` to reopen.

---

## Fix 2 — Motion Accumulation / Integrator Windup (`server/backends/soarm_backend.py` + `server/teleop_server.py`)

### Problem

The teleoperation pipeline integrates incoming deltas into a *virtual* EE position
inside `TeleoperationController`. The SO-101 backend clamps each motor step to ≤ 45°
per command for safety, but the virtual position was updated with the *requested*
target — not the *clamped* result.

Effect: if the user held the clutch button and moved their hand continuously, the
virtual position raced far ahead of where the physical arm could go. In the session
logs this looked like:

```
shoulder_pan: physical = -116.4°,  virtual target = -293.8°  (177° behind)
```

When the arm finally reached an area where it could move freely again, it received a
huge pent-up delta command and shot across the workspace — feeling "runaway" and
unsynchronised with the operator's hand.

### Fix

**`SOARMBackend.send_target_pose()`** now computes forward kinematics (FK) from the
*actually commanded* joint angles (after clamping) and exposes them:

```python
self.actual_commanded_position    # FK of final clamped joint angles
self.actual_commanded_orientation
```

**`TeleoperationServer.process_command()`** reads these after every successful send
and resets the controller's virtual state:

```python
actual_pos = getattr(self.backend, 'actual_commanded_position', None)
if actual_pos is not None:
    self.controller.set_current_pose(actual_pos, actual_ori, command.handedness)
```

Result: the virtual position now tracks *what was actually commanded*, not *what was
requested*. When a step is clamped the virtual position advances at the same limited
rate as the physical arm — the gap never builds up, and no extra serial reads are
needed.

---

## Fix 3 — Safety Gate Retry (`client/web/static/teleop.js`)

### Problem

The `activateSafety()` call fires once, 500 ms after WebSocket connect, via a `fetch`
POST to `/api/v1/safety/activate`. If the ADB reverse tunnel hiccups at that exact
moment the fetch fails silently and `safetyActive` stays `false`, blocking all arm
commands.

### Fix

A retry interval runs every 2 s inside `startCommandLoop()`:

```js
this.safetyRetryInterval = setInterval(() => {
    if (!this.safetyActive) this.activateSafety();
}, 2000);
```

Cleared in `stopCommandLoop()`. The retry stops as soon as `activateSafety()` succeeds
(sets `this.safetyActive = true`).

---

## Fix 4 — Calibration File Not Found

### Problem

`SO101FollowerConfig` defaulted to `id=None`, so LeRobot looked for
`~/.cache/huggingface/lerobot/calibration/robots/so_follower/None.json` instead of
the actual file `deep_follower.json`.

### Fix

`SOARMBackend.__init__` now accepts `robot_id="deep_follower"` (default) and passes
`id=self.robot_id` to `SO101FollowerConfig`.

---

## Fix 5 — Camera Scan Skipped `/dev/video0`

### Problem

The scan loop started at index `1`, missing cameras at index `0`.

### Fix

`range(1, 10)` → `range(0, 10)`.

---

## Fix 6 — Left/Right Movement Inverted

### Problem

The VR-to-robot coordinate transform matrix had `Y_robot = -X_vr`, causing the arm to
move right when the operator moved left.

### Fix (`client/web/vr.html`)

```js
VR_TO_ROBOT = new THREE.Matrix4().set(
   0,  0, -1,  0,
   1,  0,  0,  0,   // was -1; flipped to match operator perspective
   0,  1,  0,  0,
   0,  0,  0,  1
);
```

---

## Fix 7 — Invalid Fallback Starting Pose

### Problem

The controller's fallback starting pose was `[±0.2, 0, 0.2]`. IK for that position
requires `wrist_flex ≈ −124°`, which is outside the SO-101 physical range (~−107° to
+65°). The arm would try to move to an unreachable pose on startup and the wrist motor
would appear inaccessible.

### Fix (`server/control_logic.py`)

Changed fallback to `[±0.15, 0, 0.38]` (requires `wrist_flex ≈ −52°`, well within range).

---

## Fix 8 — Y Button / Controller Resync

### Problem

The Y button was supposed to reset the arm to its initial state, but the handler was
never wired up in the VR render loop. The old `/api/v1/controller/reset` endpoint also
reset the virtual position to `[0, 0, 0]` (arm pointing straight up), which caused a
violent jump.

### Fix

- New endpoint `POST /api/v1/controller/resync` reads the robot's actual FK position and
  calls `controller.set_current_pose()` — snapping the virtual position to match reality
  **without moving the arm**.
- Left controller `buttons[5]` (Y) is now wired to call this endpoint.

---

## Joystick → Wrist Roll

The joystick X axis was previously unused. It is now mapped to `droll` (wrist roll):

| Input | Motion |
|-------|--------|
| Joystick X | Wrist roll (twist end-effector) |
| Joystick Y | Z axis (arm up/down) |
| Grip (hold) + hand movement | Full 6-DoF tracking |
| Trigger | Gripper open/close |
| Y button (left) | Resync virtual position to arm |
| B button (right, hold 1 s) | Exit VR |

---

## Other / Housekeeping

- `python` → `python3` throughout `README.md` and `QUICKSTART.md` (Linux default is Python 3)
- `pip install -r requirements.txt` → `pip install -r server/requirements.txt` (correct path)
- `send_target_pose()` signature updated with `gripper_state` and `handedness` parameters
  on all backends (`mock`, `mock_vr`, `isaac`, `mujoco`) for API consistency
