# 🎮 Web UI Keyboard Controls Guide

## 📋 Complete Keyboard Mapping

### Current Implementation (As-Is)

The web UI currently supports these keyboard controls:

#### **Position Control (Cartesian Space)**
| Key | Action | Axis | Description |
|-----|--------|------|-------------|
| **W** | Move Forward | +Y | End-effector moves forward (away from you) |
| **S** | Move Backward | -Y | End-effector moves backward (toward you) |
| **A** | Move Left | -X | End-effector moves left |
| **D** | Move Right | +X | End-effector moves right |
| **Q** | Move Up | +Z | End-effector moves up |
| **E** | Move Down | -Z | End-effector moves down |

#### **Orientation Control (Rotation)**
| Key | Action | Axis | Description |
|-----|--------|------|-------------|
| **J** | Rotate Left | -Yaw | End-effector rotates counter-clockwise |
| **L** | Rotate Right | +Yaw | End-effector rotates clockwise |

#### **Alternative: Arrow Keys**
| Key | Action | Same as |
|-----|--------|---------|
| **↑** (Up Arrow) | Move Forward | W |
| **↓** (Down Arrow) | Move Backward | S |
| **←** (Left Arrow) | Move Left | A |
| **→** (Right Arrow) | Move Right | D |

### Control Settings

You can adjust control sensitivity in the web UI:
- **Speed (m per tick)**: Default 0.02m (adjustable in UI)
- **Yaw (rad per tick)**: Default 0.08 rad (adjustable in UI)
- **Update rate**: 50ms (20Hz)

---

## 🚀 How to Use Keyboard Controls

### Step 1: Login
1. Open web UI: `http://localhost:8080`
2. Login with: **operator** / **operator**
3. Controls panel will appear

### Step 2: Start Controlling
1. **Click anywhere on the page** to ensure keyboard focus
2. **Hold down keys** to move continuously
3. **Release keys** to stop movement
4. Watch the "keys: " indicator to see active keys

### Step 3: Adjust Speed (Optional)
- Modify "Speed (m per tick)" for faster/slower linear movement
- Modify "Yaw (rad per tick)" for faster/slower rotation
- Changes apply immediately

---

## 🎯 Control Modes

### How It Works

**Concept:** The web UI sends **delta commands** (incremental movements) to the server:
- Each key press generates a small position change (dx, dy, dz)
- Commands are sent at 20Hz (50ms intervals) while keys are held
- The robot smoothly integrates these small movements

**Example:**
```
Press W → sends dy=0.02 every 50ms
Hold for 1 second → robot moves 0.02m × 20 = 0.4m forward
```

### Position Control (WASD/Arrows + QE)
- **What it does:** Moves the end-effector in 3D space
- **Coordinate frame:** World frame (fixed)
- **Use case:** Positioning the gripper at target locations

**Example Movements:**
```
W (1 sec) → End-effector at Y+0.4m
D (0.5 sec) → End-effector at X+0.2m, Y+0.4m
Q (0.25 sec) → End-effector at X+0.2m, Y+0.4m, Z+0.1m
```

### Orientation Control (JL)
- **What it does:** Rotates the end-effector around Z-axis (yaw)
- **Coordinate frame:** World frame
- **Use case:** Orienting the gripper for grasping

**Example:**
```
L (1 sec) → End-effector rotates +1.6 rad (~92°) clockwise
J (0.5 sec) → End-effector rotates -0.8 rad (~46°) counter-clockwise
```

---

## 🔘 Button Controls (Alternative to Keyboard)

Don't like keyboard? Use the on-screen buttons:

| Button | Same as Key | Action |
|--------|-------------|--------|
| **Forward** | W | Move +Y |
| **Back** | S | Move -Y |
| **Left** | A | Move -X |
| **Right** | D | Move +X |
| **Up** | Q | Move +Z |
| **Down** | E | Move -Z |
| **Yaw-** | J | Rotate counter-clockwise |
| **Yaw+** | L | Rotate clockwise |

**How to use:**
- Click and hold button to move
- Release to stop
- Works same as keyboard

---

## ⚠️ Safety Features

### Stop Button
- **Action:** Sends zero velocity command
- **Use when:** You want to pause movement
- **Effect:** Robot stops immediately but stays connected

### E-Stop Button
- **Action:** Emergency stop + disconnects command stream
- **Use when:** Emergency situation
- **Effect:** Robot stops and session ends (must reconnect)

### Safety Gate
The system has automatic safety features:
- **Workspace limits:** Robot cannot move outside safe boundaries
- **Velocity limits:** Maximum speed capped (0.25 m/s default)
- **Deadman switch:** Commands must be continuous (if stopped for 0.5s, robot stops)

---

## 🎮 Advanced Usage Tips

### Combining Keys
You can press multiple keys simultaneously:

**Diagonal Movement:**
```
W + D → Move forward-right (45° angle)
Q + W → Move up and forward simultaneously
```

**Complex Motion:**
```
W + D + Q → Move forward-right-up (3D diagonal)
A + E → Move left and down
```

### Fine Control
For precise movements:
1. Reduce speed to 0.005m or lower
2. Tap keys briefly instead of holding
3. Watch real-time position feedback

### Speed Presets
Quick speed adjustments:
- **Slow/Precise:** 0.005 m/tick, 0.02 rad/tick
- **Normal:** 0.02 m/tick, 0.08 rad/tick
- **Fast:** 0.05 m/tick, 0.15 rad/tick

---

## 🐛 Troubleshooting

### "Keys not working"
**Cause:** Page doesn't have keyboard focus
**Solution:**
1. Click anywhere on the page
2. Check "keys: " indicator shows your keypresses
3. Ensure you're logged in (controls panel visible)

### "Robot moves too fast/slow"
**Solution:**
- Adjust "Speed (m per tick)" slider
- Lower values = slower, more precise
- Higher values = faster movement

### "Robot doesn't move at all"
**Checklist:**
1. ✅ Server running? Check connection status (top-right)
2. ✅ Logged in? Controls panel should be visible
3. ✅ Backend connected? Check "backend connected=true" in stats
4. ✅ Safety active? (This may be required depending on configuration)

### "Keys show in indicator but robot doesn't move"
**Possible causes:**
1. Backend not connected (check server terminal)
2. Robot at workspace boundary (trying to move outside limits)
3. WebSocket disconnected (check browser console F12)

---

## 📊 Real-Time Feedback

While controlling, watch these indicators:

### Keys Indicator
```
keys: w d        ← Shows which keys are currently pressed
```

### Position Display
```
Position: [0.123, 0.456, 0.789]  ← Current end-effector position (meters)
Orientation: [1.000, 0.000, 0.000, 0.000]  ← Quaternion [w,x,y,z]
```

### Status Indicators
- **Connected** (green) → WebSocket active
- **Safety Active** (green) → Commands being accepted
- **Backend** → Shows which backend (mock/mujoco/isaac)

### Command Log
Check the command log at bottom for:
```
12:34:56.789 ack: success
12:34:56.840 ack: success
```

---

## 🎯 Example Workflows

### Example 1: Move to Position
**Goal:** Move end-effector to position [0.3, 0.2, 0.5]

**Steps:**
1. Login to web UI
2. Note current position (e.g., [0.0, 0.0, 0.5])
3. Press **D** for ~7.5 seconds (0.02 × 150 ticks = 3.0m... but will hit limits)
4. Press **W** for ~5 seconds
5. Check position display matches target

### Example 2: Rotate and Grab
**Goal:** Rotate gripper 90° and move down

**Steps:**
1. Press **L** for ~1.2 seconds (0.08 × 24 ≈ 1.92 rad ≈ 110°)
2. Adjust if needed
3. Press **E** to move down to object
4. (Gripper control not yet implemented in web UI)

### Example 3: Circular Motion
**Goal:** Move end-effector in a circle

**Steps:**
1. Reduce speed to 0.01 m/tick
2. Sequence: W (2s) → D (2s) → S (2s) → A (2s)
3. Repeat to complete circle
4. Or use diagonal combinations for smoother motion

---

## 🔮 What's Missing (Not Yet Implemented)

Currently **NOT available** in the web UI:

### Roll/Pitch Control
- ❌ Roll rotation (rotation around X-axis)
- ❌ Pitch rotation (rotation around Y-axis)
- ✅ **Only Yaw** (J/L keys) is implemented

**Workaround:** Use keyboard client which has full orientation control:
```bash
python client/keyboard_client.py
```

### Gripper Control
- ❌ Open/close gripper
- ❌ Gripper force control

**Status:** Backend supports gripper, web UI needs implementation

### Joint Control Mode
- ❌ Direct joint angle control
- ❌ Joint velocity control
- ✅ **Only Cartesian** (XYZ position) is available

---

## 🔧 How to Add More Controls

Want to add Roll/Pitch or Gripper? Here's how:

### Quick Fix: Add Roll/Pitch Keys

Edit `/Users/ziguo/teleop_system/client/web/index.html`:

Find the `startStreaming()` function and add:
```javascript
// Around line 260
if (k.has("u")) droll -= yaw    // Roll left
if (k.has("o")) droll += yaw    // Roll right
if (k.has("i")) dpitch += yaw   // Pitch up
if (k.has("k")) dpitch -= yaw   // Pitch down

// Update sendDelta call:
if (dx || dy || dz || droll || dpitch || dyaw) 
    sendDelta(dx, dy, dz, dyaw)
```

Change to:
```javascript
sendDelta(dx, dy, dz, droll, dpitch, dyaw)
```

But you also need to update the `sendDelta()` function to accept these parameters!

---

## 📖 Full Keyboard Reference Card

```
╔═══════════════════════════════════════════════════════════╗
║           WEB UI KEYBOARD CONTROLS REFERENCE              ║
╠═══════════════════════════════════════════════════════════╣
║                                                           ║
║  POSITION CONTROL (End-Effector Movement)                ║
║  ───────────────────────────────────────                 ║
║    W / ↑     Move Forward    (+Y axis)                   ║
║    S / ↓     Move Backward   (-Y axis)                   ║
║    A / ←     Move Left       (-X axis)                   ║
║    D / →     Move Right      (+X axis)                   ║
║    Q         Move Up         (+Z axis)                   ║
║    E         Move Down       (-Z axis)                   ║
║                                                           ║
║  ORIENTATION CONTROL (End-Effector Rotation)             ║
║  ──────────────────────────────────────────              ║
║    J         Yaw Left        (Counter-clockwise)         ║
║    L         Yaw Right       (Clockwise)                 ║
║                                                           ║
║  SETTINGS                                                ║
║  ────────                                                ║
║    Speed:     0.02 m/tick    (adjustable in UI)          ║
║    Yaw:       0.08 rad/tick  (adjustable in UI)          ║
║    Rate:      20 Hz          (50ms updates)              ║
║                                                           ║
║  NOTES                                                   ║
║  ─────                                                   ║
║    • Hold keys to move continuously                      ║
║    • Release to stop                                     ║
║    • Combine keys for diagonal movement                  ║
║    • Check "keys:" indicator for active keys             ║
║                                                           ║
╚═══════════════════════════════════════════════════════════╝
```

---

**Summary:** The web UI keyboard controls work great for **position control (WASD/QE)** and **basic yaw rotation (JL)**. For full 6-DOF control (Roll/Pitch) or gripper control, you'll need to either:
1. Use the Python keyboard client: `python client/keyboard_client.py`
2. Modify the web UI to add more keys (see "How to Add More Controls" above)
3. Use the on-screen buttons for basic movements

Let me know if you'd like me to help you add Roll/Pitch/Gripper controls to the web UI! 🎮
