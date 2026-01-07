# 🎮 Complete Keyboard Control Guide - Web UI vs Python Client

## 📊 Quick Comparison

| Feature | Web UI | Python Client |
|---------|--------|---------------|
| **Position Control (XYZ)** | ✅ W/A/S/D/Q/E | ✅ W/A/S/D/Q/E |
| **Yaw Rotation** | ✅ J/L | ✅ J/L |
| **Roll Rotation** | ❌ Not implemented | ✅ U/O |
| **Pitch Rotation** | ❌ Not implemented | ✅ I/K |
| **Mode Toggle** | ❌ Always Position | ✅ M key (Position/Orientation) |
| **Safety Activation** | ⚠️ Manual in UI | ✅ Auto + 1 key |
| **Control Rate** | 20 Hz (50ms) | 20 Hz (50ms) |
| **Easy to Use** | ✅ Browser-based | ⚠️ Terminal-based |

---

## 🌐 WEB UI CONTROLS (Current Implementation)

### How to Use
```bash
# Terminal 1: Start server
python run_server.py --backend mujoco --mujoco-xml robots/so101.xml --mujoco-ee gripperframe

# Terminal 2: Start web UI
python client/web_server.py

# Browser: Open http://localhost:8080
# Login: operator / operator
```

### Keyboard Mapping

#### Position Control (Always Active)
```
       W (↑)
        │
    A ←─┼─→ D
        │
       S (↓)

Q = Up (+Z)
E = Down (-Z)
```

**Keys:**
- **W** / **↑** → Move Forward (+Y)
- **S** / **↓** → Move Backward (-Y)
- **A** / **←** → Move Left (-X)
- **D** / **→** → Move Right (+X)
- **Q** → Move Up (+Z)
- **E** → Move Down (-Z)

#### Rotation Control (Limited)
```
J ← Yaw → L
```

**Keys:**
- **J** → Rotate Left (counter-clockwise)
- **L** → Rotate Right (clockwise)

### Settings (Adjustable in UI)
- **Speed:** 0.02 m/tick (default)
- **Yaw:** 0.08 rad/tick (default)

### Usage Example
```javascript
1. Login to web UI
2. Click anywhere on page (for keyboard focus)
3. Hold W for 2 seconds → Robot moves forward ~0.8m
4. Hold D for 1 second → Robot moves right ~0.4m
5. Hold L for 0.5 second → Robot rotates ~0.8 rad (46°)
6. Release all keys → Robot stops
```

### What You See
- **Keys indicator:** Shows active keys (e.g., "keys: w d")
- **Position:** Real-time [X, Y, Z] coordinates
- **Orientation:** Quaternion [w, x, y, z]
- **Command log:** Acknowledgements from server

---

## 🐍 PYTHON CLIENT CONTROLS (Full 6-DOF)

### How to Use
```bash
# Terminal 1: Start server
python run_server.py --backend mujoco --mujoco-xml robots/so101.xml --mujoco-ee gripperframe

# Terminal 2: Run Python client
python client/keyboard_client.py
```

### Keyboard Mapping

#### Position Control Mode (Default)
```
       W
        │
    A ←─┼─→ D
        │
       S

Q = Forward (+Y)
E = Backward (-Y)
```

**Keys:**
- **W** → Move Up (+Z)
- **S** → Move Down (-Z)
- **A** → Move Left (-X)
- **D** → Move Right (+X)
- **Q** → Move Forward (+Y)
- **E** → Move Backward (-Y)

#### Orientation Control Mode (Press M to activate)
```
    I = Pitch Up
        │
J ← Yaw → L
        │
    K = Pitch Down

U = Roll Left
O = Roll Right
```

**Keys:**
- **I** → Pitch Up (nose up)
- **K** → Pitch Down (nose down)
- **J** → Yaw Left (turn left)
- **L** → Yaw Right (turn right)
- **U** → Roll Left (rotate left)
- **O** → Roll Right (rotate right)

#### Mode Control
- **M** → Toggle between Position / Orientation mode
- **R** → Reset all commands to zero
- **1** → Activate safety gate
- **H** → Show help
- **Ctrl+C** → Exit

### Settings (Hardcoded)
- **Position increment:** 0.02 m
- **Orientation increment:** 0.05 rad
- **Send rate:** 20 Hz

### Usage Example
```bash
1. Run: python client/keyboard_client.py
2. Automatic safety activation
3. Default mode: POSITION
4. Press W for 2 sec → Robot moves up ~0.8m
5. Press M → Switch to ORIENTATION mode
6. Press I for 1 sec → Robot pitches up ~1.0 rad (57°)
7. Press M → Switch back to POSITION mode
8. Press Ctrl+C → Exit
```

### What You See (Terminal Output)
```
✓ Connected to server
Sender loop started at 20.0 Hz

Control Mode: POSITION

[When you press keys, violations may appear:]
⚠️  WORKSPACE VIOLATION!

[Otherwise, minimal output for clean terminal]
```

---

## 🎯 Which One Should You Use?

### Use Web UI When:
✅ You want a visual interface
✅ You need to see video feed (when implemented)
✅ You're demoing to non-technical users
✅ You want buttons instead of keyboard
✅ Basic XYZ movement + Yaw is enough

### Use Python Client When:
✅ You need full 6-DOF control (Roll/Pitch)
✅ You prefer terminal-based control
✅ You're doing precise orientation adjustments
✅ You want to toggle between position/orientation modes
✅ You're comfortable with keyboard shortcuts

---

## 🔄 Side-by-Side Control Comparison

### Moving Forward 1 Meter

**Web UI:**
```
1. Hold W for 50 seconds (0.02m × 20Hz × 50s = 20m... oops!)
Actually: Hold W for 2.5 seconds (0.02m × 20Hz × 2.5s = 1.0m)
```

**Python Client:**
```
1. Press Q for 2.5 seconds (0.02m × 20Hz × 2.5s = 1.0m)
```

### Rotating 90 Degrees (π/2 ≈ 1.57 rad)

**Web UI:**
```
1. Adjust Yaw to 0.08 rad/tick
2. Hold L for ~1.0 second (0.08 × 20 ≈ 1.6 rad ≈ 92°)
```

**Python Client:**
```
1. Press M to switch to ORIENTATION mode
2. Hold L for ~1.6 seconds (0.05 × 20 × 1.6 ≈ 1.6 rad)
3. Press M to switch back to POSITION mode
```

### Complex Motion: Move Forward-Right-Up

**Web UI:**
```
1. Hold W + D + Q simultaneously
2. Robot moves diagonally in 3D space
```

**Python Client:**
```
1. Ensure in POSITION mode
2. Hold Q + D + W simultaneously
3. Robot moves diagonally in 3D space
```

---

## 🛠️ Advanced Tips

### Web UI Pro Tips

**1. Adjust Speed for Precision**
```
For fine control:
- Set Speed to 0.005 m/tick
- Tap keys briefly instead of holding

For fast movement:
- Set Speed to 0.05 m/tick
- Hold keys longer
```

**2. Use On-Screen Buttons**
```
Don't like keyboard?
- Click and hold movement buttons
- Works on touch devices (tablets/phones)
```

**3. Monitor Real-Time Feedback**
```
Watch:
- "keys:" indicator shows active keys
- Position updates in real-time
- Command log shows acknowledgements
```

### Python Client Pro Tips

**1. Mode Management**
```
Pattern for complex maneuvers:
1. Position mode → Move to location
2. Press M → Switch to orientation
3. Adjust orientation (I/K/J/L/U/O)
4. Press M → Switch back to position
5. Continue positioning
```

**2. Emergency Reset**
```
If robot behaving strangely:
- Press R to reset all deltas to zero
- Press 1 to reactivate safety
- Resume control
```

**3. Fine Control**
```
The Python client sends small increments:
- Position: 0.02m per keypress
- Orientation: 0.05 rad per keypress
- At 20Hz, this gives smooth motion
```

---

## 🚀 Future Enhancements (Not Yet Implemented)

### Web UI Roadmap
- [ ] Add Roll/Pitch controls (I/K/U/O keys)
- [ ] Add mode toggle button
- [ ] Add gripper control
- [ ] Add joint control mode
- [ ] Add 3D visualization (Three.js)
- [ ] Add touch gestures for mobile

### Python Client Roadmap
- [ ] Add gripper control
- [ ] Add joint control mode
- [ ] Add velocity control mode
- [ ] Add trajectory playback
- [ ] Add command recording

---

## 📝 Quick Reference Cards

### Web UI Quick Reference
```
╔═══════════════════════════════════════╗
║    WEB UI KEYBOARD CONTROLS           ║
╠═══════════════════════════════════════╣
║  Position (XYZ):                      ║
║    W/↑  → Forward  (+Y)               ║
║    S/↓  → Back     (-Y)               ║
║    A/←  → Left     (-X)               ║
║    D/→  → Right    (+X)               ║
║    Q    → Up       (+Z)               ║
║    E    → Down     (-Z)               ║
║                                       ║
║  Rotation (Yaw only):                 ║
║    J    → Rotate Left                 ║
║    L    → Rotate Right                ║
║                                       ║
║  Settings:                            ║
║    Speed: 0.02 m/tick (adjustable)    ║
║    Yaw:   0.08 rad/tick (adjustable)  ║
╚═══════════════════════════════════════╝
```

### Python Client Quick Reference
```
╔════════════════════════════════════════╗
║   PYTHON CLIENT KEYBOARD CONTROLS      ║
╠════════════════════════════════════════╣
║  Position Mode (default):              ║
║    W → Up      (+Z)                    ║
║    S → Down    (-Z)                    ║
║    A → Left    (-X)                    ║
║    D → Right   (+X)                    ║
║    Q → Forward (+Y)                    ║
║    E → Back    (-Y)                    ║
║                                        ║
║  Orientation Mode (press M):           ║
║    I → Pitch Up                        ║
║    K → Pitch Down                      ║
║    J → Yaw Left                        ║
║    L → Yaw Right                       ║
║    U → Roll Left                       ║
║    O → Roll Right                      ║
║                                        ║
║  Controls:                             ║
║    M → Toggle Position/Orientation     ║
║    R → Reset to zero                   ║
║    1 → Activate safety                 ║
║    H → Help                            ║
║                                        ║
║  Settings:                             ║
║    Position: 0.02 m/press              ║
║    Rotation: 0.05 rad/press            ║
║    Rate:     20 Hz                     ║
╚════════════════════════════════════════╝
```

---

## 🎬 Tutorial: First Time Using Controls

### Web UI Tutorial (5 minutes)

```bash
# 1. Start everything
cd ~/teleop_system
python run_server.py --backend mock &  # Start in background
python client/web_server.py

# 2. Open browser
open http://localhost:8080

# 3. Login
Username: operator
Password: operator

# 4. Try controls
Click on page → Press W → Robot moves!

# 5. Adjust speed
Set "Speed" to 0.01 for slower movement

# 6. Try combinations
Hold W + D → Robot moves diagonally
```

### Python Client Tutorial (5 minutes)

```bash
# 1. Start server
cd ~/teleop_system
python run_server.py --backend mock

# 2. In new terminal, start client
python client/keyboard_client.py

# 3. You'll see:
✓ Connected to server
Activating safety gate...
✓ Safety gate manually activated

# 4. Try position control (default mode)
Press W → Robot moves up
Press Q → Robot moves forward

# 5. Switch to orientation
Press M → "Switched to ORIENTATION control mode"
Press I → Robot pitches up
Press J → Robot yaws left

# 6. Switch back
Press M → "Switched to POSITION control mode"

# 7. Exit
Press Ctrl+C
```

---

**Summary:**
- **Web UI:** Great for demos, visual feedback, basic XYZ + Yaw control
- **Python Client:** Full 6-DOF control, mode switching, better for advanced users
- **Both:** Work at 20Hz, send delta commands, support same backend

**Recommendation:** Start with Web UI to learn the basics, then graduate to Python client when you need full Roll/Pitch control! 🚀
