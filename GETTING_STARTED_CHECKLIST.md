# RM75B VR Control - Getting Started Checklist

## 📋 Pre-Flight Checklist

Complete these steps in order to get VR control working.

---

## Phase 1: Environment Setup (5 minutes)

### ✅ Software Installation

- [ ] **Python 3.10+** installed
  ```bash
  python --version  # Should show 3.10 or higher
  ```

- [ ] **MuJoCo** installed
  ```bash
  pip install mujoco
  python -c "import mujoco; print(mujoco.__version__)"
  ```

- [ ] **Required packages** installed
  ```bash
  pip install numpy flask transforms3d pillow
  ```

- [ ] **ADB tools** installed (for USB tethering)
  ```bash
  # macOS
  brew install android-platform-tools
  
  # Linux
  sudo apt install adb
  
  # Test
  adb version
  ```

### ✅ Project Files

- [ ] **Navigate to project directory**
  ```bash
  cd ~/teleop_system
  pwd  # Should show: /Users/your-username/teleop_system
  ```

- [ ] **Verify new files exist**
  ```bash
  ls -lh robots/rm75b_vr.xml
  ls -lh test_rm75b_setup.py
  ls -lh RM75B_VR_SETUP.md
  ```

- [ ] **Make scripts executable** (if using bash scripts)
  ```bash
  chmod +x start_rm75b_vr.sh
  chmod +x commands_reference.sh
  ```

---

## Phase 2: System Verification (3 minutes)

### ✅ Run Verification Script

- [ ] **Execute test script**
  ```bash
  python test_rm75b_setup.py
  ```

- [ ] **All tests should pass**
  ```
  ✅ PASS - MuJoCo Installation
  ✅ PASS - Model Loading
  ✅ PASS - End Effector Site
  ✅ PASS - IK Solver
  ✅ PASS - VR Cameras
  ✅ PASS - Server Prerequisites
  ```

- [ ] **If any test fails**, fix before proceeding
  - Read error message
  - Check installation
  - See troubleshooting section below

### ✅ Visual Verification

- [ ] **View robot in MuJoCo viewer**
  ```bash
  python -m mujoco.viewer robots/rm75b_vr.xml
  ```
  
  - [ ] Robot model appears
  - [ ] Can rotate view with mouse
  - [ ] Joints move when dragging sliders
  - [ ] No error messages

---

## Phase 3: Network Setup (2 minutes)

### ✅ Find Your IP Address

- [ ] **Get local IP**
  ```bash
  # macOS
  ifconfig | grep "inet " | grep -v 127.0.0.1 | awk '{print $2}'
  
  # Linux
  hostname -I | awk '{print $1}'
  ```

- [ ] **Write down IP address**: `_____._____._____._____ `
  Example: `192.168.1.100`

### ✅ Check Firewall

- [ ] **macOS**: System Settings → Network → Firewall
  - Option A: Turn OFF (temporarily)
  - Option B: Add Python to allowed apps

- [ ] **Linux**: Check UFW status
  ```bash
  sudo ufw status
  # If active, allow port 8000
  sudo ufw allow 8000/tcp
  ```

---

## Phase 4: Server Start (2 minutes)

### ✅ Start the Server

- [ ] **Run server command**
  ```bash
  python run_server.py \
      --backend mujoco \
      --mujoco-xml robots/rm75b_vr.xml \
      --mujoco-ee ee_site \
      --port 8000
  ```

- [ ] **Verify server output shows**:
  ```
  ✅ MuJoCo Backend Initialized
  ✅ Model: realman_rm75b
  ✅ End Effector: ee_site
  ✅ DOF: 7 joints + 2 gripper
  🌐 Server running on http://0.0.0.0:8000
  ```

### ✅ Test Server Health

- [ ] **In new terminal, test API**
  ```bash
  curl http://localhost:8000/api/v1/statistics | python -m json.tool
  ```

- [ ] **Should see JSON response with**:
  ```json
  {
    "backend": "mujoco",
    "status": "connected",
    "model": "realman_rm75b"
  }
  ```

- [ ] **Test web interface**
  ```bash
  open http://localhost:8000/web/
  # Or manually navigate in browser
  ```

---

## Phase 5: Quest 3S Setup (One-time, 5 minutes)

### ✅ Developer Mode Activation

- [ ] **Install Meta Horizon app** on your phone

- [ ] **Pair Quest with phone**
  - Open app
  - Menu → Devices → Add Headset
  - Follow pairing instructions

- [ ] **Enable Developer Mode**
  - In app: Menu → Devices → Headset Settings
  - Developer Mode → ON
  - Confirm on both phone and headset

- [ ] **Connect Quest to Mac via USB**

- [ ] **Allow USB Debugging**
  - Put on Quest headset
  - Dialog appears: "Allow USB debugging?"
  - Check "Always allow from this computer"
  - Click "Allow"

### ✅ Verify ADB Connection

- [ ] **Check Quest is detected**
  ```bash
  adb devices
  ```
  
- [ ] **Should show**:
  ```
  List of devices attached
  1WMHH123456789  device
  ```

- [ ] **If "unauthorized"**:
  - Put on Quest
  - Check for USB debugging dialog
  - Click Allow

---

## Phase 6: VR Connection (Choose One)

### Option A: WiFi Connection ✅ Easier

- [ ] **Quest and Mac on same WiFi**
  - Quest: Settings → WiFi → Connected to `___________`
  - Mac: Same network

- [ ] **Put on Quest headset**

- [ ] **Open Browser app**
  - Press Oculus button
  - Select "Browser" from menu

- [ ] **Navigate to server**
  - Type in address bar: `http://YOUR_IP:8000/web/`
  - Example: `http://192.168.1.100:8000/web/`

- [ ] **Login**
  - Username: `operator`
  - Password: `operator`
  - Click "Connect to Server"

### Option B: USB Tethering ✅ Lower Latency (Recommended)

- [ ] **Quest connected via USB**

- [ ] **Setup port forwarding**
  ```bash
  adb reverse tcp:8000 tcp:8000
  ```

- [ ] **Verify forwarding**
  ```bash
  adb reverse --list
  # Should show: tcp:8000 -> tcp:8000
  ```

- [ ] **In Quest Browser, navigate to**
  ```
  http://localhost:8000/web/
  ```

- [ ] **Login** (same as WiFi option)

---

## Phase 7: Enter VR Mode (1 minute)

### ✅ Activate VR

- [ ] **In Browser, click button**
  - Look for "🥽 Enter VR Mode" button
  - Click it

- [ ] **Grant VR permissions**
  - Dialog: "Allow VR access?"
  - Click "Allow"

- [ ] **You should see**:
  - Stereoscopic view (separate images for each eye)
  - MuJoCo simulation environment
  - Robot arm visible
  - Grid floor for reference

- [ ] **Controllers appear**
  - Move Quest controllers
  - They should appear as 3D objects in VR

---

## Phase 8: Test Controls (2 minutes)

### ✅ Basic Movement

- [ ] **Move right controller slowly**
  - Robot end effector should follow
  - Movement should be smooth (no jitter)

- [ ] **Try each axis**:
  - [ ] Left/Right (X axis)
  - [ ] Forward/Back (Y axis)
  - [ ] Up/Down (Z axis)

### ✅ Rotation

- [ ] **Rotate right controller**
  - Robot end effector should rotate
  - Orientation should match controller

### ✅ Gripper Control

- [ ] **Squeeze right trigger**
  - Gripper should close
  - Smooth motion from open → closed

- [ ] **Release trigger**
  - Gripper should open

### ✅ Emergency Stop

- [ ] **Press B button** (right controller)
  - Robot should stop immediately
  - Can resume by moving controller

---

## Phase 9: Performance Check (1 minute)

### ✅ Latency Test

- [ ] **Move controller quickly**
  - Robot should follow with minimal delay
  - Target: <50ms latency
  - Should feel "responsive"

- [ ] **No motion sickness**
  - If feeling sick: latency too high
  - Try USB tethering instead of WiFi
  - Or reduce IK iterations

### ✅ Smoothness Test

- [ ] **Move in circles**
  - Motion should be fluid
  - No jumps or stutters

- [ ] **30-second session**
  - Comfortable to use
  - No crashes
  - Stable connection

---

## 🎉 Success Criteria

You're ready to use the system when:

✅ All Phase 2 tests pass  
✅ Server starts without errors  
✅ Quest connects (WiFi or USB)  
✅ VR mode activates  
✅ Robot follows controller smoothly  
✅ Gripper responds to trigger  
✅ Latency feels good (<50ms)  
✅ No crashes in 30-second test  

---

## ❌ Troubleshooting Quick Reference

### Server won't start
```bash
# Check if port is in use
lsof -i :8000

# Kill conflicting process
lsof -ti :8000 | xargs kill -9

# Try again
python run_server.py --backend mujoco --mujoco-xml robots/rm75b_vr.xml --mujoco-ee ee_site
```

### Quest can't connect (WiFi)
```bash
# Verify same network
# Quest: Settings → WiFi
# Mac: System Settings → WiFi

# Check firewall
# System Settings → Network → Firewall → Off

# Verify IP is correct
ifconfig | grep "inet "
```

### Quest can't connect (USB)
```bash
# Check USB connection
adb devices

# If unauthorized: allow on Quest

# Re-do port forwarding
adb reverse --remove-all
adb reverse tcp:8000 tcp:8000
```

### VR mode button missing
```bash
# Update Quest firmware
# Settings → System → Software Update

# Try different browser
# Use "Meta Quest Browser" (built-in)

# Check server is running
curl http://localhost:8000/api/v1/statistics
```

### Robot moves too fast
```bash
# Restart server with slower motion
python run_server.py \
    --backend mujoco \
    --mujoco-xml robots/rm75b_vr.xml \
    --mujoco-ee ee_site \
    --max-qpos-step 0.05
```

### High latency / choppy
```bash
# Switch to USB tethering (fastest)
# See Option B above

# Or reduce IK quality (faster but less accurate)
python run_server.py \
    --backend mujoco \
    --mujoco-xml robots/rm75b_vr.xml \
    --mujoco-ee ee_site \
    --ik-max-iters 15
```

---

## 📞 Need More Help?

1. **Re-read documentation**:
   - `cat RM75B_QUICK_START.md`
   - `cat RM75B_VR_SETUP.md`

2. **Check logs**:
   ```bash
   python run_server.py ... 2>&1 | tee server.log
   cat server.log
   ```

3. **Run verification again**:
   ```bash
   python test_rm75b_setup.py
   ```

4. **Check system resources**:
   ```bash
   top -pid $(pgrep -f "run_server.py")
   ```

---

## 🎯 Next Steps After Setup

Once everything works:

1. **Calibrate motion scaling**
   - Adjust sensitivity in VR settings
   - Fine-tune IK parameters

2. **Practice control**
   - Pick and place objects
   - Draw shapes in air
   - Navigate workspace

3. **Record demonstrations**
   - Start recording
   - Perform task
   - Save trajectory

4. **Explore advanced features**
   - Custom keyframes
   - Multi-robot control
   - Collision avoidance

---

**Date Completed**: ____ / ____ / ____

**Tested By**: __________________

**System Status**: ✅ Working / ⚠️ Issues / ❌ Not Working

**Notes**:
```
_____________________________________________
_____________________________________________
_____________________________________________
```
