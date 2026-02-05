# Complete Local Validation Guide
## Dual SO-ARM101 + Quest 3S + Multi-Camera Setup

**Hardware Checklist:**
- ✅ 2x SO-ARM101 Robotic Arms
- ✅ Meta Quest 3S VR Headset
- ✅ Intel RealSense D435 Depth Camera
- ✅ 2x Webcameras
- ✅ LeRobot Validated

**Goal:** Run complete VR teleoperation system locally

---

## Validation Workflow

```
Step 1: Hardware Detection (5 min)
  ↓
Step 2: Single Arm Test (10 min)
  ↓
Step 3: Camera Integration (10 min)
  ↓
Step 4: Dual Arm Control (15 min)
  ↓
Step 5: VR Connection (10 min)
  ↓
Step 6: Full System Test (15 min)
```

**Total Time: ~60-70 minutes**

---

## Step 1: Hardware Detection and Verification (5 min)

### 1.1 Detect USB Devices

```bash
cd ~/Teleop_platform

# Run hardware detection script
python detect_hardware.py
```

Manual detection if script doesn't exist:

```bash
# 1. Detect robot arm USB ports
echo "=== Robot ARM USB Ports ==="
ls /dev/tty* | grep -E "USB|ACM"
# Expected: /dev/ttyUSB0, /dev/ttyUSB1 (or ttyACM0, ttyACM1)

# 2. Detect camera devices
echo "=== Camera Devices ==="
ls /dev/video*
# Expected: /dev/video0, /dev/video1, /dev/video2, etc.

# 3. Use lsusb for details
lsusb | grep -E "ARM|Real|Camera|Webcam"

# 4. Test camera availability
python3 -c "
import cv2
for i in range(10):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f'Camera {i}: Available')
        cap.release()
"
```

### 1.2 Setup USB Permissions

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Set camera permissions
sudo chmod 666 /dev/video*

# Set serial port permissions
sudo chmod 666 /dev/ttyUSB*  # or /dev/ttyACM*

# Re-login to apply permissions (or run)
newgrp dialout
```

### 1.3 Verify LeRobot Environment

```bash
# Check if LeRobot is available
python3 -c "
try:
    from lerobot.robots.so101_follower import SO101Follower
    print('✓ LeRobot imported successfully')
except ImportError as e:
    print('✗ LeRobot import failed:', e)
"
```

---

## Step 2: Single Arm Test (10 min)

### 2.1 Test First Arm

```bash
# Assuming first arm is on /dev/ttyUSB0
cd ~/Teleop_platform

# Modify test script port
nano test_soarm_integration.py
# Line 24: port='/dev/ttyUSB0'

# Run test
python test_soarm_integration.py
```

**Expected Output:**
```
🤖 SO-ARM Backend Integration Test

============================================================
Testing SO-ARM Backend Connection
============================================================

1. Creating SO-ARM backend...
✓ Backend created: so101_test

2. Connecting to robot...
[FlexibleSO101Follower] Scanning for motors on /dev/ttyUSB0...
  - Found shoulder_pan (ID 1)
  - Found shoulder_lift (ID 2)
  - Found elbow_flex (ID 3)
  - Found wrist_flex (ID 4)
  - Found wrist_roll (ID 5)
  - Found gripper (ID 6)
[FlexibleSO101Follower] Initializing with 6/6 motors
✓ Connection successful!

✅ ALL TESTS PASSED!
```

### 2.2 Start Single Arm Server

```bash
# Start server
./start_soarm_local.sh /dev/ttyUSB0

# Or manually
python run_server.py \
    --backend soarm \
    --soarm-port /dev/ttyUSB0 \
    --host 0.0.0.0 \
    --port 8000
```

### 2.3 Test Web Interface

```bash
# Open in browser
http://localhost:8000

# Test steps:
# 1. Click "Connect" button
# 2. Press '1' to activate safety gate
# 3. Use keyboard controls:
#    - W/S: Forward/Back
#    - A/D: Left/Right
#    - Q/E: Up/Down
#    - G/H: Gripper
# 4. Observe robot response
```

### 2.4 Stop Server

```bash
# Press Ctrl+C to stop server
```

---

## Step 3: Camera Integration Test (10 min)

### 3.1 Identify Camera Devices

```bash
# Run camera detection script
cd ~/Teleop_platform
python detect_cameras.py
```

**Expected Output:**
```
=== Camera Detection ===
Camera 0: Built-in Laptop Camera (640x480)
Camera 1: USB Webcam 1 (1920x1080)
Camera 2: USB Webcam 2 (1920x1080)
Camera 4: Intel RealSense D435 (1280x720)
```

### 3.2 Test RealSense D435

```bash
# Test RealSense
python test_realsense.py
```

If script doesn't exist:
```bash
python3 << 'EOF'
import pyrealsense2 as rs
import numpy as np

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

try:
    pipeline.start(config)
    print("✓ RealSense D435 working!")

    # Get test frame
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if color_frame:
        print(f"✓ Got frame: {color_frame.get_width()}x{color_frame.get_height()}")

    pipeline.stop()
except Exception as e:
    print(f"✗ RealSense error: {e}")
EOF
```

### 3.3 Test Webcams

```bash
python3 << 'EOF'
import cv2

# Test two webcams (skip 0 which is usually built-in)
for i in [1, 2]:
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            h, w = frame.shape[:2]
            print(f"✓ Camera {i}: {w}x{h}")
        cap.release()
    else:
        print(f"✗ Camera {i}: Not available")
EOF
```

### 3.4 Configure Camera Mapping

Create camera configuration file:
```bash
cat > camera_config.json << 'EOF'
{
  "cameras": {
    "left_hand": {
      "device": 1,
      "resolution": [640, 480],
      "fps": 30
    },
    "right_hand": {
      "device": 2,
      "resolution": [640, 480],
      "fps": 30
    },
    "depth": {
      "type": "realsense",
      "serial": null,
      "resolution": [640, 480],
      "fps": 30
    }
  }
}
EOF
```

---

## Step 4: Dual Arm Control Setup (15 min)

### 4.1 Identify Both Arms' USB Ports

```bash
# Check connected USB devices
ls -l /dev/ttyUSB* /dev/ttyACM*

# If you have two devices:
# /dev/ttyUSB0 -> Left arm
# /dev/ttyUSB1 -> Right arm
```

### 4.2 Create Dual Arm Configuration

**Option A: Modify code for dual arm support (Recommended)**
I can create a dual-arm backend implementation for you.

**Option B: Run two server instances (Temporary solution)**

```bash
# Terminal 1: Left arm server
python run_server.py \
    --backend soarm \
    --soarm-port /dev/ttyUSB0 \
    --host 0.0.0.0 \
    --port 8001

# Terminal 2: Right arm server
python run_server.py \
    --backend soarm \
    --soarm-port /dev/ttyUSB1 \
    --host 0.0.0.0 \
    --port 8002
```

**Note:** This requires VR client to connect to two WebSockets.

### 4.3 Test Dual Arm Independent Control

```bash
# Test left arm
curl http://localhost:8001/api/v1/status

# Test right arm
curl http://localhost:8002/api/v1/status
```

---

## Step 5: Quest 3S VR Connection (10 min)

### 5.1 Ensure Quest 3S and PC on Same Network

```bash
# Check your PC IP
hostname -I | awk '{print $1}'
# or
ip addr show | grep "inet " | grep -v 127.0.0.1
```

Note your IP address, e.g.: `192.168.1.100`

### 5.2 Start Server (with camera support)

```bash
# Start server
python run_server.py \
    --backend soarm \
    --soarm-port /dev/ttyUSB0 \
    --host 0.0.0.0 \
    --port 8000
```

### 5.3 Quest 3S Connection Steps

**Method A: Local WiFi (if on same network)**

1. Put on Quest 3S
2. Open Meta Quest Browser
3. Navigate to: `http://YOUR_PC_IP:8000`
   - e.g.: `http://192.168.1.100:8000`
4. Should see web interface

**Method B: Use ngrok (if WiFi blocked or need HTTPS)**

```bash
# Install ngrok
# Ubuntu/Debian:
curl -s https://ngrok-agent.s3.amazonaws.com/ngrok.asc | \
    sudo tee /etc/apt/trusted.gpg.d/ngrok.asc >/dev/null && \
    echo "deb https://ngrok-agent.s3.amazonaws.com buster main" | \
    sudo tee /etc/apt/sources.list.d/ngrok.list && \
    sudo apt update && sudo apt install ngrok

# Register and get authtoken (https://ngrok.com)
ngrok config add-authtoken <YOUR_TOKEN>

# Start ngrok tunnel
ngrok http 8000

# Note the HTTPS URL displayed, e.g.:
# https://a1b2-c3d4.ngrok-free.app
```

Access this HTTPS URL in Quest 3S browser.

### 5.4 Enter VR Mode

1. In Quest browser, you should see teleoperation interface
2. Click **"Enter VR Mode"** button (bottom right)
3. Allow VR permissions
4. Should now see VR view

---

## Step 6: Complete System Test (15 min)

### 6.1 Create Complete Startup Script

I'll create a one-click startup script for you.

### 6.2 System Checklist

After startup, verify:

- [ ] **Arm 1 Connected**: Check `/dev/ttyUSB0` responds
- [ ] **Arm 2 Connected**: Check `/dev/ttyUSB1` responds
- [ ] **Left Hand Camera**: Video stream visible
- [ ] **Right Hand Camera**: Video stream visible
- [ ] **Depth Camera**: D435 output normal
- [ ] **Web Interface**: Browser accessible
- [ ] **VR Mode**: Quest 3S can enter VR
- [ ] **Control Response**: Controller input effective
- [ ] **Latency**: <100ms (local network)

### 6.3 Complete Test Workflow

```bash
# 1. Start complete system
./start_complete_local.sh

# 2. Test 2D interface in browser
# http://localhost:8000

# 3. Test VR in Quest 3S
# http://YOUR_IP:8000/vr.html

# 4. Test control
#    - Move Quest controllers
#    - Press trigger (gripper)
#    - Observe robot response

# 5. Test video streams
#    - Should see 3 video streams in VR
#    - Left camera, right camera, depth camera

# 6. Performance test
#    - Feel the latency
#    - Check frame rate
#    - Confirm smoothness
```

---

## Troubleshooting

### Issue 1: USB Device Not Found

```bash
# Check USB connection
lsusb

# View system logs
dmesg | tail -20

# Replug USB and check again
ls /dev/tty*
```

### Issue 2: Permission Denied

```bash
# Quick temporary fix
sudo chmod 666 /dev/ttyUSB*
sudo chmod 666 /dev/video*

# Permanent fix
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER
# Re-login
```

### Issue 3: Camera Not Working

```bash
# Check if camera is in use
lsof /dev/video*

# Kill occupying process
sudo fuser -k /dev/video1

# Restart camera service
sudo systemctl restart guvcview  # if installed
```

### Issue 4: LeRobot Import Failed

```bash
# Check LeRobot installation
pip list | grep lerobot

# Reinstall if needed
cd ~/lerobot
pip install -e .
```

### Issue 5: Quest 3S Cannot Connect

```bash
# Method 1: Check firewall
sudo ufw status
sudo ufw allow 8000/tcp

# Method 2: Use ngrok
ngrok http 8000
# Then use HTTPS URL in Quest

# Method 3: Check network
ping YOUR_PC_IP  # from another device
```

### Issue 6: High Video Latency

```bash
# Lower resolution
# Edit camera_config.json
# Change to 320x240 or 640x480

# Lower frame rate
# Change to 15fps or 20fps

# Use local network instead of ngrok
# ngrok adds latency
```

---

## Performance Optimization

### Network Optimization
- ✅ Use 5GHz WiFi (not 2.4GHz)
- ✅ Quest and PC close to router
- ✅ Close bandwidth-heavy apps
- ✅ Local network better than ngrok

### Camera Optimization
- ✅ Use lower resolution (640x480)
- ✅ Use H.264 hardware encoding
- ✅ Limit frame rate (30fps sufficient)
- ✅ Use MJPEG instead of raw frames

### System Optimization
- ✅ Close unnecessary background apps
- ✅ Use SSD instead of HDD
- ✅ Ensure sufficient USB bandwidth
- ✅ Use USB 3.0 ports

---

## Success Criteria

Complete these checks for successful validation:

### Basic Functions ✅
- [ ] Both arms connect and control
- [ ] All cameras work properly
- [ ] Web interface accessible
- [ ] Quest 3S enters VR mode

### Performance Metrics ✅
- [ ] Local latency <100ms
- [ ] Video frame rate ≥20fps
- [ ] Control frequency ≥20Hz
- [ ] No noticeable lag

### User Experience ✅
- [ ] Clear video in VR
- [ ] Controller responds promptly
- [ ] Gripper functions properly
- [ ] Can operate 15+ minutes continuously

---

## Next Steps

After successful validation:

1. **Record Configuration**: Note all USB ports, camera device numbers
2. **Performance Data**: Record latency, frame rate metrics
3. **Create Scripts**: Save working config as startup scripts
4. **Prepare Deployment**: Use this config for AWS deployment
5. **Report to Manager**: Demo local validation results

---

## Need Help?

If you encounter any issues:

1. **Check Logs**:
   ```bash
   tail -f server.log
   ```

2. **Check System Status**:
   ```bash
   ./check_system_status.sh
   ```

3. **Run Diagnostics**:
   ```bash
   ./diagnose_issues.sh
   ```

4. **Reference Other Docs**:
   - `SOARM_QUICKSTART.md` - Single arm quick start
   - `VR_SETUP.md` - VR detailed setup
   - `QUEST3_SYNC_GUIDE.md` - Quest sync guide

---

**Ready? Let's start!** 🚀

Begin with **Step 1: Hardware Detection** and go step by step.
