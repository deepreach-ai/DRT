#!/bin/bash
# RM75B VR Control - Command Reference
# Copy and paste these commands to get started quickly

# ============================================================================
# SETUP & VERIFICATION
# ============================================================================

# 1. Verify all prerequisites
cd ~/teleop_system
python test_rm75b_setup.py

# 2. Check MuJoCo installation
python -c "import mujoco; print(f'MuJoCo {mujoco.__version__} installed')"

# 3. View robot model in MuJoCo viewer
python -m mujoco.viewer robots/rm75b_vr.xml

# 4. Check if server file exists
ls -lh run_server.py

# ============================================================================
# START SERVER
# ============================================================================

# Basic start (recommended)
python run_server.py \
    --backend mujoco \
    --mujoco-xml robots/rm75b_vr.xml \
    --mujoco-ee ee_site \
    --port 8000

# Start with custom IK parameters (smoother but slower)
python run_server.py \
    --backend mujoco \
    --mujoco-xml robots/rm75b_vr.xml \
    --mujoco-ee ee_site \
    --ik-damping 0.1 \
    --ik-max-iters 30 \
    --port 8000

# Start with slower motion (for beginners)
python run_server.py \
    --backend mujoco \
    --mujoco-xml robots/rm75b_vr.xml \
    --mujoco-ee ee_site \
    --max-qpos-step 0.05 \
    --port 8000

# ============================================================================
# NETWORK SETUP
# ============================================================================

# Find your local IP address
# macOS:
ifconfig | grep "inet " | grep -v 127.0.0.1 | awk '{print $2}'

# Linux:
hostname -I | awk '{print $1}'

# Check if port 8000 is available
lsof -i :8000

# Kill process using port 8000 (if needed)
lsof -ti :8000 | xargs kill -9

# ============================================================================
# QUEST 3S USB TETHERING
# ============================================================================

# Check if Quest is connected via USB
adb devices

# Enable Developer Mode on Quest (first time only):
# 1. Connect Quest to Meta Horizon app on phone
# 2. Menu → Devices → Headset Settings → Developer Mode → ON
# 3. Put on Quest, connect USB to Mac
# 4. Click "Allow USB Debugging" in Quest

# Forward ports from Mac to Quest
adb reverse tcp:8000 tcp:8000
adb reverse tcp:8080 tcp:8080

# Check port forwarding
adb reverse --list

# Remove port forwarding (if needed)
adb reverse --remove tcp:8000

# ============================================================================
# SERVER TESTING
# ============================================================================

# Check server health
curl http://localhost:8000/api/v1/statistics | python -m json.tool

# Get current robot position
curl http://localhost:8000/api/v1/position

# Get current orientation
curl http://localhost:8000/api/v1/orientation

# Send test command (move end effector)
curl -X POST http://localhost:8000/api/v1/teleop \
  -H "Content-Type: application/json" \
  -d '{"position": [0.3, 0.0, 0.5], "orientation": [1, 0, 0, 0]}'

# Check video stream endpoint
curl -I http://localhost:8000/api/v1/video/mjpeg

# ============================================================================
# KEYBOARD TESTING (Before VR)
# ============================================================================

# Test with Python keyboard client
python client/keyboard_client.py

# Test with web interface
open http://localhost:8000/web/
# Or manually navigate in browser to http://localhost:8000/web/

# ============================================================================
# MONITORING & DEBUGGING
# ============================================================================

# Monitor server logs (run server with log output)
python run_server.py \
    --backend mujoco \
    --mujoco-xml robots/rm75b_vr.xml \
    --mujoco-ee ee_site 2>&1 | tee server.log

# Watch server resource usage
top -pid $(pgrep -f "run_server.py")

# Check MuJoCo rendering performance
python -c "
import time
import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path('robots/rm75b_vr.xml')
d = mujoco.MjData(m)

start = time.time()
for _ in range(1000):
    mujoco.mj_step(m, d)
elapsed = time.time() - start
print(f'1000 steps in {elapsed:.3f}s = {1000/elapsed:.1f} steps/sec')
"

# ============================================================================
# FIREWALL CONFIGURATION (macOS)
# ============================================================================

# Temporarily disable firewall (for testing)
# System Settings → Network → Firewall → Off

# Or add Python to firewall allowed apps:
# System Settings → Network → Firewall → Options
# → Add Python.app → Allow incoming connections

# ============================================================================
# QUEST 3S BROWSER URLs
# ============================================================================

# WiFi connection (replace YOUR_IP with actual IP from ifconfig)
# http://YOUR_IP:8000/web/

# USB tethering connection
# http://localhost:8000/web/

# Login credentials:
# Username: operator
# Password: operator

# ============================================================================
# COMMON TROUBLESHOOTING
# ============================================================================

# "Cannot connect to server" in Quest
# → Check same WiFi network
# → Disable Mac firewall temporarily
# → Try USB tethering instead

# "WebXR not supported"
# → Update Quest firmware: Settings → System → Software Update
# → Use Meta Quest Browser (not Chrome)

# "Black screen in VR"
# → This is normal for MuJoCo simulation
# → You should see rendered MuJoCo scene, not camera feed

# "Robot moves too fast"
# → Restart server with: --max-qpos-step 0.05

# "Latency too high"
# → Use USB tethering instead of WiFi
# → Reduce IK iterations: --ik-max-iters 15

# ============================================================================
# CLEANUP
# ============================================================================

# Stop server (Ctrl+C in terminal)

# Remove ADB port forwarding
adb reverse --remove-all

# Kill any stuck Python processes
pkill -f "run_server.py"

# Check for stuck processes
ps aux | grep python

# ============================================================================
# USEFUL ONE-LINERS
# ============================================================================

# Complete test sequence
python test_rm75b_setup.py && \
python run_server.py --backend mujoco --mujoco-xml robots/rm75b_vr.xml --mujoco-ee ee_site

# Find and display local IP
echo "Connect Quest to: http://$(ifconfig | grep "inet " | grep -v 127.0.0.1 | awk '{print $2}' | head -1):8000/web/"

# Quick health check
curl -s http://localhost:8000/api/v1/statistics | python -c "import sys,json; data=json.load(sys.stdin); print(f'Status: {data[\"status\"]}, Backend: {data[\"backend\"]}')"

# ============================================================================
# ADVANCED: MULTIPLE ROBOTS
# ============================================================================

# Start with 2 robots (requires dual robot XML)
# python run_server.py \
#     --backend mujoco \
#     --mujoco-xml robots/rm75b_dual.xml \
#     --mujoco-ee "ee_site_left,ee_site_right"

# ============================================================================
# EXPORT & DATA COLLECTION
# ============================================================================

# Record trajectory (future feature)
# curl -X POST http://localhost:8000/api/v1/record/start
# ... perform movements ...
# curl -X POST http://localhost:8000/api/v1/record/stop

# ============================================================================
# HELP & DOCUMENTATION
# ============================================================================

# View server help
python run_server.py --help

# View MuJoCo backend options
python run_server.py --backend mujoco --help

# Read documentation
cat RM75B_QUICK_START.md
cat RM75B_VR_SETUP.md
cat RM75B_IMPLEMENTATION_SUMMARY.md

# ============================================================================
# Quick reference saved to: ~/teleop_system/commands_reference.sh
# Usage: cat commands_reference.sh
# ============================================================================
