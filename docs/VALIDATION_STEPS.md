# Teleoperation System - Local Validation Steps (No Changes)

## 📋 Overview
This document provides step-by-step validation for running the teleoperation system locally **without making any changes**. We'll test with the MuJoCo backend since you have the SO-101 robot model.

---

## 🔍 Pre-Validation Checklist

### Check Python Environment
```bash
cd /Users/ziguo/drt

# Check if virtual environment exists
ls -la .venv

# Activate virtual environment
source .venv/bin/activate

# Verify Python version (should be 3.8+)
python --version

# Check installed packages
pip list | grep -E "fastapi|uvicorn|mujoco|numpy"
```

**Expected Output:**
```
fastapi       0.104.1
uvicorn       0.24.0
mujoco        3.1.6
numpy         1.26.4
```

### Check MuJoCo Installation
```bash
# Test MuJoCo Python bindings
python -c "import mujoco; print(f'MuJoCo version: {mujoco.__version__}')"

# Check if robot model exists
ls -la robots/so101_new.xml

# Check if mesh assets exist
ls -la robots/assets/*.stl | wc -l
```

**Expected Output:**
```
MuJoCo version: 3.1.6
-rw-r--r--  1 ziguo  staff  45678 ... robots/so101_new.xml
      13
```

---

## 🎯 Validation Test Plan

### Test 1: Verify Server Starts (Mock Backend)
**Purpose:** Confirm the FastAPI server starts without errors

```bash
cd /Users/ziguo/drt

# Terminal 1: Start server with Mock backend
python run_server.py --backend mock --host 0.0.0.0 --port 8000
```

**Expected Output:**
```
Starting teleoperation server on 0.0.0.0:8000
Using backend: mock
INFO:     Started server process [xxxxx]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
```

**Validation Checks:**
- ✅ Server starts without errors
- ✅ No Python import errors
- ✅ Port 8000 is listening
- ✅ Backend initializes successfully

**Quick Test:**
```bash
# In another terminal
curl http://localhost:8000/health
# Expected: {"status":"healthy","backend":"mock"}

curl http://localhost:8000/api/v1/statistics
# Expected: JSON with backend_status, controller_stats, etc.
```

**Stop server:** Press `Ctrl+C`

---

### Test 2: Verify Server Starts (MuJoCo Backend)
**Purpose:** Confirm MuJoCo backend loads the robot model

```bash
cd /Users/ziguo/drt

# Set environment variables for MuJoCo
export TELEOP_MUJOCO_XML="/Users/ziguo/drt/robots/so101_new.xml"
export TELEOP_MUJOCO_EE_SITE="gripperframe"
export TELEOP_MUJOCO_CAMERA="main"

# Start server with MuJoCo backend
python run_server.py --backend mujoco \
  --mujoco-xml robots/so101_new.xml \
  --mujoco-ee gripperframe
```

**Expected Output:**
```
Starting teleoperation server on 0.0.0.0:8000
Using backend: mujoco
[MuJoCo] Loading model from: /Users/ziguo/drt/robots/so101_new.xml
INFO:     Started server process [xxxxx]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000
```

**Validation Checks:**
- ✅ MuJoCo model loads without errors
- ✅ End-effector site "gripperframe" found
- ✅ Server responds to API requests
- ✅ No OpenGL rendering errors (on macOS this is expected)

**Quick Test:**
```bash
# Check backend status
curl http://localhost:8000/api/v1/statistics | python -m json.tool

# Expected output should include:
# {
#   "backend_status": {
#     "name": "mujoco_robot",
#     "status": "connected",
#     "backend": "mujoco",
#     "current_position": [x, y, z],
#     "current_orientation": [w, x, y, z]
#   }
# }
```

**Stop server:** Press `Ctrl+C`

---

### Test 3: Web UI Server
**Purpose:** Verify the web interface starts correctly

```bash
cd /Users/ziguo/drt

# Terminal 1: Start teleop server (keep running)
python run_server.py --backend mock

# Terminal 2: Start web UI server
python client/web_server.py
```

**Expected Output (Terminal 2):**
```
INFO:     Started server process [xxxxx]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8080
```

**Validation Checks:**
- ✅ Web server starts on port 8080
- ✅ No HTML/CSS/JS errors
- ✅ Static files served correctly

**Quick Test:**
```bash
# Check if web UI is accessible
curl http://localhost:8080 | head -20
# Expected: HTML content with "Operator Console"

# Check if web server can reach teleop server
curl http://localhost:8080/api/v1/statistics
# Expected: Should proxy to http://localhost:8000/api/v1/statistics
```

---

### Test 4: Browser Testing
**Purpose:** Verify full end-to-end web interface

**Steps:**
1. Keep both servers running (Terminal 1 & 2)
2. Open browser: `http://localhost:8080`
3. Test the UI:

**Expected Browser Behavior:**

| Action | Expected Result |
|--------|----------------|
| Load page | ✅ Blue gradient background, "Operator Console" header |
| Status pill | ✅ Shows "Disconnected" (yellow/orange) |
| Login button | ✅ Click shows login form |
| Login (operator/operator) | ✅ Status changes to "Connected" (green) |
| Press "1" key | ✅ Safety activates (green status) |
| Press "W" key | ✅ Command sent, position updates |
| Press "A/S/D" keys | ✅ Position changes in real-time |
| Check statistics | ✅ Command count increases, latency shown |
| Video placeholder | ⚠️ Shows "Video Stream Not Available" (expected) |

**Browser Console Checks:**
```javascript
// Open Developer Tools (F12)
// Check Console tab - should see:
"ws connected"
"Login ok"

// No errors like:
// ❌ "WebSocket connection failed"
// ❌ "Failed to fetch"
```

---

### Test 5: Keyboard Client
**Purpose:** Test the Python keyboard controller

```bash
cd /Users/ziguo/drt

# Terminal 1: Server should be running
python run_server.py --backend mock

# Terminal 2: Run keyboard client
python client/keyboard_client.py
```

**Expected Output:**
```
Keyboard Teleoperation Client
Press keys to control robot:
  W/S: Forward/Backward (Y-axis)
  A/D: Left/Right (X-axis)
  Q/E: Up/Down (Z-axis)
  ...
Press CTRL+C to exit

Connected to server
Sending commands at 20Hz...
```

**Validation Checks:**
- ✅ Client connects to server
- ✅ Keyboard events detected
- ✅ Commands sent continuously
- ✅ Status updates received

**Interactive Test:**
Press keys and observe output:
```
>>> Press 'W' key
Command sent: dx=0.00, dy=0.02, dz=0.00
Position: [0.000, 0.020, 0.500]

>>> Press 'Q' key
Command sent: dx=0.00, dy=0.00, dz=0.02
Position: [0.000, 0.020, 0.520]
```

---

### Test 6: API Endpoints
**Purpose:** Verify all REST API endpoints work

```bash
# Make sure server is running on port 8000

# 1. Health check
curl http://localhost:8000/health

# 2. Get statistics
curl http://localhost:8000/api/v1/statistics | python -m json.tool

# 3. Activate safety
curl -X POST http://localhost:8000/api/v1/safety/activate

# 4. Send delta command
curl -X POST http://localhost:8000/api/v1/command/delta \
  -H "Content-Type: application/json" \
  -d '{
    "dx": 0.01,
    "dy": 0.0,
    "dz": 0.0,
    "droll": 0.0,
    "dpitch": 0.0,
    "dyaw": 0.0,
    "max_velocity": 0.1,
    "max_angular_velocity": 0.5,
    "timestamp": 0.0,
    "client_id": "test"
  }'

# 5. Get video stream (should return placeholder)
curl http://localhost:8000/api/v1/video/mjpeg --output test_frame.jpg

# 6. Interactive API docs
# Open browser: http://localhost:8000/docs
```

**Expected Results:**
- All endpoints return 200 OK
- JSON responses are well-formed
- Safety activation works
- Commands are accepted
- Video endpoint returns JPEG data

---

### Test 7: WebSocket Connection
**Purpose:** Verify WebSocket real-time updates

**Simple WebSocket Test Client:**
```python
# Create file: test_websocket.py
import asyncio
import websockets
import json

async def test_ws():
    uri = "ws://localhost:8000/ws"
    async with websockets.connect(uri) as websocket:
        print("✅ WebSocket connected!")
        
        # Listen for 10 messages
        for i in range(10):
            message = await websocket.recv()
            data = json.loads(message)
            print(f"Received: {data['type']}")
            
        print("✅ WebSocket test passed!")

asyncio.run(test_ws())
```

**Run test:**
```bash
# Make sure server is running
python test_websocket.py
```

**Expected Output:**
```
✅ WebSocket connected!
Received: state
Received: state
Received: state
...
✅ WebSocket test passed!
```

---

## 📊 Validation Checklist Summary

After running all tests, verify:

### Server
- [ ] Mock backend starts without errors
- [ ] MuJoCo backend loads SO-101 model
- [ ] FastAPI server responds on port 8000
- [ ] Health endpoint returns 200 OK
- [ ] Statistics endpoint returns valid JSON
- [ ] WebSocket accepts connections

### Web UI
- [ ] Web server starts on port 8080
- [ ] Browser loads page without errors
- [ ] Login form works (operator/operator)
- [ ] WebSocket connects successfully
- [ ] Keyboard controls send commands
- [ ] Statistics update in real-time
- [ ] Safety activation works

### Client
- [ ] Python keyboard client connects
- [ ] Commands sent at 20Hz
- [ ] Robot position updates
- [ ] No connection errors

### API
- [ ] All REST endpoints return 200
- [ ] Command delta endpoint works
- [ ] Safety endpoints function
- [ ] Video endpoint returns data (placeholder)

---

## 🐛 Common Issues & Solutions

### Issue 1: "Module not found: fastapi"
```bash
# Solution: Install dependencies
cd /Users/ziguo/drt
source .venv/bin/activate
pip install -r server/requirements.txt
```

### Issue 2: "MuJoCo: Could not load model"
```bash
# Solution: Check file path and mesh directory
ls -la robots/so101_new.xml
ls -la robots/assets/*.stl

# Try absolute path
python run_server.py --backend mujoco \
  --mujoco-xml /Users/ziguo/drt/robots/so101_new.xml
```

### Issue 3: "Address already in use (port 8000)"
```bash
# Solution: Kill existing process
lsof -ti:8000 | xargs kill -9

# Or use different port
python run_server.py --backend mock --port 8001
```

### Issue 4: "WebSocket connection failed"
```bash
# Solution: Check server logs
# Make sure server is running on same host/port
# Try browser console for detailed error

# Test WebSocket manually:
wscat -c ws://localhost:8000/ws
```

### Issue 5: "OpenGL rendering failed" (macOS)
```
# Expected on macOS - this is normal
# MuJoCo falls back to placeholder rendering
# The simulation still works, just no 3D visualization
```

---

## ✅ Success Criteria

Your system is validated if:

1. **Server starts successfully** with both Mock and MuJoCo backends
2. **Web UI loads** and connects to server
3. **Commands can be sent** via keyboard or web interface
4. **Safety system works** (activate/deactivate)
5. **Real-time updates** via WebSocket
6. **API endpoints respond** correctly
7. **No crashes** during 5+ minute test session

---

## 📝 Test Results Template

```
=== TELEOP SYSTEM VALIDATION RESULTS ===
Date: YYYY-MM-DD
Tester: Your Name

Backend Tests:
[ ] Mock backend starts        - PASS/FAIL - Notes: _______
[ ] MuJoCo backend starts      - PASS/FAIL - Notes: _______
[ ] Robot model loads          - PASS/FAIL - Notes: _______

Server Tests:
[ ] Health endpoint            - PASS/FAIL - Notes: _______
[ ] Statistics endpoint        - PASS/FAIL - Notes: _______
[ ] Command endpoint           - PASS/FAIL - Notes: _______
[ ] WebSocket connection       - PASS/FAIL - Notes: _______

Web UI Tests:
[ ] Page loads                 - PASS/FAIL - Notes: _______
[ ] Login works                - PASS/FAIL - Notes: _______
[ ] Safety activation          - PASS/FAIL - Notes: _______
[ ] Keyboard controls          - PASS/FAIL - Notes: _______
[ ] Statistics display         - PASS/FAIL - Notes: _______

Client Tests:
[ ] Keyboard client connects   - PASS/FAIL - Notes: _______
[ ] Commands sent at 20Hz      - PASS/FAIL - Notes: _______
[ ] Position updates           - PASS/FAIL - Notes: _______

Overall Status: PASS / FAIL / PARTIAL
Issues Found: _________________________________
Next Steps: ___________________________________
```

---

## 🎯 Next Steps After Validation

Once all tests pass:

1. **Document findings** - Note any issues or warnings
2. **Test with stable XML** - Use the corrected so101_new_stable.xml
3. **Measure latency** - Run latency tests locally
4. **Prepare demo** - Get ready to show Chris
5. **Plan integration** - Prepare for real hardware (SO-ARM100 + RealSense D455)

---

**Good luck with validation! 🚀**

If you encounter any issues not covered here, check:
- Server logs in Terminal 1
- Browser console (F12 → Console tab)
- Python error traceback messages
