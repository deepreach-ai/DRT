# ✅ CORRECTED: How to Run Teleop System Locally

## 🎯 The Right Way (Single Server)

### Option 1: Mock Backend (No Robot Simulation)
```bash
cd /Users/ziguo/teleop_system
python run_server.py --backend mock

# Open browser to: http://localhost:8000/web/
# Or just: http://localhost:8000
```

### Option 2: MuJoCo Backend (Robot Simulation)
```bash
cd /Users/ziguo/teleop_system
python run_server.py --backend mujoco \
  --mujoco-xml robots/so101.xml \
  --mujoco-ee gripperframe

# Open browser to: http://localhost:8000/web/
```

### Option 3: Isaac Sim Backend
```bash
cd /Users/ziguo/teleop_system
python run_server.py --backend isaac

# Open browser to: http://localhost:8000/web/
```

---

## 🌐 Web UI Login

**After opening http://localhost:8000/web/ :**

1. **Leave "Server Base URL" EMPTY** (or enter `http://localhost:8000`)
2. **Username:** `operator`
3. **Password:** `operator`
4. **Click Login**

**Expected:** Status changes to "Connected" (green)

---

## ⚠️ Common Mistake (DON'T DO THIS)

**❌ WRONG:**
```bash
# Don't run web_server.py - it's incomplete!
python client/web_server.py  # ← This doesn't have auth endpoints
```

**✅ CORRECT:**
```bash
# Just run teleop server - it has everything!
python run_server.py --backend mujoco --mujoco-xml robots/so101.xml --mujoco-ee gripperframe
```

---

## 🎮 Keyboard Controls (After Login)

### Step 1: Login
- Open: `http://localhost:8000/web/`
- Login: operator / operator
- **Leave Base URL empty!**

### Step 2: Start Controlling
1. **Click anywhere on the page** (for keyboard focus)
2. **Hold W** → Robot moves forward
3. **Hold A** → Robot moves left
4. **Hold Q** → Robot moves up
5. Check "keys: " indicator shows active keys

### Step 3: Watch Feedback
- **Position:** Updates in real-time
- **Video:** Shows robot state (or placeholder)
- **Command Log:** Shows acknowledgements

---

## 📊 What You Should See

### Terminal Output (Server)
```
Starting teleoperation server on 0.0.0.0:8000
Using backend: mujoco
INFO:     Started server process [12345]
[Server] Initializing with mujoco backend...
[Server] Initialized successfully with mujoco backend
INFO:     Application startup complete.
[FastAPI] Teleoperation server started on http://localhost:8000
[FastAPI] Backend type: mujoco
[FastAPI] API Documentation: http://localhost:8000/docs
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
```

### Browser (After Login)
```
✅ Status: Connected (green)
✅ WebSocket connected
✅ Video stream shows robot (or placeholder)
✅ Position: [0.000, 0.000, 0.500]
✅ Orientation: [1.000, 0.000, 0.000, 0.000]
✅ Backend: mujoco connected=true
```

---

## 🐛 Troubleshooting

### "Login failed: Not Found"
**Cause:** Trying to login to web_server.py (port 8080) which doesn't have auth
**Fix:** Use teleop server directly at `http://localhost:8000/web/`

### "Cannot connect to server"
**Cause:** Server not running
**Fix:**
```bash
# Check if server is running
curl http://localhost:8000/health

# If not, start it
python run_server.py --backend mock
```

### "Page not found" at localhost:8000
**Fix:** Add `/web/` to the URL: `http://localhost:8000/web/`

### "WebSocket connection failed"
**Cause:** Server Base URL filled in incorrectly
**Fix:** Leave it **EMPTY** or enter exactly `http://localhost:8000`

---

## ✅ Quick Validation

Run these commands to verify everything works:

```bash
# 1. Start server
python run_server.py --backend mock

# 2. In another terminal, test API
curl http://localhost:8000/health
# Expected: {"status":"healthy","backend":"mock"}

curl http://localhost:8000/api/v1/statistics
# Expected: JSON with statistics

# 3. Test web UI manually
open http://localhost:8000/web/

# 4. Login and test keyboard controls
# Follow steps above
```

---

## 📝 Summary

**ONE SERVER** runs everything:
- Port **8000** = Teleop Server (FastAPI)
  - Serves web UI at `/web/`
  - Handles auth at `/api/v1/auth/login`
  - Handles commands at `/api/v1/command`
  - WebSocket at `/ws/v1/teleop`

**NO NEED** for separate web server (port 8080)!

**Correct workflow:**
```bash
python run_server.py --backend mujoco --mujoco-xml robots/so101.xml --mujoco-ee gripperframe
open http://localhost:8000/web/
# Login → Control robot with keyboard!
```

That's it! 🚀
