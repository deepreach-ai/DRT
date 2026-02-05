# 🚀 TELEOP SYSTEM - LOCAL VALIDATION GUIDE

## Quick Start (3 Steps)

### Step 1: Run Automated Checks
```bash
cd /Users/ziguo/teleop_system
python3 validate_prereqs.py
```

This will check:
- ✅ Python version (3.8+)
- ✅ Virtual environment
- ✅ Required packages (FastAPI, MuJoCo, etc.)
- ✅ File structure
- ✅ Robot model files
- ✅ Port availability

### Step 2: Choose Validation Mode

**Option A: Interactive Menu (Easiest)**
```bash
chmod +x quick_validate.sh
./quick_validate.sh
```

**Option B: Manual Commands**

Start Mock Backend:
```bash
python run_server.py --backend mock
```

Start MuJoCo Backend:
```bash
python run_server.py --backend mujoco \
  --mujoco-xml robots/so101.xml \
  --mujoco-ee gripperframe
```

### Step 3: Test in Browser

1. Start web UI (in another terminal):
   ```bash
   python client/web_server.py
   ```

2. Open browser: `http://localhost:8080`

3. Test controls:
   - Login: operator / operator
   - Press "1" to activate safety
   - Use W/A/S/D keys to move
   - Check statistics update

---

## 📋 Complete Validation Checklist

### Backend Tests
- [ ] Mock backend starts without errors
- [ ] MuJoCo backend loads SO-101 model
- [ ] Server responds on http://localhost:8000
- [ ] Health endpoint returns 200 OK
- [ ] No critical errors in terminal

### API Tests
```bash
# Make sure server is running first!

# Health check
curl http://localhost:8000/health

# Get statistics
curl http://localhost:8000/api/v1/statistics | python -m json.tool

# Activate safety
curl -X POST http://localhost:8000/api/v1/safety/activate

# Send command
curl -X POST http://localhost:8000/api/v1/command/delta \
  -H "Content-Type: application/json" \
  -d '{
    "dx": 0.01, "dy": 0.0, "dz": 0.0,
    "droll": 0.0, "dpitch": 0.0, "dyaw": 0.0,
    "max_velocity": 0.1,
    "max_angular_velocity": 0.5,
    "timestamp": 0.0,
    "client_id": "test"
  }'
```

### Web UI Tests
- [ ] Page loads at http://localhost:8080
- [ ] Login form appears
- [ ] Login works (operator/operator)
- [ ] WebSocket connects (check browser console)
- [ ] Safety activation works (press "1")
- [ ] Keyboard controls work (W/A/S/D)
- [ ] Statistics update in real-time
- [ ] No JavaScript errors in console

### Client Tests
```bash
# Start keyboard client
python client/keyboard_client.py

# Test:
# - Press W/A/S/D keys
# - Verify commands sent at 20Hz
# - Check position updates
# - Press Ctrl+C to exit
```

---

## 📊 Expected Results

### Successful Mock Backend Startup
```
Starting teleoperation server on 0.0.0.0:8000
Using backend: mock
INFO:     Started server process [12345]
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000
```

### Successful MuJoCo Backend Startup
```
Starting teleoperation server on 0.0.0.0:8000
Using backend: mujoco
INFO:     Started server process [12345]
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000
```

⚠️ **Note:** On macOS you may see:
```
[MuJoCo] Rendering failed: ...
[MuJoCo] Falling back to placeholder rendering
```
This is **expected and OK**. The simulation still works; only 3D visualization is affected.

### Successful Web UI Behavior
1. Page loads with blue gradient
2. "Disconnected" status (orange/yellow)
3. After login: "Connected" status (green)
4. After safety: "Safety Active" (green)
5. Statistics update continuously
6. Keyboard controls send commands

---

## 🐛 Troubleshooting

### "Module not found"
```bash
# Solution: Install dependencies
source .venv/bin/activate  # if using venv
pip install -r server/requirements.txt
```

### "Address already in use"
```bash
# Solution: Kill process on port 8000
lsof -ti:8000 | xargs kill -9

# Or use different port
python run_server.py --backend mock --port 8001
```

### "Could not load MuJoCo model"
```bash
# Check file exists
ls -la robots/so101.xml
ls -la robots/assets/*.stl

# Use absolute path
python run_server.py --backend mujoco \
  --mujoco-xml /Users/ziguo/teleop_system/robots/so101.xml \
  --mujoco-ee gripperframe
```

### "WebSocket connection failed"
1. Check server is running: `curl http://localhost:8000/health`
2. Check browser console for errors (F12)
3. Verify web_server.py is running on port 8080
4. Try refreshing browser page

### Robot Shaking/Unstable in MuJoCo
This should be fixed in `robots/so101.xml`, but if you see shaking:
1. Check if `<option integrator="implicit">` is present
2. Verify `kp="50"` (not 998.22)
3. Verify `damping="2.0"` (not 0.6)
4. Use the stable version: `/Users/ziguo/so101_new_stable.xml`

---

## 📖 Documentation Files

Created for you:
- **VALIDATION_STEPS.md** - Detailed validation procedures
- **validate_prereqs.py** - Automated prerequisite checker
- **quick_validate.sh** - Interactive validation menu
- **THIS FILE** - Quick reference guide

Existing documentation:
- **QUICKSTART.md** - Original quick start guide
- **README.md** - System architecture overview
- **CHEATSHEET.md** - Command reference

---

## ✅ Success Criteria

Your validation is successful if:

1. ✅ Prerequisite checks pass (python, packages, files)
2. ✅ Server starts with Mock backend
3. ✅ Server starts with MuJoCo backend
4. ✅ Web UI loads and connects
5. ✅ Commands can be sent via API
6. ✅ Keyboard controls work
7. ✅ No crashes during 5+ minute test

---

## 🎯 What To Test

### Minimum Validation (15 minutes)
1. Run `validate_prereqs.py`
2. Start server with Mock backend
3. Test health endpoint
4. Start web UI
5. Login and test basic controls

### Complete Validation (30 minutes)
1. All minimum validation steps
2. Test MuJoCo backend
3. Test all API endpoints
4. Test keyboard client
5. Test WebSocket connection
6. Document any issues

### Full Integration Test (60 minutes)
1. All complete validation steps
2. Test with both backends
3. Measure latency
4. Test safety features
5. Test workspace limits
6. Prepare demo for stakeholders

---

## 📝 Validation Report Template

After testing, fill this out:

```
=== TELEOP VALIDATION REPORT ===
Date: [DATE]
Tester: [NAME]
Duration: [MINUTES]

Environment:
- OS: macOS [VERSION]
- Python: [VERSION]
- MuJoCo: [VERSION]

Tests Performed:
[ ] Prerequisite checks
[ ] Mock backend
[ ] MuJoCo backend
[ ] Web UI
[ ] API endpoints
[ ] Keyboard client
[ ] WebSocket

Results:
- Tests Passed: __/7
- Tests Failed: __/7
- Issues Found: [LIST]

Performance:
- Server startup time: __ seconds
- API latency: __ ms
- WebSocket latency: __ ms
- Command rate achieved: __ Hz

Issues Encountered:
1. [ISSUE DESCRIPTION]
   Resolution: [SOLUTION]

2. [ISSUE DESCRIPTION]
   Resolution: [SOLUTION]

Recommendations:
[YOUR NOTES]

Overall Status: PASS / FAIL / PARTIAL
Ready for Demo: YES / NO / WITH CAVEATS

Next Steps:
1. [ACTION ITEM]
2. [ACTION ITEM]
```

---

## 🚀 After Validation

Once all tests pass:

1. **Document findings** - Create validation report
2. **Fix any issues** - Address warnings/failures
3. **Test with stable XML** - Use corrected robot model
4. **Prepare demo** - Get ready to show stakeholders
5. **Plan hardware integration** - SO-ARM100 + RealSense D455

---

**Good luck! If you encounter issues not covered here, check the server logs and browser console for detailed error messages.**
