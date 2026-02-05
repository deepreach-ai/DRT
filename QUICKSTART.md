# Quick Start Guide - Web Teleoperation

## 🚀 Test the Web UI (Before Hardware Arrives)

### Step 1: Start the Teleoperation Server
```bash
cd /Users/ziguo/teleop_system

# Start with Mock backend (no robot needed)
python run_server.py --backend mock

# Or with Isaac Sim backend (if you have it running)
# python run_server.py --backend isaac
```

The server will start on `http://localhost:8000`

### Step 2: Start the Web UI Server
Open a **new terminal**:
```bash
cd /Users/ziguo/teleop_system

# Start web UI server
python client/web_server.py
```

The web UI will be available at `http://localhost:8080`

### Step 3: Open Browser and Test!
1. Open browser to `http://localhost:8080`
2. Click "Connect to Robot"
3. Try keyboard controls:
   - **W/S**: Up/Down
   - **A/D**: Left/Right
   - **Q/E**: Forward/Backward
   - **1**: Activate Safety (must press first!)
   - **M**: Toggle mode

Or use the virtual joystick buttons on screen!

---

## 📊 What You Should See

### Initial State:
```
✅ Web page loads with blue gradient background
✅ Status shows "Disconnected" and "Safety Inactive"
✅ Virtual joystick buttons visible
✅ Video placeholder says "Video Stream Not Available"
```

### After Clicking "Connect":
```
✅ Status changes to "Connected"
✅ WebSocket connects to backend
✅ Can send commands using keyboard or buttons
✅ Statistics update (command count, latency, uptime)
```

### After Pressing "1" (Activate Safety):
```
✅ Status shows "Safety Active" in green
✅ Robot will now respond to movement commands
✅ Commands are being sent at 20Hz
```

---

## 🧪 Testing Checklist

Before hardware arrives, verify:

- [ ] Web UI loads without errors (check browser console)
- [ ] WebSocket connects to server
- [ ] Safety activation works (press "1")
- [ ] Position control works (W/A/S/D/Q/E keys)
- [ ] Orientation control works (press M, then I/J/K/L/U/O)
- [ ] Virtual joystick buttons work (mouse/touch)
- [ ] Statistics update (command count increases)
- [ ] Latency is reasonable (<50ms on localhost)
- [ ] Disconnect button works
- [ ] Reconnect works after disconnect

---

## 🎯 Next Steps (This Week)

### 1. Add Video Streaming
Currently the video placeholder shows "not available". You need to add:

**Option A: Test with Isaac Sim (if you have it)**
- Isaac Sim has built-in cameras
- Export frames and stream to web UI

**Option B: Wait for RealSense D455**
- Will provide real robot camera feed
- 848×480 @ 90fps RGB + Depth

**Implementation**: Add video streaming endpoint to server:
```python
# In server/teleop_server.py
@app.get("/api/v1/video/stream")
async def video_stream():
    # Return MJPEG stream or setup WebRTC
    pass
```

### 2. Deploy to Cloud (Test Mexico → USA Latency)
```bash
# Deploy to AWS/Azure/etc
# Get public IP
# Test from different locations
```

### 3. Add Session Recording
```python
# Record commands + video for review
# Save to disk with timestamp
# Implement playback feature
```

### 4. Show Demo to Chris!
Once you have:
- ✅ Web UI working
- ✅ Mock backend responding
- ✅ Basic controls functional

→ Schedule demo with Chris to show progress before hardware arrives!

---

## 🔧 Troubleshooting

### "Cannot connect to server"
- Make sure `run_server.py` is running on port 8000
- Check firewall settings
- Try `http://localhost:8000/docs` to see if server is up

### "WebSocket connection failed"
- Server must support WebSocket (FastAPI already does)
- Check browser console for error messages
- Verify URL in `teleop.js` matches your server

### "Safety Inactive" won't change
- Press "1" key or click the "⚡ Activate" button
- Check server logs for safety activation
- Try POST to `http://localhost:8000/api/v1/safety/activate`

### Keyboard controls not working
- Make sure browser window has focus
- Check browser console for JavaScript errors
- Try virtual joystick buttons instead

---

## 📝 Current System Status

### ✅ What You Have:
- FastAPI backend with WebSocket support
- Mock backend for testing (no robot needed)
- Isaac Sim backend (if you have Isaac Sim)
- Safety gate system
- Control logic with workspace limits
- **NEW**: Web UI with virtual joystick
- **NEW**: Keyboard control support
- **NEW**: Real-time statistics display

### ❌ What's Missing (Pre-Hardware):
- Video streaming (waiting for RealSense D455)
- Session recording
- Authentication/login
- SO-ARM100 backend (waiting for robot)

### 🎯 What's Coming (When Hardware Arrives):
- RealSense D455 integration
- SO-ARM100 robot control
- Real video streaming @ 90fps
- VR interface (Meta Quest 3 or Pico 4)

---

## 💡 Tips

1. **Test thoroughly with Mock backend**
   - No risk of damaging hardware
   - Fast iteration on UI/UX
   - Validate all features work

2. **Measure baseline latency**
   - Test on local network first
   - Then test over internet
   - Document latency numbers for Chris

3. **Get feedback early**
   - Show Chris the web UI demo
   - Ask Mexico operators what they need
   - Iterate based on real user feedback

4. **Document everything**
   - Take screenshots of working UI
   - Record demo video
   - Write setup instructions for operators

---

## 🎉 Success Metrics

Your Week 1 demo is successful if:
- ✅ Chris can open browser and control robot (mock)
- ✅ Latency is measured and reasonable
- ✅ Safety system works
- ✅ No crashes during 10+ minute session
- ✅ You can explain the architecture clearly

**You're 80% done with Phase 1!** Just need:
- Video streaming (can mock for now)
- Session recording
- Deploy to test USA ↔ Mexico latency

Keep going! 🚀
