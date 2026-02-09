# 🎮 RM75B VR Control - Quick Reference

## 🚀 One-Command Start

```bash
cd ~/teleop_system

# Test setup first
python test_rm75b_setup.py

# Start VR server
python run_server.py \
    --backend mujoco \
    --mujoco-xml robots/rm75b_vr.xml \
    --mujoco-ee ee_site \
    --port 8000
```

## 📱 Connect Quest 3S

### WiFi Method
1. Find your IP: `ifconfig | grep "inet " | grep -v 127.0.0.1`
2. Quest Browser → `http://YOUR_IP:8000/web/`
3. Login: `operator` / `operator`
4. Click "Enter VR Mode"

### USB Method (Lower Latency)
```bash
adb reverse tcp:8000 tcp:8000
# Quest Browser → http://localhost:8000/web/
```

## 🎮 Controls

| Action | Control |
|--------|---------|
| Move Robot | Right controller position |
| Rotate Robot | Right controller orientation |
| Close Gripper | Right trigger |
| Open Gripper | Release trigger |
| Emergency Stop | B button |
| Reset Position | Y button |
| Exit VR | Menu button |

## 📁 Files Created

```
~/teleop_system/
├── robots/rm75b_vr.xml          # MuJoCo model (7-DOF)
├── test_rm75b_setup.py          # Verification script
├── RM75B_VR_SETUP.md            # Detailed guide
└── start_rm75b_vr.sh            # Auto-start script
```

## ⚡ Quick Tests

```bash
# 1. Verify setup
python test_rm75b_setup.py

# 2. Check server health
curl http://localhost:8000/api/v1/statistics | python -m json.tool

# 3. Test keyboard control (before VR)
python client/keyboard_client.py

# 4. View in MuJoCo viewer
python -m mujoco.viewer robots/rm75b_vr.xml
```

## 🔧 Common Issues

**"Cannot connect"** → Check firewall, verify same WiFi
**"WebXR not supported"** → Update Quest firmware
**"Robot too fast"** → Add `--max-qpos-step 0.05` to server command
**"Black screen"** → MuJoCo uses render output, not real cameras

## 📚 Full Documentation

- **Detailed Setup**: [RM75B_VR_SETUP.md](RM75B_VR_SETUP.md)
- **General VR Guide**: [docs/VR_SETUP.md](docs/VR_SETUP.md)
- **Troubleshooting**: [docs/LOCAL_VALIDATION_GUIDE.md](docs/LOCAL_VALIDATION_GUIDE.md)

## ✅ Success Criteria

- [ ] `test_rm75b_setup.py` passes all tests
- [ ] Server starts without errors
- [ ] Quest connects and enters VR
- [ ] Controller moves robot smoothly
- [ ] Latency < 50ms (check in VR)

---

**Status**: ✅ Ready for testing
**Last Updated**: 2024-02-09
