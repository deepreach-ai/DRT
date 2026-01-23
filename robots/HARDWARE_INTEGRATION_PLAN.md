# Hardware Integration Plan
## Approved Equipment List & Integration Strategy

### 📦 Hardware Inventory

#### 1. Robot Platform
- **LeRobot SO-ARM100** ×1 (+ Lerobot_plan2)
  - 6-DOF robotic arm
  - Integration: Python SDK / ROS interface
  - Control frequency: TBD (typically 20-100Hz)

#### 2. Vision System
- **Primary Camera**: Intel RealSense D455 ×1
  - Resolution: 848×480 @ 90fps
  - Outputs: RGB + Depth streams
  - Mounting: Robot end-effector or fixed front view
  - SDK: pyrealsense2
  
- **Secondary Camera** (Optional): 
  - Additional RGB camera for third-person view
  - Or second RealSense D455 (if budget permits)

#### 3. VR Headsets
- **Primary Option**: Meta Quest 3
  - WiFi 6E support (lower latency)
  - Resolution: 2064×2208 per eye
  - Consumer-friendly, good app ecosystem
  
- **Alternative**: Pico 4 Enterprise
  - Better for commercial/business applications
  - Enterprise management features
  - Similar specs to Quest 3

#### 4. Control Station
- **ROG Gaming PC** (高端电竞主机)
  - Runs: Isaac Sim (simulation), Control Server, Video Encoding
  - GPU: Required for Isaac Sim and video encoding
  - Network: WiFi 6E support for VR headset streaming

---

## 🏗️ System Architecture (Updated)

```
┌─────────────────────────────────────────────────────────────┐
│                    OPERATOR (Mexico)                         │
│                                                              │
│  ┌──────────────┐              ┌──────────────┐            │
│  │  Web Browser │              │  VR Headset  │            │
│  │   (Phase 1)  │              │  (Phase 3)   │            │
│  │              │              │              │            │
│  │ - Video      │              │ - Stereo 3D  │            │
│  │ - Controls   │              │ - 6DOF Track │            │
│  └──────┬───────┘              └──────┬───────┘            │
│         │                             │                     │
└─────────┼─────────────────────────────┼─────────────────────┘
          │         WebSocket/WebRTC    │
          │                             │
          └──────────────┬──────────────┘
                         │ Internet (USA ↔ Mexico)
                         ↓
┌─────────────────────────────────────────────────────────────┐
│                  CONTROL SERVER (USA)                        │
│                    ROG Gaming PC                             │
│                                                              │
│  ┌────────────────────────────────────────────────────┐    │
│  │           FastAPI Teleoperation Server              │    │
│  │  - WebSocket control endpoint                       │    │
│  │  - Video streaming (WebRTC/MJPEG)                  │    │
│  │  - Safety gate & limits                            │    │
│  │  - Session recording                               │    │
│  └───────────────┬─────────────────┬──────────────────┘    │
│                  │                 │                         │
│                  ↓                 ↓                         │
│  ┌───────────────────┐  ┌──────────────────────┐          │
│  │  Video Pipeline   │  │   Robot Backend      │          │
│  │                   │  │   (Abstract Layer)    │          │
│  │ - RealSense D455  │  │                      │          │
│  │ - RGB @ 90fps     │  │   ┌──────────────┐  │          │
│  │ - Depth @ 90fps   │  │   │ SO-ARM100    │  │          │
│  │ - H.264 encode    │  │   │  Backend     │  │          │
│  │ - WebRTC stream   │  │   └──────────────┘  │          │
│  └───────────────────┘  └──────────┬───────────┘          │
│                                     │                        │
└─────────────────────────────────────┼────────────────────────┘
                                      │
                                      ↓
                          ┌────────────────────┐
                          │  SO-ARM100 Robot   │
                          │   (Physical Arm)   │
                          └────────────────────┘
```

---

## 📋 Phase-by-Phase Implementation

### **Phase 1: Web UI + Simulation (CURRENT - Week 1-2)**
**Goal**: Prove teleoperation concept before hardware arrives

✅ **Already Built:**
- FastAPI server with WebSocket support
- Mock backend for testing
- Isaac Sim backend (simulation)
- Keyboard client

🔨 **Build Now:**
- [ ] Complete web UI (index.html + teleop.js)
- [ ] Add video streaming from Isaac Sim camera
- [ ] Implement session recording (commands + video)
- [ ] Add authentication/login system
- [ ] Test Mexico → USA latency (VPN or cloud deployment)

**Deliverable**: Working web demo that Chris can test

---

### **Phase 2: Hardware Integration (Week 3-4, when hardware arrives)**

#### 2A. RealSense D455 Integration
```python
# New file: server/video/realsense_stream.py
import pyrealsense2 as rs
import cv2
import numpy as np

class RealSenseStreamer:
    """Manages RealSense D455 RGB-D streaming"""
    
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Configure streams: 848×480 @ 90fps
        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 90)
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 90)
        
    def start(self):
        self.pipeline.start(self.config)
        
    def get_frames(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        
        # Convert to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        return color_image, depth_image
```

**Tasks:**
- [ ] Install pyrealsense2 SDK
- [ ] Test RealSense D455 connection
- [ ] Integrate with video streaming pipeline
- [ ] Add depth visualization overlay (optional for operators)

#### 2B. SO-ARM100 Robot Backend
```python
# New file: server/backends/soarm_backend.py
from robot_backend import RobotBackend
import numpy as np
# Import SO-ARM100 SDK (TBD based on actual SDK)

class SOARMBackend(RobotBackend):
    """Backend for LeRobot SO-ARM100"""
    
    def __init__(self, device_id: str = "/dev/ttyUSB0"):
        super().__init__()
        self.device_id = device_id
        self.robot = None
        
    def connect(self) -> bool:
        """Connect to SO-ARM100"""
        try:
            # Initialize SO-ARM100 SDK
            # self.robot = SOARM100(self.device_id)
            # self.robot.enable()
            print(f"[SO-ARM100] Connected to {self.device_id}")
            return True
        except Exception as e:
            print(f"[SO-ARM100] Connection failed: {e}")
            return False
    
    def send_target_pose(self, position: np.ndarray, orientation: np.ndarray, 
                         velocity_limit: float = 0.1) -> bool:
        """Send target pose to SO-ARM100"""
        if not self.is_connected():
            return False
        
        try:
            # Convert to SO-ARM100 format and send
            # self.robot.move_to_pose(position, orientation, velocity_limit)
            return True
        except Exception as e:
            print(f"[SO-ARM100] Command failed: {e}")
            return False
```

**Tasks:**
- [ ] Research SO-ARM100 control interface (Python SDK? ROS?)
- [ ] Implement SOARMBackend class
- [ ] Test basic movements (home, calibration, simple trajectory)
- [ ] Add safety limits specific to SO-ARM100 workspace
- [ ] Test emergency stop functionality

#### 2C. System Integration
**Tasks:**
- [ ] Connect RealSense to ROG PC (USB 3.0)
- [ ] Mount RealSense on robot (end-effector or fixed position)
- [ ] Configure video encoding (H.264, target: <50ms latency)
- [ ] Test end-to-end: Web UI → Server → SO-ARM100
- [ ] Measure actual USA → Mexico latency

---

### **Phase 3: VR Interface (Week 5-6)**

#### 3A. VR Headset Setup
**Choose**: Meta Quest 3 OR Pico 4 Enterprise

**For Meta Quest 3:**
- Use Meta Quest Developer Hub
- Deploy via App Lab or SideQuest
- WebXR for browser-based VR (easier) OR native Unity/Unreal app

**For Pico 4 Enterprise:**
- Pico Developer Platform
- Enterprise deployment options
- Better for managed fleet if scaling to multiple operators

#### 3B. VR Application Development

**Option 1: WebXR (Recommended for MVP)**
- Fastest to implement
- Uses your existing web stack
- Browser-based VR (works in Quest Browser)

**Option 2: Native VR App (Better Performance)**
- Unity + Meta XR SDK / Pico SDK
- Custom 3D UI
- Better performance, more immersive

**Features to Implement:**
```
VR Features (from your roadmap):
✅ Stereo 3D video display
✅ 6DOF head tracking
✅ VR controller input → robot commands
✅ In-VR status UI (battery, connection, safety)
✅ VR comfort settings (FOV, motion smoothing)
✅ Emergency stop button (both controllers)
```

#### 3C. Stereo Video for VR
**Challenge**: RealSense D455 is mono camera, VR needs stereo

**Solutions:**
1. **Synthetic Stereo** (Recommended):
   - Use depth data from RealSense
   - Generate left/right eye views synthetically
   - Libraries: Open3D, or custom shader

2. **Dual Cameras**:
   - Buy second RealSense D455
   - Mount side-by-side for true stereo
   - More expensive but better immersion

---

## 🎯 Priority Order (Immediate Next Steps)

### This Week (Pre-Hardware):
1. ✅ **Complete Web UI** - Make it fully functional
2. ✅ **Add video streaming** - Test with Isaac Sim first
3. ✅ **Session recording** - Save video + commands for review
4. ✅ **Deploy to cloud** - Test Mexico → USA latency

### When Hardware Arrives (Week 3):
1. **RealSense Setup** - Get video working (2-3 days)
2. **SO-ARM100 Integration** - Basic control (3-5 days)
3. **End-to-End Testing** - Web UI → Robot (2-3 days)

### VR Development (Week 5+):
1. **VR Platform Choice** - Quest 3 vs Pico 4
2. **WebXR Prototype** - Browser-based VR (1 week)
3. **VR-Specific Features** - Comfort, UI, stereo video (1-2 weeks)

---

## 🚨 Key Questions to Answer Before Hardware Arrives

1. **SO-ARM100 Control Interface**:
   - Does LeRobot provide Python SDK?
   - ROS integration available?
   - What's the control frequency supported?

2. **RealSense Mounting**:
   - End-effector mount (moves with robot)?
   - Fixed mount (third-person view)?
   - Need custom 3D-printed bracket?

3. **Network Setup**:
   - Direct internet or VPN between USA-Mexico?
   - Will ROG PC have static IP or use dynamic DNS?
   - Firewall rules needed?

4. **VR Headset Choice**:
   - Who will manage the VR headsets (IT support)?
   - Meta Quest 3 (easier for developers) or Pico 4 Enterprise (better for business)?

---

## 📊 Success Metrics

### Phase 1 (Web UI):
- [ ] <100ms end-to-end latency (command → robot response)
- [ ] 30fps video streaming
- [ ] 20Hz control frequency
- [ ] 0 critical failures in 1-hour session

### Phase 2 (Hardware):
- [ ] SO-ARM100 responds to commands within 50ms
- [ ] RealSense streams at 60fps+ (target 90fps)
- [ ] No collisions during 10+ test sessions
- [ ] Emergency stop works 100% of time

### Phase 3 (VR):
- [ ] No motion sickness in 15+ minute sessions
- [ ] VR video latency <80ms (motion-to-photon)
- [ ] Comfortable for 30+ minute operations
- [ ] Stereo 3D provides depth perception

---

## 💡 Recommendations

### 1. Start Simple, Add Complexity
- Get web UI working first (no VR distraction)
- Prove basic teleop with one camera
- Add VR as "premium mode" later

### 2. Hardware Priorities
- **RealSense D455**: Critical for vision (buy ASAP if not ordered)
- **VR Headset**: Can wait until after basic teleop works
- **Second Camera**: Optional, add if operators need third-person view

### 3. Architecture Decision
- Build backend to support BOTH web and VR from start
- Same WebSocket protocol for both interfaces
- Different frontends (HTML vs VR app) but same server logic

### 4. Testing Strategy
Before Mexico operators:
- Test in same room (LAN)
- Test in same city (WAN)
- Then test USA → Mexico
- Measure latency at each step

---

## 🔧 Next Steps for You

### Immediate (Today):
1. Finish the web UI I started for you
2. Add video streaming endpoint to server
3. Test with Isaac Sim camera

### This Week:
1. Deploy server to cloud (AWS/Azure with public IP)
2. Test latency from different locations
3. Show demo to Chris

### When Hardware Ships:
1. Research SO-ARM100 SDK/documentation
2. Prepare RealSense integration code
3. Plan physical setup (desk, mounting, cables)

**Want me to help you with any specific part?** I can:
- Complete the web UI JavaScript
- Add RealSense streaming code
- Create SO-ARM100 backend skeleton
- Set up video encoding pipeline
