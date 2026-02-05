#!/usr/bin/env python3
"""
Test Camera Server - Test dual webcam streaming
Run this AFTER applying DUAL_CAMERA_PATCH.txt to soarm_backend.py
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'server'))

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse, HTMLResponse
import uvicorn
import time
import cv2
import numpy as np

# Simple dual camera tester (no robot needed)
class DualCameraTester:
    def __init__(self, left_id=1, right_id=2):
        self.left_id = left_id
        self.right_id = right_id
        self.cameras = {}
        
    def connect(self):
        """Connect to both cameras"""
        print(f"🎥 Connecting to cameras...")
        
        # Left camera
        try:
            cap = cv2.VideoCapture(self.left_id)
            if cap.isOpened():
                ret, _ = cap.read()
                if ret:
                    self.cameras['left'] = cap
                    print(f"✅ Left camera (device {self.left_id}) connected")
                else:
                    cap.release()
                    print(f"⚠️  Left camera can't read frames")
            else:
                print(f"❌ Failed to open left camera (device {self.left_id})")
        except Exception as e:
            print(f"❌ Left camera error: {e}")
        
        # Right camera
        try:
            cap = cv2.VideoCapture(self.right_id)
            if cap.isOpened():
                ret, _ = cap.read()
                if ret:
                    self.cameras['right'] = cap
                    print(f"✅ Right camera (device {self.right_id}) connected")
                else:
                    cap.release()
                    print(f"⚠️  Right camera can't read frames")
            else:
                print(f"❌ Failed to open right camera (device {self.right_id})")
        except Exception as e:
            print(f"❌ Right camera error: {e}")
        
        if not self.cameras:
            print("\n❌ No cameras connected!")
            print("💡 Run: python3 detect_cameras.py")
            return False
        
        print(f"\n✅ Connected: {list(self.cameras.keys())}")
        return True
    
    def render_camera(self, name):
        """Render single camera"""
        cap = self.cameras.get(name)
        if not cap or not cap.isOpened():
            return None
        
        ret, frame = cap.read()
        if not ret:
            return None
        
        frame = cv2.resize(frame, (640, 480))
        _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return jpeg.tobytes()
    
    def render_stereo(self):
        """Render side-by-side stereo"""
        left = self.render_camera('left')
        right = self.render_camera('right')
        
        if not (left and right):
            return None
        
        left_img = cv2.imdecode(np.frombuffer(left, np.uint8), 1)
        right_img = cv2.imdecode(np.frombuffer(right, np.uint8), 1)
        stereo = np.hstack([left_img, right_img])
        _, jpeg = cv2.imencode('.jpg', stereo, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return jpeg.tobytes()
    
    def disconnect(self):
        """Disconnect cameras"""
        for name, cap in self.cameras.items():
            if cap:
                cap.release()
                print(f"✓ {name} camera released")

# Create app
app = FastAPI(title="Camera Test Server")

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global camera tester
camera_tester = None

@app.on_event("startup")
async def startup():
    global camera_tester
    # Get camera IDs from command line or use defaults
    import sys
    left_id = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    right_id = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    
    camera_tester = DualCameraTester(left_id, right_id)
    camera_tester.connect()

@app.on_event("shutdown")
async def shutdown():
    if camera_tester:
        camera_tester.disconnect()

@app.get("/")
async def root():
    return HTMLResponse("""
    <html>
    <head><title>Camera Test</title></head>
    <body style="background: #0b1220; color: #fff; font-family: sans-serif; padding: 20px;">
        <h1>🎥 Dual Camera Test Server</h1>
        <h2>Left Camera (Arm 1)</h2>
        <img src="/api/v1/video/left/mjpeg" style="width: 640px; border: 2px solid #4f8cff;">
        
        <h2>Right Camera (Arm 2)</h2>
        <img src="/api/v1/video/right/mjpeg" style="width: 640px; border: 2px solid #4f8cff;">
        
        <h2>Stereo (Side-by-Side)</h2>
        <img src="/api/v1/video/stereo/mjpeg" style="width: 1280px; border: 2px solid #4f8cff;">
        
        <p style="margin-top: 40px; color: #a9b7d0;">
            Individual endpoints:<br>
            • <a href="/api/v1/video/left/mjpeg" style="color: #4f8cff;">/api/v1/video/left/mjpeg</a><br>
            • <a href="/api/v1/video/right/mjpeg" style="color: #4f8cff;">/api/v1/video/right/mjpeg</a><br>
            • <a href="/api/v1/video/stereo/mjpeg" style="color: #4f8cff;">/api/v1/video/stereo/mjpeg</a>
        </p>
    </body>
    </html>
    """)

def make_stream(camera_name):
    def generate():
        while True:
            frame = camera_tester.render_camera(camera_name)
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.033)  # ~30fps
    return generate

@app.get("/api/v1/video/left/mjpeg")
async def video_left():
    return StreamingResponse(
        make_stream('left')(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@app.get("/api/v1/video/right/mjpeg")
async def video_right():
    return StreamingResponse(
        make_stream('right')(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@app.get("/api/v1/video/stereo/mjpeg")
async def video_stereo():
    def generate():
        while True:
            frame = camera_tester.render_stereo()
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.033)
    return StreamingResponse(
        generate(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

if __name__ == "__main__":
    print("\n" + "="*60)
    print("🎥 Dual Camera Test Server")
    print("="*60)
    print("\nUsage:")
    print("  python3 test_camera_server.py [left_id] [right_id]")
    print("\nExamples:")
    print("  python3 test_camera_server.py          # Use default IDs (1, 2)")
    print("  python3 test_camera_server.py 1 2      # Explicitly set IDs")
    print("\n💡 First run: python3 detect_cameras.py to find your camera IDs")
    print("\n" + "="*60 + "\n")
    
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")
