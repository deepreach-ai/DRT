#!/usr/bin/env python3
"""
Quick VR Test Server - Test VR video streaming without modifying main server
"""
import sys
import os

# Add server directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'server'))

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
import uvicorn
import time
import cv2
import numpy as np

# Create mock backend inline (simpler for testing)
class QuickMockBackend:
    def __init__(self):
        self.frame_count = 0
        self.position = [0.0, 0.0, 0.0]
    
    def is_connected(self):
        return True
    
    def render_camera(self, name):
        """Generate mock camera frame"""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Different color for each camera
        colors = {
            'left': (80, 80, 200),
            'right': (80, 200, 80),
            'depth': (200, 80, 80)
        }
        base_color = colors.get(name, (128, 128, 128))
        
        # Fill with color
        frame[:, :] = base_color
        
        # Add text
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, f"{name.upper()} CAMERA (MOCK)", 
                   (100, 100), font, 1, (255, 255, 255), 2)
        cv2.putText(frame, f"Frame: {self.frame_count}", 
                   (20, 450), font, 0.7, (255, 255, 255), 2)
        
        # Animated dot
        dot_x = int((self.frame_count * 3) % 640)
        cv2.circle(frame, (dot_x, 240), 8, (255, 255, 0), -1)
        
        self.frame_count += 1
        
        # Encode to JPEG
        _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return jpeg.tobytes()
    
    def render_stereo_frame(self):
        """Generate side-by-side stereo"""
        left = cv2.imdecode(np.frombuffer(self.render_camera('left'), np.uint8), 1)
        right = cv2.imdecode(np.frombuffer(self.render_camera('right'), np.uint8), 1)
        stereo = np.hstack([left, right])
        _, jpeg = cv2.imencode('.jpg', stereo, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return jpeg.tobytes()

# Create app and backend
app = FastAPI(title="Quick VR Test")
backend = QuickMockBackend()

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Routes
@app.get("/")
async def root():
    return {
        "status": "Quick VR Test Server",
        "endpoints": [
            "/api/v1/video/left/mjpeg",
            "/api/v1/video/right/mjpeg",
            "/api/v1/video/depth/mjpeg",
            "/api/v1/video/stereo/mjpeg"
        ]
    }

def make_stream(camera_name):
    def generate():
        while True:
            frame = backend.render_camera(camera_name)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.033)
    return generate

@app.get("/api/v1/video/left/mjpeg")
async def video_left():
    return StreamingResponse(make_stream('left')(), 
                           media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/api/v1/video/right/mjpeg")
async def video_right():
    return StreamingResponse(make_stream('right')(), 
                           media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/api/v1/video/depth/mjpeg")
async def video_depth():
    return StreamingResponse(make_stream('depth')(), 
                           media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/api/v1/video/stereo/mjpeg")
async def video_stereo():
    def generate():
        while True:
            frame = backend.render_stereo_frame()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.033)
    return StreamingResponse(generate(), 
                           media_type="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    print("\n" + "="*60)
    print("🚀 Quick VR Test Server Starting...")
    print("="*60)
    print("\n📹 Video Endpoints:")
    print("   http://localhost:8000/api/v1/video/left/mjpeg")
    print("   http://localhost:8000/api/v1/video/right/mjpeg")
    print("   http://localhost:8000/api/v1/video/depth/mjpeg")
    print("   http://localhost:8000/api/v1/video/stereo/mjpeg")
    print("\n🌐 Also start web server:")
    print("   cd client/web && python3 -m http.server 8080")
    print("\n" + "="*60 + "\n")
    
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")
