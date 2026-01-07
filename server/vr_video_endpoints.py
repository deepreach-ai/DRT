"""
VR Video Endpoints for Multi-Camera Streaming
Add these routes to your teleop_server.py
"""

# Add to teleop_server.py after existing imports:
# from vr_video_endpoints import setup_vr_video_routes

def setup_vr_video_routes(app, server):
    """
    Setup VR-specific video streaming endpoints
    Supports 3-camera setup: left webcam, right webcam, depth camera
    """
    
    @app.get("/api/v1/video/left/mjpeg")
    async def video_left_stream():
        """
        Left camera stream (Arm 1 webcam) for VR left eye
        """
        def generate():
            import time
            while True:
                if server.backend and server.backend.is_connected():
                    # Try to get left camera frame
                    frame = server.backend.render_camera('left')
                    if frame:
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                    else:
                        # Fallback: render status frame
                        fallback = render_status_frame("Left Camera Offline", 640, 480)
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + fallback + b'\r\n')
                else:
                    fallback = render_status_frame("Robot Disconnected", 640, 480)
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + fallback + b'\r\n')
                
                time.sleep(0.033)  # ~30fps
        
        return StreamingResponse(
            generate(),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={
                "Cache-Control": "no-cache, no-store, must-revalidate",
                "Pragma": "no-cache",
                "Expires": "0"
            }
        )
    
    @app.get("/api/v1/video/right/mjpeg")
    async def video_right_stream():
        """
        Right camera stream (Arm 2 webcam) for VR right eye
        """
        def generate():
            import time
            while True:
                if server.backend and server.backend.is_connected():
                    frame = server.backend.render_camera('right')
                    if frame:
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                    else:
                        fallback = render_status_frame("Right Camera Offline", 640, 480)
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + fallback + b'\r\n')
                else:
                    fallback = render_status_frame("Robot Disconnected", 640, 480)
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + fallback + b'\r\n')
                
                time.sleep(0.033)
        
        return StreamingResponse(
            generate(),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={
                "Cache-Control": "no-cache, no-store, must-revalidate",
                "Pragma": "no-cache",
                "Expires": "0"
            }
        )
    
    @app.get("/api/v1/video/depth/mjpeg")
    async def video_depth_stream():
        """
        Depth camera stream (RealSense D435) for spatial awareness
        """
        def generate():
            import time
            while True:
                if server.backend and server.backend.is_connected():
                    frame = server.backend.render_camera('depth')
                    if frame:
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                    else:
                        fallback = render_status_frame("Depth Camera Offline", 640, 480)
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + fallback + b'\r\n')
                else:
                    fallback = render_status_frame("Robot Disconnected", 640, 480)
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + fallback + b'\r\n')
                
                time.sleep(0.033)
        
        return StreamingResponse(
            generate(),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={
                "Cache-Control": "no-cache, no-store, must-revalidate",
                "Pragma": "no-cache",
                "Expires": "0"
            }
        )
    
    @app.get("/api/v1/video/stereo/mjpeg")
    async def video_stereo_stream():
        """
        Side-by-side stereo stream combining left and right cameras
        """
        def generate():
            import time
            while True:
                if server.backend and server.backend.is_connected():
                    # Get stereo frame (left + right side-by-side)
                    frame = server.backend.render_stereo_frame()
                    if frame:
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                    else:
                        fallback = render_status_frame("Stereo Cameras Offline", 1280, 480)
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + fallback + b'\r\n')
                else:
                    fallback = render_status_frame("Robot Disconnected", 1280, 480)
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + fallback + b'\r\n')
                
                time.sleep(0.033)
        
        return StreamingResponse(
            generate(),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={
                "Cache-Control": "no-cache, no-store, must-revalidate",
                "Pragma": "no-cache",
                "Expires": "0"
            }
        )


# Helper function to generate status frames
def render_status_frame(message: str, width: int = 640, height: int = 480) -> bytes:
    """
    Render a status message as a JPEG frame
    """
    try:
        import cv2
        import numpy as np
        
        # Create black background
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Add text
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_size = cv2.getTextSize(message, font, 1, 2)[0]
        text_x = (width - text_size[0]) // 2
        text_y = (height + text_size[1]) // 2
        
        cv2.putText(img, message, (text_x, text_y), font, 1, (255, 255, 255), 2)
        
        # Encode to JPEG
        _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        return jpeg.tobytes()
        
    except Exception as e:
        print(f"[VR] Error rendering status frame: {e}")
        return b''
