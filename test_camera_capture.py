#!/usr/bin/env python3
"""
Test camera capture and save sample frames
"""
import cv2
import sys
import os

def test_camera(device_id, save_path="test_camera_frames"):
    """Test a specific camera and save a frame"""
    
    os.makedirs(save_path, exist_ok=True)
    
    print(f"\n🎥 Testing Camera {device_id}...")
    print("-" * 40)
    
    cap = cv2.VideoCapture(device_id)
    
    if not cap.isOpened():
        print(f"❌ Failed to open camera {device_id}")
        return False
    
    # Get camera properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    print(f"📊 Resolution: {width}x{height}")
    print(f"📊 FPS: {fps:.1f}")
    
    # Capture a frame
    print("📸 Capturing frame...")
    ret, frame = cap.read()
    
    if not ret:
        print("❌ Failed to capture frame")
        cap.release()
        return False
    
    # Save frame
    filename = f"{save_path}/camera_{device_id}_test.jpg"
    cv2.imwrite(filename, frame)
    print(f"✅ Frame saved: {filename}")
    
    # Try to capture 10 frames to test stability
    print("🔄 Testing stability (10 frames)...")
    success_count = 0
    for i in range(10):
        ret, frame = cap.read()
        if ret:
            success_count += 1
    
    print(f"✅ Captured {success_count}/10 frames successfully")
    
    cap.release()
    return True

if __name__ == "__main__":
    print("🎬 Camera Capture Test")
    print("=" * 60)
    
    # Test cameras 0, 1, 2 (adjust based on detect_cameras.py output)
    cameras_to_test = [0, 1, 2]
    
    if len(sys.argv) > 1:
        # Allow specifying camera IDs as arguments
        cameras_to_test = [int(x) for x in sys.argv[1:]]
    
    results = {}
    for cam_id in cameras_to_test:
        results[cam_id] = test_camera(cam_id)
    
    print("\n" + "=" * 60)
    print("📊 Summary:")
    for cam_id, success in results.items():
        status = "✅ Working" if success else "❌ Failed"
        print(f"   Camera {cam_id}: {status}")
    
    working_cams = [k for k, v in results.items() if v]
    
    if len(working_cams) >= 2:
        print(f"\n🎉 Success! {len(working_cams)} cameras working")
        print(f"\n💡 Recommended assignment:")
        print(f"   Camera {working_cams[0]}: Built-in (skip or use as depth)")
        if len(working_cams) >= 2:
            print(f"   Camera {working_cams[1]}: LEFT ARM")
        if len(working_cams) >= 3:
            print(f"   Camera {working_cams[2]}: RIGHT ARM")
        
        print(f"\n🔧 Next: Update soarm_backend.py with these camera IDs")
        print(f"📂 Test frames saved in: test_camera_frames/")
        print(f"   Open them to verify camera angles!")
    
    elif len(working_cams) == 1:
        print(f"\n⚠️  Only {len(working_cams)} camera working")
        print("   Make sure both arm webcams are plugged in!")
    
    else:
        print("\n❌ No working cameras found")
        print("   Check USB connections and permissions")
