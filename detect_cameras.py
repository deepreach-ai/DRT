#!/usr/bin/env python3
"""
Detect and test all connected cameras
"""
import cv2
import sys

print("🔍 Scanning for cameras...")
print("=" * 60)

found_cameras = []

for i in range(10):  # Check first 10 device IDs
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        # Try to read a frame
        ret, frame = cap.read()
        if ret:
            height, width = frame.shape[:2]
            found_cameras.append({
                'id': i,
                'width': width,
                'height': height,
                'fps': cap.get(cv2.CAP_PROP_FPS)
            })
            print(f"✅ Camera {i}: {width}x{height} @ {cap.get(cv2.CAP_PROP_FPS):.0f}fps")
        else:
            print(f"⚠️  Camera {i}: Opens but can't read frames")
        cap.release()

print("=" * 60)
print(f"\n📊 Summary: Found {len(found_cameras)} working camera(s)")

if len(found_cameras) == 0:
    print("\n❌ No cameras detected!")
    print("\n💡 Troubleshooting:")
    print("   1. Check if webcams are plugged in via USB")
    print("   2. Try: ls /dev/video* (Linux) or system_profiler SPCameraDataType (Mac)")
    print("   3. Grant camera permissions if prompted")
    sys.exit(1)

elif len(found_cameras) == 1:
    print("\n⚠️  Only 1 camera detected")
    print("   Expected: 2 webcams (one per follower arm)")
    print("   Detected: 1 camera (might be built-in Mac webcam)")
    print("\n💡 Next steps:")
    print("   1. Plug in both external USB webcams")
    print("   2. Run this script again")

elif len(found_cameras) >= 2:
    print("\n✅ Great! Multiple cameras detected")
    print("\n📝 Camera assignments:")
    print(f"   Device {found_cameras[0]['id']}: Likely built-in Mac webcam")
    print(f"   Device {found_cameras[1]['id']}: External webcam 1 → LEFT ARM")
    if len(found_cameras) >= 3:
        print(f"   Device {found_cameras[2]['id']}: External webcam 2 → RIGHT ARM")
    
    print("\n🎬 Test camera capture:")
    print("   python3 test_camera_capture.py")

print()
