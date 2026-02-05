
import cv2
import sys

def test_opencv_cam():
    print("Testing OpenCV camera access...")
    # Try indices 0 to 5
    for i in range(6):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"Camera {i} is available")
            ret, frame = cap.read()
            if ret:
                print(f"  - Read frame: {frame.shape}")
            else:
                print(f"  - Could not read frame")
            cap.release()
        else:
            print(f"Camera {i} is NOT available")

if __name__ == "__main__":
    test_opencv_cam()
