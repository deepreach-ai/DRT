import argparse
import sys
import time
from typing import Optional, Tuple

class HandPose:
    def __init__(self, position=None, orientation=None, pinch_strength=0.0):
        self.position = position or [0, 0, 0]
        self.orientation = orientation or [0, 0, 0, 1]
        self.pinch_strength = pinch_strength

class Quest3Controller:
    def __init__(self, use_ngrok: bool = False, port: int = 8012):
        self.use_ngrok = use_ngrok
        self.port = port
        self.resolution = (1920, 1080)
        self.control_scale = 1.0
        self.deadman_threshold = 0.8
        self.running = False
        
    def start(self):
        print(f"🎮 VR Controller Server Running on port {self.port}!")
        if self.use_ngrok:
            self._start_ngrok()
        else:
            print(f"📱 Open on Quest 3: http://localhost:{self.port}/vr.html")
            
    def _start_ngrok(self):
        try:
            from pyngrok import ngrok
            public_url = ngrok.connect(self.port).public_url
            print(f"🌐 ngrok tunnel established!")
            print(f"   Public URL: {public_url}")
            print(f"   Use this URL on Quest 3")
        except ImportError:
            print("❌ pyngrok not installed. Install with: pip install pyngrok")
        except Exception as e:
            print(f"❌ Failed to start ngrok: {e}")

def main():
    parser = argparse.ArgumentParser(description="Quest 3 VR Controller Server")
    parser.add_argument("--ngrok", action="store_true", help="Expose via ngrok")
    parser.add_argument("--port", type=int, default=8012, help="Port to serve VR interface")
    args = parser.parse_args()
    
    controller = Quest3Controller(use_ngrok=args.ngrok, port=args.port)
    controller.start()
    
    try:
        # Keep alive
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping VR Controller...")

if __name__ == "__main__":
    main()
