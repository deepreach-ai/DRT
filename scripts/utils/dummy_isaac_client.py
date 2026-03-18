import socket
import json
import time
import threading
import argparse
import sys
import numpy as np

class DummyIsaacClient:
    def __init__(self, host='localhost', port=9000):
        self.host = host
        self.port = port
        self.sock = None
        self.running = False
        self.current_position = [0.0, 0.0, 0.5]
        self.current_orientation = [1.0, 0.0, 0.0, 0.0]
        
    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            print(f"Connected to {self.host}:{self.port}")
            self.running = True
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
            
    def receive_loop(self):
        buffer = ""
        while self.running:
            try:
                data = self.sock.recv(4096)
                if not data:
                    print("Server disconnected")
                    self.running = False
                    break
                    
                buffer += data.decode('utf-8')
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        self.process_message(line)
            except Exception as e:
                print(f"Receive error: {e}")
                self.running = False
                break

    def process_message(self, message):
        try:
            data = json.loads(message)
            if data.get('type') == 'command':
                payload = data.get('payload', {})
                target_pos = payload.get('target_position')
                target_orient = payload.get('target_orientation')
                
                print(f"Received command: Pos={target_pos}, Orient={target_orient}")
                
                # Update "simulated" state
                if target_pos:
                    self.current_position = target_pos
                if target_orient:
                    self.current_orientation = target_orient
                    
        except json.JSONDecodeError:
            print(f"Invalid JSON: {message}")

    def send_state_loop(self):
        while self.running:
            try:
                message = {
                    "type": "state",
                    "payload": {
                        "position": self.current_position,
                        "orientation": self.current_orientation,
                        "velocity": [0.0, 0.0, 0.0],
                        "angular_velocity": [0.0, 0.0, 0.0]
                    }
                }
                self.sock.sendall((json.dumps(message) + "\n").encode('utf-8'))
                time.sleep(0.1)  # 10Hz update rate
            except Exception as e:
                print(f"Send error: {e}")
                self.running = False
                break

    def run(self):
        if not self.connect():
            return
            
        receive_thread = threading.Thread(target=self.receive_loop)
        send_thread = threading.Thread(target=self.send_state_loop)
        
        receive_thread.start()
        send_thread.start()
        
        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Stopping...")
            self.running = False
            self.sock.close()
            
        receive_thread.join()
        send_thread.join()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Dummy Isaac Sim Client")
    parser.add_argument("--host", default="localhost", help="Server host")
    parser.add_argument("--port", type=int, default=9000, help="Server port")
    args = parser.parse_args()
    
    client = DummyIsaacClient(args.host, args.port)
    client.run()
