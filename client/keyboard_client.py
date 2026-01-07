"""
Simple keyboard client for teleoperation
"""
import requests
import time
import json
import threading
from enum import Enum
import sys
import os
import tty
import termios
import select

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from server.models import DeltaCommand, ReferenceFrame


class Getch:
    """Gets a single character from standard input. Does not echo to the screen."""
    def __call__(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class ControlMode(Enum):
    POSITION = "position"
    ORIENTATION = "orientation"


class KeyboardTeleopClient:
    """Keyboard-based teleoperation client"""
    
    def __init__(self, server_url: str = "http://localhost:8000"):
        self.server_url = server_url
        self.running = False
        self.control_mode = ControlMode.POSITION
        self.sender_thread = None
        self.send_frequency = 20.0  # Hz
        
        # Control increments
        self.position_increment = 0.02  # meters
        self.orientation_increment = 0.05  # radians
        
        # Current accumulated command values (thread-safe access needed)
        self.command_lock = threading.Lock()
        self.dx = 0.0
        self.dy = 0.0
        self.dz = 0.0
        self.droll = 0.0
        self.dpitch = 0.0
        self.dyaw = 0.0
        
        self.getch = Getch()
        
        # Key mapping
        self.key_actions = {
            'w': ('dz', self.position_increment),
            's': ('dz', -self.position_increment),
            'a': ('dx', -self.position_increment),
            'd': ('dx', self.position_increment),
            'q': ('dy', self.position_increment),
            'e': ('dy', -self.position_increment),
            
            'i': ('dpitch', -self.orientation_increment),
            'k': ('dpitch', self.orientation_increment),
            'j': ('dyaw', self.orientation_increment),
            'l': ('dyaw', -self.orientation_increment),
            'u': ('droll', -self.orientation_increment),
            'o': ('droll', self.orientation_increment),
            
            'm': self.toggle_mode,
            'r': self.reset_command,
            '1': self.activate_safety,    # New: manually activate safety
        }

    def start_sender_loop(self):
        """Start background thread to send commands at fixed frequency"""
        self.running = True
        
        def sender_loop():
            print(f"Sender loop started at {self.send_frequency} Hz")
            interval = 1.0 / self.send_frequency
            
            while self.running:
                start_time = time.time()
                
                # Prepare command
                with self.command_lock:
                    command = DeltaCommand(
                        dx=self.dx, dy=self.dy, dz=self.dz,
                        droll=self.droll, dpitch=self.dpitch, dyaw=self.dyaw,
                        reference_frame=ReferenceFrame.END_EFFECTOR,
                        max_velocity=0.5, # Allow higher max since we send frequent small updates
                        max_angular_velocity=1.0,
                        timestamp=time.time(),
                        client_id="keyboard_client"
                    )
                    
                    # Reset accumulators
                    self.dx = 0.0
                    self.dy = 0.0
                    self.dz = 0.0
                    self.droll = 0.0
                    self.dpitch = 0.0
                    self.dyaw = 0.0
                
                # Send command (even if zero - acts as keep-alive)
                try:
                    self.send_command(command)
                except Exception as e:
                    pass
                
                # Sleep remainder
                elapsed = time.time() - start_time
                sleep_time = max(0, interval - elapsed)
                time.sleep(sleep_time)
        
        self.sender_thread = threading.Thread(target=sender_loop, daemon=True)
        self.sender_thread.start()

    def stop_sender_loop(self):
        self.running = False
        if self.sender_thread:
            self.sender_thread.join(timeout=1.0)

    def toggle_mode(self):
        """Toggle between position and orientation control"""
        if self.control_mode == ControlMode.POSITION:
            self.control_mode = ControlMode.ORIENTATION
            print("Switched to ORIENTATION control mode")
        else:
            self.control_mode = ControlMode.POSITION
            print("Switched to POSITION control mode")
    
    def reset_command(self):
        """Reset all command values to zero"""
        with self.command_lock:
            self.dx = self.dy = self.dz = 0.0
            self.droll = self.dpitch = self.dyaw = 0.0
        print("Command reset to zero")
    
    def activate_safety(self):
        """Manually activate safety gate"""
        try:
            response = requests.post(
                f"{self.server_url}/api/v1/safety/activate",
                timeout=1
            )
            if response.status_code == 200:
                print("✓ Safety gate manually activated")
            else:
                print(f"Failed to activate safety: {response.status_code}")
        except Exception as e:
            print(f"Error activating safety: {e}")
    
    def send_command(self, command: DeltaCommand):
        """Send command to server"""
        try:
            # Use model_dump() instead of dict() for Pydantic v2
            response = requests.post(
                f"{self.server_url}/api/v1/command",
                json=command.model_dump(),
                timeout=0.5
            )
            
            if response.status_code == 200:
                result = response.json()
                if result.get('violations', {}):
                    for violation, active in result['violations'].items():
                        if active:
                            # We might want to suppress this if it spams too much at 20Hz
                            # But for now it's useful feedback
                            print(f"\r⚠️  {violation.upper()} VIOLATION!   ", end="", flush=True)
                return True
            else:
                # print(f"Server error {response.status_code}: {response.text}")
                return False
                
        except requests.exceptions.ConnectionError:
            # print("✗ Cannot connect to server. Is it running?")
            return False
        except requests.exceptions.RequestException as e:
            # print(f"Connection error: {e}")
            return False
    
    def handle_key(self, key: str):
        """Handle a key press"""
        if key in self.key_actions:
            action = self.key_actions[key]
            
            if callable(action):
                action()
            else:
                attr, value = action
                
                with self.command_lock:
                    # Apply based on control mode
                    if self.control_mode == ControlMode.POSITION:
                        if attr.startswith('d') and not attr.startswith('dr') and not attr.startswith('dp') and not attr.startswith('dy'):
                            current_value = getattr(self, attr)
                            setattr(self, attr, current_value + value)
                    else:  # orientation mode
                        if attr.startswith('dr') or attr.startswith('dp') or attr.startswith('dy'):
                            current_value = getattr(self, attr)
                            setattr(self, attr, current_value + value)

    def print_help(self):
        """Print control help"""
        print("\n" + "="*50)
        print("TELEOPERATION KEYBOARD CLIENT")
        print("="*50)
        print("\nControl Mode: {}".format(self.control_mode.value.upper()))
        print("\nPOSITION CONTROLS:")
        print("  w/s: Move forward/back (dz)")
        print("  a/d: Move left/right (dx)")
        print("  q/e: Move up/down (dy)")
        print("\nORIENTATION CONTROLS:")
        print("  i/k: Pitch down/up")
        print("  j/l: Yaw left/right")
        print("  u/o: Roll left/right")
        print("\nOTHER CONTROLS:")
        print("  m: Toggle position/orientation mode")
        print("  r: Reset command to zero")
        print("  1: Manually activate safety gate")
        print("  CTRL+C: Exit")
        print("\nServer URL:", self.server_url)
        print("="*50 + "\n")
    
    def run_interactive(self):
        """Run interactive keyboard client"""
        self.running = True
        
        # Check server connection first
        if not self.test_connection():
            print(f"✗ Cannot connect to server at {self.server_url}")
            return
        
        print("✓ Connected to server")
        self.print_help()
        
        # Initial safety gate activation
        print("\nActivating safety gate...")
        self.activate_safety()
        
        # Start sender loop
        self.start_sender_loop()
        
        print(f"\nReady for input. Press keys to move. Ctrl+C to exit.")
        
        try:
            while self.running:
                # Blocking read of single character
                char = self.getch()
                
                # Handle special keys
                if char == '\x03':  # Ctrl+C
                    raise KeyboardInterrupt
                
                # Map to lower case
                key = char.lower()
                
                if key == 'h':
                    self.print_help()
                else:
                    self.handle_key(key)
                    
        except KeyboardInterrupt:
            print("\n\nExiting...")
        except Exception as e:
            print(f"Error: {e}")
        finally:
            self.stop_sender_loop()
    
    def test_connection(self) -> bool:
        """Test connection to server"""
        try:
            print(f"DEBUG: Connecting to {self.server_url}/ ...")
            response = requests.get(f"{self.server_url}/", timeout=5)
            print(f"DEBUG: Status code: {response.status_code}")
            return response.status_code == 200
        except Exception as e:
            print(f"DEBUG: Connection error: {e}")
            return False


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Keyboard Teleoperation Client")
    parser.add_argument("--server", default="http://localhost:8000",
                       help="Server URL (default: http://localhost:8000)")
    parser.add_argument("--test", action="store_true",
                       help="Test connection and exit")
    
    args = parser.parse_args()
    
    if args.test:
        print(f"Testing connection to {args.server}...")
        client = KeyboardTeleopClient(server_url=args.server)
        if client.test_connection():
            print("✓ Connection successful!")
            # Test safety activation too
            client.activate_safety()
        else:
            print("✗ Connection failed!")
        sys.exit(0)
    
    client = KeyboardTeleopClient(server_url=args.server)
    client.run_interactive()
