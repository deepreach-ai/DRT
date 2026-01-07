#!/usr/bin/env python3
"""
Latency measurement script for Teleoperation System.
Tests:
1. ICMP Ping (Network Layer)
2. HTTP REST Latency (Application Layer - Status)
3. WebSocket Command Latency (Application Layer - Real-time Control)
"""

import argparse
import time
import statistics
import subprocess
import sys
import json
import asyncio
import platform
from typing import List

# Try importing requests for HTTP tests
try:
    import requests
except ImportError:
    print("Error: 'requests' module not found. Please install it: pip install requests")
    sys.exit(1)

# Try importing websockets for WS tests
try:
    import websockets
except ImportError:
    print("Error: 'websockets' module not found. Please install it: pip install websockets")
    sys.exit(1)


def measure_ping(host: str, count: int = 5) -> List[float]:
    """Measure ICMP ping latency."""
    print(f"\n[1/3] Measuring Network Latency (Ping) to {host}...")
    latencies = []
    
    param = '-n' if platform.system().lower() == 'windows' else '-c'
    # Set timeout to 1 second to avoid hanging on blocked ICMP
    timeout_param = ['-w', '1000'] if platform.system().lower() == 'windows' else ['-W', '1000']
    command = ['ping', param, '1'] + timeout_param + [host]
    
    for _ in range(count):
        try:
            # Run ping command
            if platform.system().lower() == 'windows':
                # Windows ping output parsing is different, simplified for now
                output = subprocess.check_output(command).decode()
                if "time=" in output:
                    time_ms = float(output.split("time=")[1].split("ms")[0])
                    latencies.append(time_ms)
            else:
                # Unix ping
                output = subprocess.check_output(command).decode()
                if "time=" in output:
                    time_ms = float(output.split("time=")[1].split(" ")[0])
                    latencies.append(time_ms)
            time.sleep(0.5)
        except subprocess.CalledProcessError:
            print("  Ping failed (timeout or unreachable)")
        except Exception as e:
            print(f"  Ping error: {e}")

    return latencies


def measure_http(url: str, count: int = 10) -> List[float]:
    """Measure HTTP GET latency."""
    print(f"\n[2/3] Measuring HTTP Latency (GET {url})...")
    latencies = []
    
    session = requests.Session()
    
    for i in range(count):
        try:
            start = time.perf_counter()
            response = session.get(url, timeout=5)
            end = time.perf_counter()
            
            if response.status_code == 200:
                latencies.append((end - start) * 1000)  # Convert to ms
            else:
                print(f"  HTTP Error: {response.status_code}")
        except Exception as e:
            print(f"  Request failed: {e}")
            
    return latencies


async def measure_websocket(ws_url: str, count: int = 20) -> List[float]:
    """Measure WebSocket Command Round-Trip Latency."""
    print(f"\n[3/3] Measuring WebSocket Control Latency ({ws_url})...")
    latencies = []
    
    # Zero-movement command
    command = {
        "dx": 0.0, "dy": 0.0, "dz": 0.0,
        "droll": 0.0, "dpitch": 0.0, "dyaw": 0.0,
        "max_velocity": 0.1,
        "timestamp": 0.0
    }
    
    try:
        async with websockets.connect(ws_url) as websocket:
            # Authenticate if needed (skipping for now as default is open or handles anon)
            
            print(f"  Connected. Sending {count} commands...")
            
            for i in range(count):
                command["timestamp"] = time.time()
                
                start = time.perf_counter()
                await websocket.send(json.dumps(command))
                
                # Wait for ACK
                while True:
                    try:
                        response = await asyncio.wait_for(websocket.recv(), timeout=2.0)
                        data = json.loads(response)
                        if data.get("type") == "ack":
                            end = time.perf_counter()
                            latencies.append((end - start) * 1000)
                            break
                    except asyncio.TimeoutError:
                        print("  Timeout waiting for ACK")
                        break
                
                # Small delay between commands
                await asyncio.sleep(0.1)
                
    except Exception as e:
        print(f"  WebSocket Error: {e}")
        
    return latencies


def print_stats(name: str, data: List[float]):
    if not data:
        print(f"  {name}: No data")
        return
        
    avg = statistics.mean(data)
    min_val = min(data)
    max_val = max(data)
    std_dev = statistics.stdev(data) if len(data) > 1 else 0.0
    
    print(f"  {name} Stats (ms):")
    print(f"    Average: {avg:.2f} ms")
    print(f"    Min:     {min_val:.2f} ms")
    print(f"    Max:     {max_val:.2f} ms")
    print(f"    Jitter:  {std_dev:.2f} ms")


def main():
    parser = argparse.ArgumentParser(description="Test Teleoperation Latency")
    parser.add_argument("--host", default="localhost", help="Server host (e.g., 54.x.x.x)")
    parser.add_argument("--port", type=int, default=8000, help="Server port")
    args = parser.parse_args()
    
    host = args.host
    port = args.port
    base_url = f"http://{host}:{port}"
    http_url = f"{base_url}/api/v1/status"
    
    # 1. Ping
    ping_data = measure_ping(host)
    print_stats("Ping", ping_data)
    
    # 2. HTTP
    http_data = measure_http(http_url)
    print_stats("HTTP", http_data)
    
    # 3. WebSocket
    # Attempt login first
    print("\n[Auth] Logging in as 'operator'...")
    token = None
    try:
        resp = requests.post(f"{base_url}/api/v1/auth/login", 
                           json={"username": "operator", "password": "operator"},
                           timeout=5)
        if resp.status_code == 200:
            token = resp.json().get("token")
            print("  Login successful.")
        else:
            print(f"  Login failed: {resp.status_code} {resp.text}")
            print("  Attempting connection without token (may fail if auth enabled)...")
    except Exception as e:
        print(f"  Login error: {e}")
        print("  Attempting connection without token...")

    ws_url = f"ws://{host}:{port}/ws/v1/teleop"
    if token:
        ws_url += f"?token={token}"
        
    ws_data = asyncio.run(measure_websocket(ws_url))
    print_stats("WebSocket Control", ws_data)
    
    print("\n" + "="*50)
    print("SUMMARY")
    print("="*50)
    if ping_data: print(f"Network RTT:  {statistics.mean(ping_data):.2f} ms")
    if http_data: print(f"HTTP RTT:     {statistics.mean(http_data):.2f} ms")
    if ws_data:   print(f"Control Loop: {statistics.mean(ws_data):.2f} ms")
    print("="*50)

if __name__ == "__main__":
    main()
