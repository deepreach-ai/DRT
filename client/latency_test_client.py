"""
Latency Testing Client for Teleoperation System

This script measures end-to-end latency from client to server and back.
Run this from Mexico (or simulated Mexico location via VPN) to test real-world latency.

Usage:
    python latency_test_client.py --server ws://your-server-ip:8000/ws/v1/teleop
    python latency_test_client.py --server ws://localhost:8000/ws/v1/teleop  # local test
"""

import asyncio
import websockets
import json
import time
import statistics
import argparse
from datetime import datetime
from typing import List, Dict
import sys

class LatencyTester:
    def __init__(self, server_url: str):
        self.server_url = server_url
        self.latencies: List[float] = []
        self.errors = 0
        self.successes = 0
        
    async def test_single_command(self) -> float:
        """Send a command and measure round-trip time"""
        command = {
            "dx": 0.0,
            "dy": 0.0,
            "dz": 0.0,
            "droll": 0.0,
            "dpitch": 0.0,
            "dyaw": 0.0,
            "reference_frame": "end_effector",
            "max_velocity": 0.5,
            "max_angular_velocity": 1.0,
            "timestamp": time.time(),
            "client_id": "latency_test"
        }
        
        start_time = time.time()
        
        try:
            async with websockets.connect(self.server_url, ping_interval=None) as ws:
                # Send command
                await ws.send(json.dumps(command))
                
                # Wait for response
                response = await asyncio.wait_for(ws.recv(), timeout=5.0)
                
                # Calculate latency
                end_time = time.time()
                latency_ms = (end_time - start_time) * 1000
                
                self.successes += 1
                return latency_ms
                
        except asyncio.TimeoutError:
            self.errors += 1
            print("⚠️  Timeout - no response from server")
            return -1
        except Exception as e:
            self.errors += 1
            print(f"⚠️  Error: {e}")
            return -1
    
    async def run_test(self, num_samples: int = 100, interval: float = 0.1):
        """Run multiple latency tests"""
        print(f"\n{'='*60}")
        print(f"🧪 LATENCY TEST")
        print(f"{'='*60}")
        print(f"Server: {self.server_url}")
        print(f"Samples: {num_samples}")
        print(f"Interval: {interval}s")
        print(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"{'='*60}\n")
        
        # Warmup
        print("Warming up... ", end="", flush=True)
        for _ in range(5):
            await self.test_single_command()
            await asyncio.sleep(0.1)
        print("✓")
        
        # Reset counters after warmup
        self.latencies = []
        self.errors = 0
        self.successes = 0
        
        # Run tests
        print(f"\nRunning {num_samples} tests...\n")
        
        for i in range(num_samples):
            latency = await self.test_single_command()
            
            if latency > 0:
                self.latencies.append(latency)
                
                # Print progress
                if (i + 1) % 10 == 0:
                    current_avg = statistics.mean(self.latencies)
                    print(f"Progress: {i+1}/{num_samples} | "
                          f"Current Avg: {current_avg:.1f}ms | "
                          f"Last: {latency:.1f}ms")
            
            await asyncio.sleep(interval)
        
        # Print results
        self.print_results()
    
    def print_results(self):
        """Print comprehensive test results"""
        print(f"\n{'='*60}")
        print(f"📊 TEST RESULTS")
        print(f"{'='*60}\n")
        
        if not self.latencies:
            print("❌ No successful measurements!")
            print(f"Errors: {self.errors}")
            return
        
        # Calculate statistics
        avg_latency = statistics.mean(self.latencies)
        median_latency = statistics.median(self.latencies)
        min_latency = min(self.latencies)
        max_latency = max(self.latencies)
        stdev = statistics.stdev(self.latencies) if len(self.latencies) > 1 else 0
        
        # Calculate percentiles
        sorted_latencies = sorted(self.latencies)
        p95_idx = int(len(sorted_latencies) * 0.95)
        p99_idx = int(len(sorted_latencies) * 0.99)
        p95 = sorted_latencies[p95_idx]
        p99 = sorted_latencies[p99_idx]
        
        # Success rate
        total_tests = self.successes + self.errors
        success_rate = (self.successes / total_tests * 100) if total_tests > 0 else 0
        
        # Print statistics
        print(f"Success Rate: {success_rate:.1f}% ({self.successes}/{total_tests})")
        print(f"Errors: {self.errors}")
        print(f"\nLatency Statistics (milliseconds):")
        print(f"  Average:    {avg_latency:.2f} ms")
        print(f"  Median:     {median_latency:.2f} ms")
        print(f"  Min:        {min_latency:.2f} ms")
        print(f"  Max:        {max_latency:.2f} ms")
        print(f"  Std Dev:    {stdev:.2f} ms")
        print(f"  P95:        {p95:.2f} ms")
        print(f"  P99:        {p99:.2f} ms")
        
        # Performance assessment
        print(f"\n🎯 Performance Assessment:")
        if avg_latency < 50:
            print("  ✅ EXCELLENT - Very low latency!")
        elif avg_latency < 100:
            print("  ✅ GOOD - Acceptable for teleoperation")
        elif avg_latency < 150:
            print("  ⚠️  ACCEPTABLE - May feel slightly laggy")
        elif avg_latency < 200:
            print("  ⚠️  MARGINAL - Noticeable lag, may affect precision")
        else:
            print("  ❌ POOR - Too much latency for smooth teleoperation")
        
        # Distribution histogram (simple ASCII)
        print(f"\n📈 Latency Distribution:")
        self.print_histogram()
        
        # Recommendations
        print(f"\n💡 Recommendations:")
        if avg_latency > 100:
            print("  • Consider using a data center closer to Mexico")
            print("  • Check network path with traceroute")
            print("  • Optimize WebSocket frame size")
            print("  • Consider compression if not enabled")
        if p99 > 200:
            print("  • High variance detected - investigate network stability")
        if self.errors > 0:
            print(f"  • {self.errors} failed requests - check server stability")
        
        print(f"\n{'='*60}\n")
    
    def print_histogram(self):
        """Print simple ASCII histogram"""
        if not self.latencies:
            return
        
        # Create bins
        min_lat = min(self.latencies)
        max_lat = max(self.latencies)
        num_bins = 10
        bin_size = (max_lat - min_lat) / num_bins
        
        if bin_size == 0:
            bin_size = 1
        
        bins = [0] * num_bins
        
        for lat in self.latencies:
            bin_idx = int((lat - min_lat) / bin_size)
            if bin_idx >= num_bins:
                bin_idx = num_bins - 1
            bins[bin_idx] += 1
        
        max_count = max(bins)
        scale = 40 / max_count if max_count > 0 else 1
        
        for i, count in enumerate(bins):
            bin_start = min_lat + i * bin_size
            bin_end = bin_start + bin_size
            bar = '█' * int(count * scale)
            print(f"  {bin_start:6.1f}-{bin_end:6.1f}ms |{bar} ({count})")


async def main():
    parser = argparse.ArgumentParser(description="Teleoperation Latency Tester")
    parser.add_argument("--server", type=str, 
                       default="ws://localhost:8000/ws/v1/teleop",
                       help="WebSocket server URL")
    parser.add_argument("--samples", type=int, default=100,
                       help="Number of test samples (default: 100)")
    parser.add_argument("--interval", type=float, default=0.1,
                       help="Interval between tests in seconds (default: 0.1)")
    
    args = parser.parse_args()
    
    tester = LatencyTester(args.server)
    
    try:
        await tester.run_test(num_samples=args.samples, interval=args.interval)
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user")
        if tester.latencies:
            tester.print_results()
    except Exception as e:
        print(f"\n❌ Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
