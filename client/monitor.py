"""
Simple terminal monitor to visualize robot state
"""
import requests
import time
import sys
import os

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

def get_status(url="http://localhost:8000"):
    try:
        # Get statistics which includes backend status and controller stats
        response = requests.get(f"{url}/api/v1/statistics", timeout=0.5)
        if response.status_code == 200:
            return response.json()
    except:
        return None
    return None

def print_dashboard(stats):
    clear_screen()
    print("="*50)
    print("TELEOPERATION SYSTEM MONITOR")
    print("="*50)
    
    if not stats:
        print("\nWaiting for server connection...")
        return

    # Server Info
    uptime = stats.get('uptime', 0)
    print(f"\nSERVER STATUS:")
    print(f"  Uptime: {uptime:.1f}s")
    print(f"  Total Commands: {stats.get('total_commands', 0)}")
    print(f"  Clients: {stats.get('connected_clients', 0)}")
    print(f"  Safety Gate: {'ACTIVE ✅' if stats.get('safety_gate_active') else 'INACTIVE ❌'}")

    # Robot State
    backend = stats.get('backend_status', {})
    controller = stats.get('controller_stats', {})
    
    # Use controller position as it's the command target
    pos = controller.get('current_position', [0,0,0])
    orient = controller.get('current_orientation', [1,0,0,0])
    
    print(f"\nROBOT STATE (Mock/Sim):")
    print(f"  Position (XYZ):  [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
    print(f"  Orient (WXYZ):   [{orient[0]:.3f}, {orient[1]:.3f}, {orient[2]:.3f}, {orient[3]:.3f}]")
    
    # Violations
    w_viol = controller.get('workspace_violations', 0)
    v_viol = controller.get('velocity_violations', 0)
    
    print(f"\nSAFETY STATS:")
    print(f"  Workspace Violations: {w_viol}")
    print(f"  Velocity Limit Hits:  {v_viol}")
    
    print("\n" + "="*50)
    print("Press Ctrl+C to exit")

def main():
    print("Starting monitor...")
    try:
        while True:
            stats = get_status()
            print_dashboard(stats)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nExiting monitor.")

if __name__ == "__main__":
    main()
