#!/usr/bin/env python3
"""
Test SO-ARM backend integration
"""
import sys
import os
import time

# Add server to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'server'))

from robot_backend import BackendFactory

def test_soarm_connection():
    """Test connecting to SO-ARM"""
    print("="*60)
    print("Testing SO-ARM Backend Connection")
    print("="*60)
    
    # Create backend
    print("\n1. Creating SO-ARM backend...")
    backend = BackendFactory.create_backend(
        'soarm',
        port='/dev/tty.usbmodem5B3E1187881',  # Your port from test_motor_connection.py
        name='so101_test'
    )
    print(f"✓ Backend created: {backend.name}")
    
    # Connect
    print("\n2. Connecting to robot...")
    if backend.connect():
        print("✓ Connection successful!")
    else:
        print("✗ Connection failed!")
        return False
    
    # Get status
    print("\n3. Getting robot status...")
    status = backend.get_status()
    print(f"Status: {status}")
    
    # Try to read joint positions
    print("\n4. Reading current joint positions...")
    try:
        if hasattr(backend, 'get_current_joints'):
            joints = backend.get_current_joints()
            if joints is not None:
                print(f"Current joints: {joints}")
            else:
                print("Could not read joints")
    except Exception as e:
        print(f"Error reading joints: {e}")
    
    # Wait a bit
    print("\n5. Keeping connection alive for 5 seconds...")
    time.sleep(5)
    
    # Disconnect
    print("\n6. Disconnecting...")
    backend.disconnect()
    print("✓ Test complete!")
    
    return True

if __name__ == "__main__":
    print("\n🤖 SO-ARM Backend Integration Test\n")
    
    try:
        success = test_soarm_connection()
        if success:
            print("\n✅ ALL TESTS PASSED!")
            print("\nNext steps:")
            print("1. Update server/config.yml to use 'soarm' backend")
            print("2. Run: python run_server.py")
            print("3. Open web UI and control your robot!")
        else:
            print("\n❌ TEST FAILED")
            print("\nTroubleshooting:")
            print("1. Check robot is powered on")
            print("2. Verify USB connection")
            print("3. Check port name is correct")
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
