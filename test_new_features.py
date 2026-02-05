#!/usr/bin/env python3
"""
Quick Test Script - Universal Teleoperation System
Tests all new components: config system, unified backend, VR controller
"""

import sys
import os

# Add server to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'server'))

def test_config_system():
    """Test robot configuration system"""
    print("\n" + "="*60)
    print("TEST 1: Robot Configuration System")
    print("="*60)
    
    try:
        from robot_config import RobotRegistry, load_robot_config
        
        # Test registry
        print("\n📂 Loading robot registry...")
        registry = RobotRegistry()
        
        robots = registry.list_robots()
        print(f"✅ Found {len(robots)} robots: {robots}")
        
        if len(robots) == 0:
            print("⚠️  No robots loaded! Check configs/robots/ directory")
            return False
        
        # Test loading individual robot
        first_robot = robots[0]
        print(f"\n📝 Testing config load for '{first_robot}'...")
        config = registry.get_robot(first_robot)
        
        print(f"   Robot: {config.robot_name}")
        print(f"   DOF: {config.dof}")
        print(f"   Has simulation: {config.has_simulation_backend()}")
        print(f"   Has real robot: {config.has_real_backend()}")
        
        # Validate
        config.validate()
        print(f"✅ Config validation passed")
        
        # Test summary
        summary = registry.get_summary()
        print(f"\n📊 Registry summary:")
        print(f"   Total robots: {summary['total_robots']}")
        
        return True
        
    except Exception as e:
        print(f"❌ Config system test FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_unified_backend():
    """Test unified backend (simulation mode only - no Isaac Sim required)"""
    print("\n" + "="*60)
    print("TEST 2: Unified Backend")
    print("="*60)
    
    try:
        from unified_backend import UnifiedRobotBackend
        from robot_config import RobotRegistry
        
        # Get first available robot with simulation backend
        registry = RobotRegistry()
        sim_robots = registry.list_robots_with_sim()
        
        if not sim_robots:
            print("⚠️  No robots with simulation backend configured")
            return False
        
        robot_name = sim_robots[0]
        config = registry.get_robot(robot_name)
        
        print(f"\n🤖 Creating unified backend for '{robot_name}'...")
        
        # Create backend (don't connect - no Isaac Sim)
        config_path = config.file_path
        if not config_path:
             config_path = f"configs/robots/{robot_name.lower()}.yaml"
             
        robot = UnifiedRobotBackend(
            config_path,
            mode="simulation"
        )
        
        print(f"✅ Unified backend created")
        print(f"   Robot: {robot.config.robot_name}")
        print(f"   Mode: {robot.mode.value}")
        print(f"   Has sim: {robot.config.has_simulation_backend()}")
        print(f"   Has real: {robot.config.has_real_backend()}")
        
        # Test workspace limits
        limits = robot.get_workspace_limits()
        print(f"\n📏 Workspace limits:")
        print(f"   X: {limits['x']}")
        print(f"   Y: {limits['y']}")
        print(f"   Z: {limits['z']}")
        
        # Test max velocity
        max_vel = robot.get_max_velocity()
        print(f"\n⚡ Max velocity: {max_vel} m/s")
        
        print(f"\n✅ Unified backend test passed")
        print(f"   Note: Not testing connection (requires Isaac Sim)")
        
        return True
        
    except Exception as e:
        print(f"❌ Unified backend test FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_vr_controller_imports():
    """Test VR controller imports (don't start server)"""
    print("\n" + "="*60)
    print("TEST 3: VR Controller Imports")
    print("="*60)
    
    try:
        # Add client to path
        client_path = os.path.join(os.path.dirname(__file__), 'client')
        sys.path.insert(0, client_path)
        
        print("\n📦 Testing VR controller imports...")
        
        # Test imports
        from vr.quest3_controller import Quest3Controller, HandPose
        
        print("✅ Quest3Controller imported")
        print("✅ HandPose imported")
        
        # Create controller (don't run)
        controller = Quest3Controller(use_ngrok=False)
        print(f"✅ Controller instance created")
        print(f"   Resolution: {controller.resolution}")
        print(f"   Control scale: {controller.control_scale}")
        print(f"   Deadman threshold: {controller.deadman_threshold}")
        
        print(f"\n✅ VR controller imports test passed")
        print(f"   Note: Not starting server (run manually to test)")
        
        return True
        
    except Exception as e:
        print(f"❌ VR controller test FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False


def print_summary(results):
    """Print test summary"""
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    
    tests = [
        ("Config System", results[0]),
        ("Unified Backend", results[1]),
        ("VR Controller", results[2])
    ]
    
    all_passed = all(r for r in results)
    
    print()
    for test_name, passed in tests:
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{test_name:.<30} {status}")
    
    print("\n" + "="*60)
    if all_passed:
        print("🎉 ALL TESTS PASSED!")
        print("\nNext steps:")
        print("  1. Get robot URDF files")
        print("  2. Convert URDFs to USD for Isaac Sim")
        print("  3. Test with actual Isaac Sim")
        print("  4. Test VR control: python client/vr/quest3_controller.py")
    else:
        print("⚠️  SOME TESTS FAILED")
        print("\nCheck error messages above for details")
    print("="*60)
    
    return all_passed


def main():
    """Run all tests"""
    print("="*60)
    print("Universal Teleoperation System - Quick Test")
    print("="*60)
    print("\nThis script tests:")
    print("  1. Robot configuration system")
    print("  2. Unified backend (sim/real switcher)")
    print("  3. VR controller imports")
    print("\nNote: Does NOT require Isaac Sim or Quest 3 to run")
    
    # Run tests
    results = []
    
    # Test 1: Config system
    results.append(test_config_system())
    
    # Test 2: Unified backend
    results.append(test_unified_backend())
    
    # Test 3: VR controller
    results.append(test_vr_controller_imports())
    
    # Summary
    all_passed = print_summary(results)
    
    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
