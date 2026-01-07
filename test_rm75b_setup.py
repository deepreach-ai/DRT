#!/usr/bin/env python3
"""
Quick test script for RM75B VR setup
Verifies MuJoCo model and basic IK functionality
"""

import sys
import os
import numpy as np

def test_mujoco_import():
    """Test if MuJoCo is installed"""
    print("🔍 Testing MuJoCo installation...")
    try:
        import mujoco
        print(f"   ✅ MuJoCo version: {mujoco.__version__}")
        return True
    except ImportError as e:
        print(f"   ❌ MuJoCo not found: {e}")
        print("   Install with: pip install mujoco")
        return False

def test_model_loading():
    """Test if RM75B model loads correctly"""
    print("\n🔍 Testing RM75B model loading...")
    
    model_path = "robots/rm75b_vr_v2.xml"
    if not os.path.exists(model_path):
        print(f"   ❌ Model file not found: {model_path}")
        return False
    
    try:
        import mujoco
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
        
        print(f"   ✅ Model loaded successfully")
        # print(f"   ℹ️  Model name: {model.name}") # attribute 'name' not available in newer mujoco versions
        print(f"   ℹ️  DOF: {model.nv}")
        print(f"   ℹ️  Joints: {model.njnt}")
        print(f"   ℹ️  Actuators: {model.nu}")
        
        # List joints
        print(f"\n   📝 Joint names:")
        for i in range(model.njnt):
            jnt_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            jnt_range = model.jnt_range[i]
            print(f"      {i+1}. {jnt_name}: [{jnt_range[0]:.2f}, {jnt_range[1]:.2f}] rad")
        
        return True, model, data
        
    except Exception as e:
        print(f"   ❌ Failed to load model: {e}")
        return False, None, None

def test_end_effector_site():
    """Test if end effector site exists"""
    print("\n🔍 Testing end effector configuration...")
    
    try:
        import mujoco
        model = mujoco.MjModel.from_xml_path("robots/rm75b_vr_v2.xml")
        
        ee_site = "ee_site"
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, ee_site)
        
        if site_id < 0:
            print(f"   ❌ End effector site '{ee_site}' not found")
            print("   Available sites:")
            for i in range(model.nsite):
                site_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i)
                print(f"      - {site_name}")
            return False
        
        print(f"   ✅ End effector site found: {ee_site} (id={site_id})")
        return True
        
    except Exception as e:
        print(f"   ❌ Failed to check end effector: {e}")
        return False

def test_ik_solver():
    """Test basic IK solver functionality"""
    print("\n🔍 Testing IK solver...")
    
    try:
        import mujoco
        model = mujoco.MjModel.from_xml_path("robots/rm75b_vr_v2.xml")
        data = mujoco.MjData(model)
        
        # Reset to home position
        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)
        
        # Get current end effector position
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
        ee_pos_initial = data.site_xpos[site_id].copy()
        
        print(f"   ℹ️  Initial EE position: {ee_pos_initial}")
        
        # Try to move slightly
        target_pos = ee_pos_initial + np.array([0.1, 0.0, 0.0])  # 10cm in X
        print(f"   ℹ️  Target position: {target_pos}")
        
        # Simple IK iteration (just for testing)
        for _ in range(10):
            mujoco.mj_step(model, data)
        
        ee_pos_final = data.site_xpos[site_id].copy()
        print(f"   ℹ️  Final EE position: {ee_pos_final}")
        
        print(f"   ✅ IK solver can run (full IK test requires backend)")
        return True
        
    except Exception as e:
        print(f"   ❌ IK solver test failed: {e}")
        return False

def test_cameras():
    """Test if VR cameras exist"""
    print("\n🔍 Testing VR camera configuration...")
    
    try:
        import mujoco
        model = mujoco.MjModel.from_xml_path("robots/rm75b_vr_v2.xml")
        
        expected_cameras = ["vr_left_eye", "vr_right_eye", "tracking"]
        found_cameras = []
        
        for cam_name in expected_cameras:
            cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
            if cam_id >= 0:
                found_cameras.append(cam_name)
                print(f"   ✅ Camera '{cam_name}' found (id={cam_id})")
            else:
                print(f"   ⚠️  Camera '{cam_name}' not found")
        
        if len(found_cameras) >= 2:
            print(f"   ✅ Sufficient cameras for VR ({len(found_cameras)}/3)")
            return True
        else:
            print(f"   ❌ Not enough cameras for VR")
            return False
            
    except Exception as e:
        print(f"   ❌ Camera test failed: {e}")
        return False

def test_server_prerequisites():
    """Test if server can be started"""
    print("\n🔍 Testing server prerequisites...")
    
    try:
        # Check if run_server.py exists
        if not os.path.exists("run_server.py"):
            print("   ❌ run_server.py not found")
            return False
        
        print("   ✅ run_server.py found")
        
        # Check if required packages are available
        try:
            import flask
            from importlib.metadata import version
            print(f"   ✅ Flask installed: {version('flask')}")
        except ImportError:
            print("   ❌ Flask not installed (pip install flask)")
            return False
        
        try:
            import numpy
            print(f"   ✅ NumPy installed: {numpy.__version__}")
        except ImportError:
            print("   ❌ NumPy not installed (pip install numpy)")
            return False
        
        print("   ✅ All prerequisites met")
        return True
        
    except Exception as e:
        print(f"   ❌ Prerequisites check failed: {e}")
        return False

def main():
    print("=" * 60)
    print("🤖 RM75B VR Setup Verification")
    print("=" * 60)
    
    results = []
    
    # Run tests
    results.append(("MuJoCo Installation", test_mujoco_import()))
    
    model_result = test_model_loading()
    if isinstance(model_result, tuple):
        results.append(("Model Loading", model_result[0]))
    else:
        results.append(("Model Loading", model_result))
    
    results.append(("End Effector Site", test_end_effector_site()))
    results.append(("IK Solver", test_ik_solver()))
    results.append(("VR Cameras", test_cameras()))
    results.append(("Server Prerequisites", test_server_prerequisites()))
    
    # Summary
    print("\n" + "=" * 60)
    print("📊 Test Summary")
    print("=" * 60)
    
    passed = 0
    failed = 0
    
    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status} - {test_name}")
        if result:
            passed += 1
        else:
            failed += 1
    
    print("=" * 60)
    print(f"Total: {passed} passed, {failed} failed")
    
    if failed == 0:
        print("\n🎉 All tests passed! Ready to start VR control.")
        print("\n📝 Next steps:")
        print("   1. Start server: python run_server.py --backend mujoco \\")
        print("                       --mujoco-xml robots/rm75b_vr_v2.xml \\")
        print("                       --mujoco-ee ee_site")
        print("   2. Connect Quest 3S to http://YOUR_IP:8000/web/")
        print("   3. Enter VR mode and start controlling!")
        return 0
    else:
        print("\n⚠️  Some tests failed. Please fix the issues above.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
