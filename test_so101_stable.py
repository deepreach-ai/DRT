#!/usr/bin/env python3
"""
Test script for stable SO-101 robot model
"""
import mujoco
import mujoco.viewer
import numpy as np

# Load the stable model
print("Loading stable SO-101 model...")
model = mujoco.MjModel.from_xml_path("robots/so101.xml")
data = mujoco.MjData(model)

# Load the "home" keyframe (predefined pose)
keyframe_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
if keyframe_id >= 0:
    print("Loading 'home' keyframe...")
    data.qpos[:] = model.key_qpos[keyframe_id]
    data.ctrl[:] = model.key_qpos[keyframe_id][:model.nu]
else:
    print("Warning: No 'home' keyframe found, using default pose")
    # Fallback to manual home pose
    home_pose = np.array([0.0, -0.785, 1.57, -0.785, 0.0, 0.5])
    data.qpos[:len(home_pose)] = home_pose
    data.ctrl[:len(home_pose)] = home_pose

# Forward kinematics
mujoco.mj_forward(model, data)

print("\n" + "="*60)
print("SO-101 Robot Viewer - STABLE VERSION")
print("="*60)
print("Model loaded successfully!")
print(f"Joints: {model.njnt}")
print(f"Actuators: {model.nu}")
print(f"Initial pose: {data.qpos[:6]}")
print("\nControls:")
print("  Mouse scroll: Zoom in/out")
print("  Right-drag: Pan view")
print("  Left-drag: Rotate view")
print("  Double-click: Select body")
print("  Press '[' or ']': Switch cameras")
print("  Press SPACE: Pause/Resume simulation")
print("  Press ESC: Close viewer")
print("="*60)

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Set good initial camera view
    viewer.cam.lookat = np.array([0.0, 0.0, 0.25])
    viewer.cam.distance = 0.8
    viewer.cam.azimuth = 145
    viewer.cam.elevation = -25
    
    step_count = 0
    while viewer.is_running():
        # Keep control targets at home position for stability
        data.ctrl[:] = model.key_qpos[keyframe_id][:model.nu] if keyframe_id >= 0 else home_pose
        
        # Step simulation
        mujoco.mj_step(model, data)
        viewer.sync()
        
        step_count += 1
        
        # Print status every 1000 steps
        if step_count % 1000 == 0:
            pos_error = np.linalg.norm(data.qpos[:6] - data.ctrl[:6])
            print(f"Steps: {step_count:6d} | Position error: {pos_error:.6f} | Stable: {'✓' if pos_error < 0.01 else '✗'}")

print("\n✓ Viewer closed successfully!")
print("The robot should have been stable (no shaking).")
