"""
MuJoCo Interactive Viewer Test - stays open until you close it
"""
import mujoco
import mujoco.viewer
import numpy as np
import time

print("MuJoCo version:", mujoco.__version__)

# Create a more interesting pendulum model
xml = """
<mujoco model="double_pendulum">
  <option timestep="0.01" gravity="0 0 -9.81"/>
  
  <visual>
    <global offwidth="640" offheight="480"/>
  </visual>
  
  <worldbody>
    <light diffuse=".8 .8 .8" pos="0 0 3" dir="0 0 -1"/>
    <camera name="main_cam" pos="2 -2 1.5" xyaxes="1 1 0 0 1 2"/>
    
    <!-- Ground plane -->
    <geom type="plane" size="2 2 0.1" rgba=".9 .9 .9 1"/>
    
    <!-- First pendulum link -->
    <body name="link1" pos="0 0 1">
      <joint name="joint1" type="hinge" axis="1 0 0" limited="false"/>
      <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5" rgba="1 0 0 1"/>
      <geom type="sphere" size="0.08" pos="0 0 -0.5" rgba="1 0 0 1"/>
      
      <!-- Second pendulum link -->
      <body name="link2" pos="0 0 -0.5">
        <joint name="joint2" type="hinge" axis="1 0 0" limited="false"/>
        <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.5" rgba="0 1 0 1"/>
        <geom type="sphere" size="0.1" pos="0 0 -0.5" rgba="0 1 0 1"/>
      </body>
    </body>
  </worldbody>
  
  <actuator>
    <motor joint="joint1" gear="1" ctrllimited="false"/>
    <motor joint="joint2" gear="1" ctrllimited="false"/>
  </actuator>
</mujoco>
"""

# Load model
print("Loading model...")
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

print("✓ Model loaded successfully!")
print(f"  Bodies: {model.nbody}")
print(f"  Joints: {model.njnt}")
print(f"  Actuators: {model.nu}")

# Give initial perturbation
data.qvel[0] = 2.0  # Initial angular velocity

print("\n" + "="*60)
print("LAUNCHING VIEWER")
print("="*60)
print("Controls:")
print("  - Click and drag: Rotate view")
print("  - Right-click drag: Pan view")
print("  - Scroll: Zoom")
print("  - Double-click: Select body")
print("  - Press SPACE: Pause/Resume simulation")
print("  - Press ESC or close window: Exit")
print("="*60)
print("\nViewer window should stay open...")
print("The pendulum will swing continuously until you close the window.\n")

# Launch viewer with continuous simulation
try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Keep running until user closes the window
        while viewer.is_running():
            # Step the simulation
            mujoco.mj_step(model, data)
            
            # Sync viewer (this updates the display)
            viewer.sync()
            
            # Optional: Small sleep to control frame rate
            # Without this, it runs as fast as possible
            time.sleep(0.001)
            
    print("\n✓ Viewer closed normally!")
    
except KeyboardInterrupt:
    print("\n✓ Interrupted by user (Ctrl+C)")
    
except Exception as e:
    print(f"\n✗ Error: {e}")
    print("\nIf you see 'requires mjpython on macOS', try:")
    print("  mjpython test_mujoco_interactive.py")

print("\n✅ Test complete!")
