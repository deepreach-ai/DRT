"""
Test MuJoCo headless rendering (for teleoperation)
This is what you'll use in your actual backend
"""
import mujoco
import numpy as np

print("MuJoCo version:", mujoco.__version__)

# Create a simple pendulum model
xml = """
<mujoco>
  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="1 1 0.1" rgba=".9 0 0 1"/>
    <body pos="0 0 1">
      <joint type="hinge" axis="1 0 0"/>
      <geom type="capsule" size="0.1" fromto="0 0 0 0 0 -1"/>
      <body pos="0 0.2 -1">
        <geom type="sphere" size="0.2" rgba="0 .9 0 1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

# Load model
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

print("✓ MuJoCo model created successfully!")

# Create offscreen renderer (no GUI needed!)
renderer = mujoco.Renderer(model, height=480, width=640)

print("✓ Offscreen renderer created (headless mode)")

# Simulate and render frames
print("\nRunning simulation and capturing frames...")
for i in range(100):
    # Step simulation
    mujoco.mj_step(model, data)
    
    # Render frame every 10 steps
    if i % 10 == 0:
        # Update renderer with current state
        renderer.update_scene(data)
        
        # Render to numpy array (this is what you'd stream to web UI!)
        frame = renderer.render()
        
        print(f"  Frame {i//10}: shape={frame.shape}, dtype={frame.dtype}")

print("\n✓ Headless rendering works perfectly!")
print("  This is exactly what you need for teleoperation:")
print("  - No GUI required on server")
print("  - Render frames to numpy arrays")
print("  - Stream to web browser or VR headset")
print("\n✅ MuJoCo is ready for your teleoperation backend!")
