"""
Test MuJoCo installation - macOS compatible version
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
    </body>
  </worldbody>
</mujoco>
"""

# Load model
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

print("✓ MuJoCo model created successfully!")
print(f"✓ Number of bodies: {model.nbody}")
print(f"✓ Number of joints: {model.njnt}")

# Run a few simulation steps
for i in range(100):
    mujoco.mj_step(model, data)

print("✓ Simulation ran successfully!")

# Try alternative viewer (works better on macOS)
try:
    import mujoco_viewer
    
    print("\nLaunching mujoco_viewer... (Close window to exit)")
    viewer = mujoco_viewer.MujocoViewer(model, data)
    
    # Run simulation in viewer
    for _ in range(1000):
        if viewer.is_alive:
            mujoco.mj_step(model, data)
            viewer.render()
        else:
            break
    
    viewer.close()
    print("✓ Viewer test passed!")
    
except ImportError:
    print("\n⚠️  mujoco_viewer not installed. Install with:")
    print("    pip install mujoco-python-viewer")
    print("\n✓ Core MuJoCo is working! Viewer is optional.")

print("\n✅ All tests passed! MuJoCo is ready to use.")
