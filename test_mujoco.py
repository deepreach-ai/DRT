# """
# Test MuJoCo installation
# """
# import mujoco
# import mujoco.viewer
# import numpy as np

# print("MuJoCo version:", mujoco.__version__)

# # Create a simple pendulum model
# xml = """
# <mujoco>
#   <worldbody>
#     <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
#     <geom type="plane" size="1 1 0.1" rgba=".9 0 0 1"/>
#     <body pos="0 0 1">
#       <joint type="hinge" axis="1 0 0"/>
#       <geom type="capsule" size="0.1" fromto="0 0 0 0 0 -1"/>
#     </body>
#   </worldbody>
# </mujoco>
# """

# # Load model
# model = mujoco.MjModel.from_xml_string(xml)
# data = mujoco.MjData(model)

# print("✓ MuJoCo model created successfully!")
# print(f"✓ Number of bodies: {model.nbody}")
# print(f"✓ Number of joints: {model.njnt}")

# # Run a few simulation steps
# for i in range(100):
#     mujoco.mj_step(model, data)

# print("✓ Simulation ran successfully!")

# # Optional: Launch interactive viewer
# print("\nLaunching viewer... (Press ESC to close)")
# with mujoco.viewer.launch_passive(model, data) as viewer:
#     # Run simulation in viewer
#     for _ in range(1000):
#         mujoco.mj_step(model, data)
#         viewer.sync()
        
# print("✓ All tests passed! MuJoCo is working correctly.")
import mujoco
import mujoco.viewer
import os

# Define the path to your XML file
xml_path = os.path.join(os.path.dirname(__file__), "robots/so101.xml")

# Load the model
print(f"Loading model from: {xml_path}")
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

print("Model loaded successfully!")
print(f"Press Space to toggle simulation, or use the mouse to rotate/pan.")

# After creating model and data
keyframe_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
if keyframe_id >= 0:
    data.qpos[:] = model.key_qpos[keyframe_id]
    mujoco.mj_forward(model, data)

# Launch the interactive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Loop while the viewer is open
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()