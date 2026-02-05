import mujoco
import mujoco.viewer
import os
import numpy as np

xml_path = os.path.join(os.path.dirname(__file__), "robots/so101.xml")

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# Set good initial pose
data.qpos[:] = [0.0, -0.785, 1.57, -0.785, 0.0, 0.5]

# DISABLE ALL ACTUATORS by setting ctrl to match qpos
# This makes the robot passive
data.ctrl[:] = data.qpos[:model.nu]

mujoco.mj_forward(model, data)

print("Passive mode - robot should be stable")
print("Joint positions:", data.qpos[:6])

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.lookat = np.array([0.0, 0.0, 0.25])
    viewer.cam.distance = 0.8
    viewer.cam.azimuth = 145
    viewer.cam.elevation = -25
    
    while viewer.is_running():
        # Keep ctrl matching qpos to prevent actuation
        data.ctrl[:] = data.qpos[:model.nu]
        mujoco.mj_step(model, data)
        viewer.sync()