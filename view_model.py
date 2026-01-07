import mujoco as mj
import mujoco.viewer as viewer
import os

model_path = 'robots/rm75b_vr_v2.xml'
print(f"Loading model from: {model_path}")

try:
    m = mj.MjModel.from_xml_path(model_path)
    d = mj.MjData(m)
    
    print("Model loaded successfully. Launching viewer...")
    with viewer.launch_passive(m, d) as v:
        while v.is_running():
            mj.mj_step(m, d)
            v.sync()
except Exception as e:
    print(f"Error: {e}")
