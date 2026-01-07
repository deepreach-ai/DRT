
import mujoco
import numpy as np
from transforms3d.euler import mat2euler

def main():
    model_path = "../robots/so101.xml"
    try:
        model = mujoco.MjModel.from_xml_path(model_path)
    except Exception as e:
        print(f"Error loading model: {e}")
        return

    print('<?xml version="1.0"?>')
    print('<robot name="so101">')
    
    # Iterate bodies
    # Body 0 is world, Body 1 is base
    for i in range(1, model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        parent_id = model.body_parentid[i]
        parent_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, parent_id)
        if parent_name == "world":
            parent_name = "base_link" # URDF convention
        
        # Get relative pose
        pos = model.body_pos[i]
        quat = model.body_quat[i] # w, x, y, z
        mat = np.zeros(9)
        mujoco.mju_quat2Mat(mat, quat)
        mat = mat.reshape(3, 3)
        rpy = mat2euler(mat, 'sxyz')
        
        print(f'  <link name="{body_name}">')
        
        # Find geoms for this body
        geom_start = model.body_geomadr[i]
        geom_num = model.body_geomnum[i]
        for g in range(geom_start, geom_start + geom_num):
            mesh_id = model.geom_dataid[g]
            if mesh_id != -1:
                mesh_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mesh_id)
                # Geom pose
                g_pos = model.geom_pos[g]
                g_quat = model.geom_quat[g]
                g_mat = np.zeros(9)
                mujoco.mju_quat2Mat(g_mat, g_quat)
                g_mat = g_mat.reshape(3, 3)
                g_rpy = mat2euler(g_mat, 'sxyz')
                
                print('    <visual>')
                print(f'      <origin xyz="{g_pos[0]} {g_pos[1]} {g_pos[2]}" rpy="{g_rpy[0]} {g_rpy[1]} {g_rpy[2]}"/>')
                print('      <geometry>')
                print(f'        <mesh filename="assets/{mesh_name}.stl"/>')
                print('      </geometry>')
                print('    </visual>')
        print('  </link>')
        
        # Joint (connects parent to this body)
        # MuJoCo defines joints INSIDE the child body. URDF defines joints CONNECTING parent and child.
        jnt_start = model.body_jntadr[i]
        jnt_num = model.body_jntnum[i]
        
        if jnt_num > 0:
            for j in range(jnt_start, jnt_start + jnt_num):
                jnt_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
                jnt_type = model.jnt_type[j] # 2 = hinge (revolute), 3 = slide (prismatic)
                jnt_axis = model.jnt_axis[j]
                
                type_str = "fixed"
                if jnt_type == 2: type_str = "revolute"
                elif jnt_type == 3: type_str = "prismatic"
                
                print(f'  <joint name="{jnt_name}" type="{type_str}">')
                print(f'    <parent link="{parent_name}"/>')
                print(f'    <child link="{body_name}"/>')
                print(f'    <origin xyz="{pos[0]} {pos[1]} {pos[2]}" rpy="{rpy[0]} {rpy[1]} {rpy[2]}"/>')
                print(f'    <axis xyz="{jnt_axis[0]} {jnt_axis[1]} {jnt_axis[2]}"/>')
                # Limits
                if model.jnt_limited[j]:
                    limits = model.jnt_range[j]
                    print(f'    <limit lower="{limits[0]}" upper="{limits[1]}" effort="10" velocity="3"/>')
                print('  </joint>')
        else:
            # Fixed joint
            print(f'  <joint name="{body_name}_joint" type="fixed">')
            print(f'    <parent link="{parent_name}"/>')
            print(f'    <child link="{body_name}"/>')
            print(f'    <origin xyz="{pos[0]} {pos[1]} {pos[2]}" rpy="{rpy[0]} {rpy[1]} {rpy[2]}"/>')
            print('  </joint>')

    print('</robot>')

if __name__ == "__main__":
    main()
