# convert_urdf_to_usd.py
import argparse
import os
import sys

# Isaac Sim specific imports
try:
    from omni.isaac.kit import SimulationApp
    
    # Configure and start simulation app (required for conversion)
    CONFIG = {"renderer": "RayTracedLighting", "headless": False}
    simulation_app = SimulationApp(CONFIG)
    
    import omni.kit.commands
    from omni.isaac.core.utils.extensions import enable_extension
    
except ImportError:
    print("❌ Error: This script must be run within Isaac Sim's Python environment.")
    sys.exit(1)

def convert_urdf(urdf_path, usd_path, headless=True):
    """
    Convert URDF file to USD using Isaac Sim's internal commands.
    """
    abs_urdf_path = os.path.abspath(urdf_path)
    abs_usd_path = os.path.abspath(usd_path)
    
    print(f"🔄 Converting: {abs_urdf_path}")
    print(f"➡️  To: {abs_usd_path}")
    
    if not os.path.exists(abs_urdf_path):
        print(f"❌ Error: URDF file not found at {abs_urdf_path}")
        return False

    # Ensure output directory exists
    os.makedirs(os.path.dirname(abs_usd_path), exist_ok=True)
    
    # Enable URDF Importer extension
    # Try enabling the new extension name first (Isaac Sim 4.5+), then fallback to legacy
    try:
        enable_extension("isaacsim.asset.importer.urdf")
        urdf_extension_name = "isaacsim.asset.importer.urdf"
    except Exception:
        enable_extension("omni.importer.urdf")
        urdf_extension_name = "omni.importer.urdf"
    
    # Force update app to ensure extension is loaded
    for _ in range(10):
        simulation_app.update()

    # Import config settings
    import_config = _get_import_config(urdf_extension_name)
    
    # Run conversion
    # Note: URDFParseAndImportFile expects dest_path, not output_path
    # The command might vary by version. Let's try the modern signature first.
    # Isaac Sim 2022+ usually uses 'dest_path'
    
    # Fix for "package://" paths if needed
    # We can try to set the ROS_PACKAGE_PATH env var to the parent of the robot directory
    # E.g. if urdf is in /abs/path/to/robots/realman_65/RM65-6F.urdf
    # and it references package://RM65-6F/meshes/...
    # then ROS_PACKAGE_PATH should include /abs/path/to/robots/realman_65 (where the package dir is)
    
    # Check if the URDF uses package://
    # We assume the directory name containing the URDF is the package name or the parent contains it
    # In this case: robots/realman_65/RM65-6F.urdf
    # The mesh path is package://RM65-6F/meshes/base_link.STL
    # So we need a directory named "RM65-6F" that contains "meshes"
    # But our directory is "realman_65". 
    # The user might have renamed it or extracted it.
    # We need to ensure the structure matches what the URDF expects.
    
    # Hack: Temporarily symlink if needed or just set ROS_PACKAGE_PATH to the parent of the folder that matches the package name
    # But here the folder "realman_65" contains the URDF. The URDF expects "RM65-6F".
    # So we should probably rename "realman_65" to "RM65-6F" OR symlink it.
    
    # Let's try to infer the package root.
    urdf_dir = os.path.dirname(abs_urdf_path) # .../robots/realman_65
    
    # If the mesh paths are package://RM65-6F/...
    # We need ROS_PACKAGE_PATH to point to the directory CONTAINING "RM65-6F"
    # Current structure: .../robots/realman_65/RM65-6F.urdf
    #                    .../robots/realman_65/meshes/...
    # So "realman_65" acts as the package root, but its name is "realman_65", not "RM65-6F".
    # The URDF importer might fail to resolve "package://RM65-6F" if it looks for a folder named "RM65-6F".
    
    # Workaround: Create a symlink "RM65-6F" -> "." inside "realman_65" ? No, that would be recursive loop if naive.
    # Better: Create a symlink "RM65-6F" -> "realman_65" in the "robots" directory.
    
    robots_dir = os.path.dirname(urdf_dir) # .../robots
    symlink_path = os.path.join(robots_dir, "RM65-6F")
    
    created_symlink = False
    if not os.path.exists(symlink_path) and os.path.basename(urdf_dir) != "RM65-6F":
        try:
            os.symlink(urdf_dir, symlink_path)
            created_symlink = True
            print(f"🔗 Created symlink for ROS package: {symlink_path} -> {urdf_dir}")
        except Exception as e:
            print(f"⚠️ Failed to create symlink: {e}")

    # Set ROS_PACKAGE_PATH
    os.environ["ROS_PACKAGE_PATH"] = f"{robots_dir}:{os.environ.get('ROS_PACKAGE_PATH', '')}"
    print(f"ℹ️  Set ROS_PACKAGE_PATH to include: {robots_dir}")

    status = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=abs_urdf_path,
        import_config=import_config,
        dest_path=abs_usd_path,
    )
    
    if created_symlink:
        try:
            os.unlink(symlink_path)
            print("🧹 Removed temporary symlink")
        except:
            pass
    
    if status:
        print("✅ Conversion Successful!")
    else:
        print("❌ Conversion Failed!")
        
    return status

def _get_import_config(extension_name="omni.importer.urdf"):
    if extension_name == "isaacsim.asset.importer.urdf":
        from isaacsim.asset.importer.urdf import _urdf
    else:
        from omni.importer.urdf import _urdf
    
    # Create default config
    import_config = _urdf.ImportConfig()
    
    # Common settings for robotic arms
    import_config.merge_fixed_joints = False
    import_config.fix_base = True
    import_config.make_default_prim = True
    import_config.self_collision = False
    import_config.create_physics_scene = True
    import_config.import_inertia_tensor = True
    import_config.default_drive_strength = 10000.0
    import_config.default_position_drive_damping = 100.0
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.distance_scale = 1.0
    # import_config.up_vector = 0, 0, 1
    
    return import_config

def convert_mjcf(xml_path, usd_path):
    """
    Convert MJCF (MuJoCo XML) to USD.
    Note: Isaac Sim MJCF importer is sometimes experimental.
    """
    abs_xml_path = os.path.abspath(xml_path)
    abs_usd_path = os.path.abspath(usd_path)
    
    print(f"🔄 Converting MJCF: {abs_xml_path}")
    
    if not os.path.exists(abs_xml_path):
        print(f"❌ Error: XML file not found at {abs_xml_path}")
        return False

    # Try enabling the new extension name first (Isaac Sim 4.5+), then fallback to legacy
    try:
        enable_extension("isaacsim.asset.importer.mjcf")
        mjcf_extension_name = "isaacsim.asset.importer.mjcf"
    except Exception:
        enable_extension("omni.importer.mjcf")
        mjcf_extension_name = "omni.importer.mjcf"
    
    if mjcf_extension_name == "isaacsim.asset.importer.mjcf":
        from isaacsim.asset.importer.mjcf import _mjcf
    else:
        from omni.importer.mjcf import _mjcf
    
    import_config = _mjcf.ImportConfig()
    import_config.fix_base = True
    import_config.make_default_prim = True
    import_config.create_physics_scene = True
    
    status = omni.kit.commands.execute(
        "MJCFCreateAsset",
        mjcf_path=abs_xml_path,
        import_config=import_config,
        prim_path="/World/so101",
        dest_path=abs_usd_path
    )
    
    if status:
        print("✅ MJCF Conversion Successful!")
    else:
        print("❌ MJCF Conversion Failed!")
        
    return status

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert URDF/MJCF to USD for Isaac Sim")
    parser.add_argument("input", help="Path to input file (.urdf or .xml)")
    parser.add_argument("output", help="Path to output .usd file")
    args = parser.parse_args()
    
    file_ext = os.path.splitext(args.input)[1].lower()
    
    if file_ext == ".urdf":
        convert_urdf(args.input, args.output)
    elif file_ext == ".xml":
        convert_mjcf(args.input, args.output)
    else:
        print(f"❌ Unsupported file type: {file_ext}")
    
    simulation_app.close()
