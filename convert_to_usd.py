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
    status = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=abs_urdf_path,
        import_config=import_config,
        dest_path=abs_usd_path,
    )
    
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
