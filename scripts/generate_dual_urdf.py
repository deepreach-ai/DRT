import xml.etree.ElementTree as ET
import sys
import os

def generate_dual_urdf(input_file, output_file):
    tree = ET.parse(input_file)
    root = tree.getroot()
    
    new_robot = ET.Element("robot", name="so101_dual")
    
    # Add world link
    world_link = ET.SubElement(new_robot, "link", name="world")
    
    def add_arm(prefix, y_offset):
        # Add base link to world joint
        base_joint = ET.SubElement(new_robot, "joint", name=f"world_to_{prefix}base", type="fixed")
        ET.SubElement(base_joint, "parent", link="world")
        ET.SubElement(base_joint, "child", link=f"{prefix}base")
        ET.SubElement(base_joint, "origin", xyz=f"0.0 {y_offset} 0.0", rpy="0 0 0")
        
        # Copy all links and joints
        for elem in root:
            if elem.tag == "link":
                new_link = ET.SubElement(new_robot, "link", name=f"{prefix}{elem.get('name')}")
                for child in elem:
                    new_child = ET.SubElement(new_link, child.tag)
                    new_child.attrib = child.attrib.copy()
                    for subchild in child:
                        new_subchild = ET.SubElement(new_child, subchild.tag)
                        new_subchild.attrib = subchild.attrib.copy()
                        if subchild.tag == "mesh":
                            # Meshes are in assets/ relative to so101.urdf
                            pass
            elif elem.tag == "joint":
                # Skip the original base_joint that connects to base_link
                if elem.get('name') == "base_joint":
                    continue
                
                new_joint = ET.SubElement(new_robot, "joint", name=f"{prefix}{elem.get('name')}", type=elem.get('type'))
                new_joint.attrib = elem.attrib.copy()
                new_joint.set('name', f"{prefix}{elem.get('name')}")
                
                for child in elem:
                    new_child = ET.SubElement(new_joint, child.tag)
                    new_child.attrib = child.attrib.copy()
                    if child.tag in ["parent", "child"]:
                        new_child.set("link", f"{prefix}{child.get('link')}")

    add_arm("left_", 0.2)
    add_arm("right_", -0.2)
    
    # Write to file
    with open(output_file, "wb") as f:
        f.write(b'<?xml version="1.0"?>\n')
        f.write(ET.tostring(new_robot))

if __name__ == "__main__":
    input_urdf = "robots/so101.urdf"
    output_urdf = "robots/so101_dual.urdf"
    generate_dual_urdf(input_urdf, output_urdf)
    print(f"Generated {output_urdf}")
