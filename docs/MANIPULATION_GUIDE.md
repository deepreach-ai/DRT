# 🤖 SO-ARM101 Teleoperation Manipulation Guide

Welcome to your VR Teleoperation system! This guide will help you master the control of your SO-ARM101 robot using the Meta Quest 3S.

## 🕹 VR Controls (Quest Controllers)

### **1. The Clutch (Grip Button)**
The most important control is the **Grip Button** (the one under your middle finger).
- **HOLD Grip**: The robot's end-effector will follow your hand's 6-DoF movement (position and rotation) in real-time.
- **RELEASE Grip**: The robot stays in place. You can now move your physical hand to a more comfortable position without moving the robot.
- **Pro Tip**: Use this like a "mouse lift" to reset your reach when you run out of arm space.

### **2. Moving the Arm (Joystick)**
When **NOT** holding the Grip button:
- **Right Joystick (Up/Down)**: Moves the arm Forward/Backward (Robot X).
- **Right Joystick (Left/Right)**: Moves the arm Left/Right (Robot Y).
- **A Button (Right Controller)**: Moves the arm Down (Robot Z-).
- **B Button (Right Controller)**: Moves the arm Up (Robot Z+).
- **B Button (Long Press)**: Exit VR mode.

### **3. The Gripper (Trigger)**
- **Right Trigger (Index Finger)**: Controls the opening/closing of the gripper.
- Squeeze to close, release to open.

---

## 🚀 Best Practices for Smooth Operation

### **1. Slow is Smooth**
The robot has safety limits to prevent violent movements. If you move your hand too fast, the system will "clamp" the movement to a safe speed. Move your hands steadily for the best precision.

### **2. Calibration at Start**
When you first enter VR:
1. Look at the 3D robot model.
2. Hold the **Grip button** and move your hand to align the virtual "Ghost" robot with your physical expectation.
3. If the robot seems offset, release the grip, move your hand, and re-engage.

### **3. Dealing with "Flips"**
If you rotate your wrist too far, the Inverse Kinematics (IK) might try to "flip" the robot's arm to reach the target. The system automatically suppresses these flips to prevent damage. If the robot stops moving during a rotation, try rotating your wrist back slightly.

---

## 🛠 Troubleshooting Common Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| **Robot moves too fast** | High sensitivity | We have reduced the default sensitivity. Move your hands slower for precision. |
| **"Negative values" error** | Internal data mismatch | Fixed! The system now correctly converts degrees to motor units. |
| **Robot doesn't move** | Clutch not engaged | Ensure you are holding the **Grip** button while moving. |
| **Ghost robot is far away** | Workspace limit | The robot cannot reach outside its physical workspace (approx 30cm radius). |

---

*Happy Teleoperating!* 🦾
