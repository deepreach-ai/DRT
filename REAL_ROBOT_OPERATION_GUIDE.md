# Real Robot Operation Guide (soarm101)

Congratulations! You can now use Meta Quest 3/3S to remotely control the real **soarm101** robotic arm. To ensure safe and smooth operation, please read the following steps carefully.

## 1. Hardware Setup

- **Connection**: Connect the soarm101 to the computer running the server (Mac/Linux) using a USB cable.
- **Power**: **MUST** connect the 12V external power adapter. USB power alone is not sufficient to drive the motors.
- **Port Check**: 
  - The default port is `/dev/tty.usbmodem5B3E1224691`.
  - If the port is different, add the port parameter after the startup script, e.g., `./start_so101_vr_real.sh /dev/ttyUSB0`.

## 2. Software Startup

Run the following command in the terminal to start the server:

```bash
./start_so101_vr_real.sh
```

- The terminal will display a URL, e.g., `http://192.168.1.100:8000/web/vr.html?urdf=so101/so101`.
- **Note**: Ensure the Quest 3 and the computer are on the same Wi-Fi network.

## 3. VR Connection Steps

1. **Open Browser**: Open the browser app in Quest 3.
2. **Enter Address**: Enter the URL address displayed in the terminal.
3. **Login**: 
   - Username: `operator`
   - Password: `operator`
4. **Enter VR**: 
   - Click **Connect to Server** first.
   - Once the status shows "Connected", click **Enter VR Mode**.

## 4. Control Logic

| Button | Description |
| :--- | :--- |
| **Grip Button (Side)** | **Clutch**: **MUST hold this button** for the robot arm to follow your hand movement. Release this button to move your hand freely without affecting the robot's pose. |
| **Index Trigger** | **Gripper Control**: Hold to close, release to open. |
| **B Button (Right)** | **Emergency Stop (E-Stop)**: Immediately disables all motors. |
| **Y Button (Left)** | **Home**: Smoothly returns the robot arm to its initial position. |

## 5. Safety Rules ⚠️

1. **Space Clearance**: Ensure there are no obstacles like water cups, computers, or other objects within the robot arm's range before starting.
2. **Hand Movements**: Avoid sudden, extreme, or high-speed hand movements. Real motors have physical limits, and excessive impact may cause structural damage or motor overheating.
3. **Latency Observation**: If you notice a latency of more than 0.5 seconds in the VR view, stop the operation immediately and check the Wi-Fi signal.
4. **Low Power Warning**: If the robot arm's movements are weak or shaky, check if the 12V power is properly plugged in.

---

*For any questions, please refer to [HANDOVER_MANUAL.md](HANDOVER_MANUAL.md) or contact the development team.*
