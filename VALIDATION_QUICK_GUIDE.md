# 🧪 Isaac Sim Video Streaming Validation Guide

This guide explains how to validate that the video stream is correctly flowing from your Remote PC (RTX 4080) to your local VR headset or browser.

## 📋 Prerequisites
*   **Remote PC (RTX 4080)**: Running Isaac Sim and this repository.
*   **Local Device**: Laptop or Quest 3 Headset connected to the same network (or reachable IP).

## 🚀 Step 1: Launch the System on Remote PC

1.  **SSH into your Remote PC**:
    ```bash
    ssh user@<REMOTE_IP>
    cd teleop_system
    ```

2.  **Pull Latest Code** (Important for video fixes):
    ```bash
    git pull origin main
    ```

3.  **Start the Simulation**:
    Use the helper script to start both the Server and Isaac Sim in headless mode.
    ```bash
    ./start_vr_simulation.sh realman_rm65
    ```
    *   **What happens:** 
        *   Starts `teleop_server.py` on port 8000.
        *   Starts `isaac_sim_client.py` which loads the robot and begins capturing frames.
    *   **Wait**: It takes ~1-2 minutes for Isaac Sim to fully load. Watch the logs for "✅ Simulation initialized".

## 🔍 Step 2: Validate Server Connection

On your **Local Device** (Laptop/Quest):

1.  Open a browser (Chrome/Edge recommended).
2.  Navigate to the status endpoint:
    ```
    http://<REMOTE_IP>:8000/api/v1/status
    ```
    *   **Expected Result**: JSON response with `"robot_connected": true` and `"backend_type": "isaac"`.

## 📺 Step 3: Validate Video Stream (Browser)

1.  **Open the MJPEG Stream directly**:
    ```
    http://<REMOTE_IP>:8000/api/v1/video/mjpeg?token=default
    ```
    *   **Expected Result**: You should see a live video feed of the robot in the simulation.
    *   **Troubleshooting**:
        *   If the page loads but image is broken/static: The server is running but receiving no frames from Isaac. Check remote logs for "Error sending state" or "Capture error".
        *   If connection refused: Check firewall/security groups on Remote PC (allow port 8000).

## 🥽 Step 4: Validate in VR (Quest 3)

1.  **Open the VR Interface**:
    Navigate to:
    ```
    http://<REMOTE_IP>:8000/vr.html
    ```

2.  **Connect**:
    *   Enter Server URL: `http://<REMOTE_IP>:8000`
    *   Click **"Connect to Server"**.
    *   Check the "Video Streams" status indicator. It should turn green (1/3 or 3/3).

3.  **Enter VR**:
    *   Click **"Enter VR Mode"**.
    *   You should see the simulation view on the virtual screens in front of you.

## 🛠️ Troubleshooting Checklist

| Issue | Possible Cause | Fix |
| :--- | :--- | :--- |
| **No Video (Black)** | Headless rendering issue | Ensure `start_vr_simulation.sh` uses `--enable-livestream`. |
| **Connection Refused** | Firewall | Run `sudo ufw allow 8000` on Remote PC. |
| **Low Framerate** | Network bandwidth | Check ping to remote PC. Ensure you are on 5GHz WiFi. |
| **"Backend not connected"** | Isaac crash | Check if `isaac_sim_client.py` is still running in the SSH terminal. |

