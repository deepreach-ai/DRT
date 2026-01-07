# Quest 3 Sync Guide (WebXR via Ngrok)

This guide explains how to connect your **Meta Quest 3** to the Teleoperation System running on a remote PC (e.g., RTX 4080) or local machine.

⚠️ **Requirement**: Meta Quest 3 WebXR requires an **HTTPS** connection (secure context) to access VR features like hand tracking and controller input. We use `ngrok` to create a secure tunnel.

## 1. Setup on Server PC

1.  **Install ngrok**:
    *   **Linux**: `curl -s https://ngrok-agent.s3.amazonaws.com/ngrok.asc | sudo tee /etc/apt/trusted.gpg.d/ngrok.asc >/dev/null && echo "deb https://ngrok-agent.s3.amazonaws.com buster main" | sudo tee /etc/apt/sources.list.d/ngrok.list && sudo apt update && sudo apt install ngrok`
    *   **Mac/Windows**: Download from [ngrok.com](https://ngrok.com/download).

2.  **Authenticate ngrok** (First time only):
    *   Sign up at ngrok.com to get your auth token.
    *   Run: `ngrok config add-authtoken <YOUR_TOKEN>`

3.  **Start the DRT**:
    ```bash
    ./start_vr_simulation.sh realman_rm65
    ```
    (Ensure it is running on port 8000)

4.  **Start the ngrok Tunnel**:
    Open a *new terminal* and run:
    ```bash
    ngrok http 8000
    ```

5.  **Get the Public URL**:
    Ngrok will display a URL like:
    `Forwarding                    https://a1b2-c3d4.ngrok-free.app -> http://localhost:8000`
    
    Copy this **HTTPS URL**.

## 2. Connect on Meta Quest 3

1.  Put on your Quest 3 headset.
2.  Open the **Meta Quest Browser**.
3.  Type the **HTTPS URL** you copied from ngrok (e.g., `https://a1b2-c3d4.ngrok-free.app`).
    *   *Note: You might see a warning page from ngrok ("You are about to visit..."). Click "Visit Site".*
4.  Navigate to the **VR Page**:
    *   Append `/vr.html` to the URL or click the link if you are on the homepage.
    *   Final URL example: `https://a1b2-c3d4.ngrok-free.app/vr.html`
5.  **Connect**:
    *   In the Web UI, ensure the "Server URL" field matches the ngrok URL (e.g., `https://a1b2-c3d4.ngrok-free.app`).
    *   Click **"Connect to Server"**.
    *   Verify the status indicators turn green.
6.  **Enter VR**:
    *   Click the **"Enter VR Mode"** button at the bottom right.
    *   Select **"Allow"** if asked for permissions (immersive mode).

## 3. Sync Validation

*   **Visuals**: You should see the Isaac Sim camera view inside the VR headset.
*   **Control**: Moving your controllers should move the robot in Isaac Sim.

---

### Troubleshooting
*   **"Enter VR" button is disabled/hidden**: This means the site is not loaded via HTTPS or localhost. Ensure you are using the `https://` ngrok link.
*   **Latency is high**: Ngrok free tier adds some latency. For local network (LAN), use self-signed certificates (complex) or a dedicated low-latency tunnel solution (e.g., LocalTunnel, Tailscale).
*   **Video is black**: Check if the server is receiving frames from Isaac Sim (`./start_vr_simulation.sh` output).
