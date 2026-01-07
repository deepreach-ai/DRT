# Network Streaming Setup (ngrok)

For Meta Quest 3 and other remote devices, using `ngrok` is the recommended way to expose your local server securely over HTTPS. This is required for WebXR support on Quest browsers.

## 1. Install ngrok

Download and install ngrok from [ngrok.com/download](https://ngrok.com/download).

Mac (via Homebrew):
```bash
brew install ngrok/ngrok/ngrok
```

## 2. Authenticate

Sign up for a free ngrok account at [dashboard.ngrok.com](https://dashboard.ngrok.com/get-started/your-authtoken) and copy your authtoken.

Run the following command in your terminal (using the local binary):
```bash
./bin/ngrok config add-authtoken <your-token>
```
Replace `<your-token>` with the actual token string starting with `2...`.

## 3. Run ngrok

Start ngrok pointing to your local server port (8000):

```bash
./scripts/run_ngrok.sh
# OR manually:
ngrok http 8000
```

You will see output like:
```
Forwarding                    https://1234-56-78-90.ngrok-free.app -> http://localhost:8000
```

## 4. Connect VR Client

1.  Put on your Meta Quest 3.
2.  Open the Meta Browser.
3.  Navigate to the ngrok URL with `/web/vr.html` appended:
    ```
    https://1234-56-78-90.ngrok-free.app/web/vr.html
    ```
4.  **Important**: In the login screen, the "Server URL" should automatically match the ngrok URL (e.g., `https://1234-56-78-90.ngrok-free.app`). If not, enter it manually.
5.  Login with `operator` / `operator`.
6.  Click "Enter VR".

## Notes

- **Latency**: Network streaming adds latency compared to local Wi-Fi. Ensure you have a good internet connection.
- **Safety**: Do not share your ngrok URL publicly as it allows control of your robot.
