# Teleoperation System Latency Testing Guide

This document provides instructions on how to use the `measure_latency.py` tool to test network performance between your local machine and the Teleoperation Server (e.g., hosted on AWS).

## 1. Prerequisites

### Software Requirements
Ensure you have Python 3 installed. You will need the following Python libraries:

```bash
pip install requests websockets
```

### Server Configuration (AWS)
If your server is hosted on AWS EC2, you must configure the **Security Group** to allow the necessary traffic:

1.  **ICMP (Ping)**: Add an Inbound Rule for "All ICMP - IPv4" (or Custom ICMP) from your IP (or 0.0.0.0/0).
    *   *Without this, the Ping test will fail/timeout.*
2.  **TCP Port 8000**: Add an Inbound Rule for "Custom TCP" on Port `8000` (or your configured server port).

## 2. Usage

The tool is located at `measure_latency.py` in the project root.

### Basic Command
Run the script specifying the host IP address of your server:

```bash
python3 measure_latency.py --host <SERVER_IP_ADDRESS>
```

**Example:**
```bash
python3 measure_latency.py --host 44.254.63.252
```

### Arguments
*   `--host`: The IP address or hostname of the server (default: `localhost`).
*   `--port`: The port the server is running on (default: `8000`).

## 3. What It Tests

The tool performs three types of latency measurements:

### 1. Network Layer (ICMP Ping)
*   **What it is:** Sends raw ICMP packets to the server.
*   **Significance:** This is the baseline "speed of light" limit of your network connection. You cannot get faster than this.
*   **Target:** < 50ms for good teleoperation, < 100ms is acceptable.

### 2. Application Layer (HTTP REST)
*   **What it is:** Sends HTTP GET requests to the `/api/v1/status` endpoint.
*   **Significance:** Measures the overhead of the HTTP protocol and the web server's request handling.
*   **Target:** Typically Ping + 10-30ms.

### 3. Control Loop (WebSocket)
*   **What it is:** Opens a WebSocket connection, authenticates as `operator`, and sends dummy control commands. It measures the Round-Trip Time (RTT) for the server to acknowledge (`ack`) each command.
*   **Significance:** This is the **real-world latency** you will feel when controlling the robot. It includes network time + server processing + serialization overhead.
*   **Target:** < 100ms for smooth control.

## 4. Interpreting Results

The script outputs detailed statistics (Min, Max, Average, Jitter) and a summary:

```text
==================================================
SUMMARY
==================================================
Network RTT:  45.20 ms  <-- Baseline Network Latency
HTTP RTT:     65.40 ms  <-- Web Server Response Time
Control Loop: 72.10 ms  <-- Actual Teleop Input Lag
==================================================
```

*   **Jitter:** High jitter (standard deviation) means the connection is unstable, which can cause "stuttering" during robot control.

## 5. Troubleshooting

*   **Ping Failed (Timeout)**:
    *   Check your AWS Security Group. You need to allow "All ICMP - IPv4".
    *   Check if the server IP is correct.
*   **Connection Refused**:
    *   Ensure the server is actually running on the remote machine (`python3 run_server.py ...`).
    *   Check AWS Security Group for Port 8000 (TCP).
*   **Login Failed**:
    *   The script attempts to log in with default credentials (`operator`/`operator`). If you changed the password on the server, update the `measure_latency.py` script (lines 180-181).
