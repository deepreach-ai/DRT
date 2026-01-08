# Cloud Deployment Guide

Since you are waiting for your SSH key, here is a guide to prepare your AWS environment and deploy the system once you have access.

## 1. AWS Prerequisites

Before deploying, ensure your EC2 instance is configured correctly on the AWS Console:

1.  **Instance Type**: `t2.micro` (Free Tier) or `t3.small` is sufficient.
2.  **OS**: Ubuntu 22.04 LTS or 24.04 LTS (Script assumes Ubuntu).
3.  **Security Group (Firewall)**:
    *   **Inbound Rules**:
        *   `SSH (22)`: My IP (for your access).
        *   `Custom TCP (8000)`: Anywhere (0.0.0.0/0) or My IP. This is for the Teleop API.

## 2. Deployment Steps (Once you have the Key)

We have prepared a one-click deployment script `deploy_remote.sh`.

1.  **Save your key**: Place your `.pem` key file somewhere safe (e.g., `~/.ssh/my-key.pem`) and run `chmod 400 ~/.ssh/my-key.pem`.

2.  **Run the script**:
    Run the following command from the `deployment/` directory:

    ```bash
    cd deployment
    chmod +x deploy_remote.sh
    
    # Replace with your actual IP and Key path
    EC2_HOST=1.2.3.4 SSH_KEY=~/.ssh/my-key.pem ./deploy_remote.sh
    ```

    **What this script does:**
    *   Packages your local code.
    *   Uploads it to the EC2 instance.
    *   Installs Docker (if missing).
    *   Builds and starts the Teleop Server.

## 3. Verification

Once the script finishes:

1.  Open your browser or Postman.
2.  Visit `http://<EC2_IP>:8000/docs`.
3.  You should see the Swagger UI.

## 4. Connecting the Client

On your local machine, run the keyboard client pointing to the remote server:

```bash
python client/keyboard_client.py --server http://<EC2_IP>:8000
```
