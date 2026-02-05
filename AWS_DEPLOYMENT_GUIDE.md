# AWS EC2 Deployment Guide - Teleoperation System

## Quick Deploy Checklist

### 1. Launch EC2 Instance

**Via AWS Console:**
1. Go to EC2 Dashboard
2. Click "Launch Instance"
3. Configure:

**Instance Configuration:**
```
Name: teleop-server-prod

AMI: Ubuntu Server 22.04 LTS (HVM), SSD Volume Type
     - 64-bit (x86)
     
Instance Type: t3.medium
     - 2 vCPU, 4 GB RAM
     - Good for initial deployment
     - Can scale up later if needed

Key Pair: 
     - Create new or use existing
     - Download .pem file if new
     - Save to: ~/.ssh/teleop-aws.pem
     
Network Settings:
     ✅ Allow SSH (port 22) - From My IP only
     ✅ Allow Custom TCP (port 8000) - From Anywhere (0.0.0.0/0)
     ✅ Allow Custom TCP (port 8080) - From Anywhere (0.0.0.0/0)
     ✅ Allow HTTPS (port 443) - From Anywhere (future use)

Storage:
     - 30 GB gp3 (General Purpose SSD)

Advanced Details:
     - Leave defaults
```

4. Click "Launch Instance"
5. Wait for "Instance State" = "Running"
6. Note the "Public IPv4 address"

---

### 2. Connect to Instance

```bash
# Set correct permissions on key
chmod 400 ~/.ssh/teleop-aws.pem

# Connect via SSH
ssh -i ~/.ssh/teleop-aws.pem ubuntu@YOUR_PUBLIC_IP

# Test connection
# You should see: ubuntu@ip-xxx-xxx-xxx-xxx:~$
```

---

### 3. Deploy Using Quick Script

**On Your Mac:**
```bash
cd ~/teleop_system

# Make sure script is executable
chmod +x quick_deploy.sh

# Deploy (replace with your actual IP)
./quick_deploy.sh YOUR_PUBLIC_IP
```

**What the script does:**
- ✅ Packages your code
- ✅ Uploads to server
- ✅ Installs dependencies
- ✅ Creates systemd services
- ✅ Starts servers automatically

---

### 4. Verify Deployment

**Test from your browser:**
```
http://YOUR_PUBLIC_IP:8080
```

**Test API:**
```
http://YOUR_PUBLIC_IP:8000/docs
```

**Check service status:**
```bash
ssh -i ~/.ssh/teleop-aws.pem ubuntu@YOUR_PUBLIC_IP

# Check if services are running
sudo systemctl status teleop-server
sudo systemctl status teleop-web

# View logs
sudo journalctl -u teleop-server -f
```

---

### 5. Test Latency

**From your Mac:**
```bash
cd ~/teleop_system

# Test network latency first
ping YOUR_PUBLIC_IP

# Test full system latency
python client/latency_test_client.py --server ws://YOUR_PUBLIC_IP:8000/ws/v1/teleop?token=e4acc03b3dbb41fbb2608371491f414c --samples 200
```

---

## Troubleshooting

### Issue: Can't SSH
```bash
# Check security group allows SSH from your IP
# Verify key permissions: chmod 400 ~/.ssh/teleop-aws.pem
# Try verbose: ssh -v -i ~/.ssh/teleop-aws.pem ubuntu@YOUR_PUBLIC_IP
```

### Issue: Can't Access Web UI (port 8080)
```bash
# Check security group allows port 8080
# Verify service is running:
ssh -i ~/.ssh/teleop-aws.pem ubuntu@YOUR_PUBLIC_IP 'sudo systemctl status teleop-web'
```

### Issue: WebSocket Connection Failed
```bash
# Check security group allows port 8000
# Verify server is running:
ssh -i ~/.ssh/teleop-aws.pem ubuntu@YOUR_PUBLIC_IP 'sudo systemctl status teleop-server'
```

### View Logs
```bash
# Server logs
ssh -i ~/.ssh/teleop-aws.pem ubuntu@YOUR_PUBLIC_IP 'sudo journalctl -u teleop-server -n 50'

# Web UI logs
ssh -i ~/.ssh/teleop-aws.pem ubuntu@YOUR_PUBLIC_IP 'sudo journalctl -u teleop-web -n 50'
```

---

## Alternative: Manual Deployment

If quick_deploy.sh has issues, deploy manually:

```bash
# 1. SSH to server
ssh -i ~/.ssh/teleop-aws.pem ubuntu@YOUR_PUBLIC_IP

# 2. Install dependencies
sudo apt-get update
sudo apt-get install -y python3 python3-pip python3-venv git

# 3. Clone or upload code
# Option A: If code is in git
git clone YOUR_REPO_URL teleop_system

# Option B: Upload from Mac
# (On Mac): scp -i ~/.ssh/teleop-aws.pem -r ~/teleop_system ubuntu@YOUR_PUBLIC_IP:~/

# 4. Setup virtual environment
cd ~/teleop_system
python3 -m venv venv
source venv/bin/activate

# 5. Install Python packages
pip install -r server/requirements.txt
pip install -r client/requirements.txt

# 6. Test manual start
python run_server.py --backend mock --host 0.0.0.0 --port 8000 &
python client/web_server.py &

# 7. Test from browser
# Visit: http://YOUR_PUBLIC_IP:8080

# 8. If working, set up systemd (see quick_deploy.sh for service files)
```

---

## Security Hardening (After Initial Testing)

### Basic Security
```bash
ssh -i ~/.ssh/teleop-aws.pem ubuntu@YOUR_PUBLIC_IP

# Update system
sudo apt-get update && sudo apt-get upgrade -y

# Install fail2ban (SSH protection)
sudo apt-get install -y fail2ban
sudo systemctl enable fail2ban
sudo systemctl start fail2ban

# Configure UFW firewall
sudo ufw default deny incoming
sudo ufw default allow outgoing
sudo ufw allow 22/tcp    # SSH
sudo ufw allow 8000/tcp  # API
sudo ufw allow 8080/tcp  # Web UI
sudo ufw enable
```

### Add Authentication (Recommended)
```python
# In server/teleop_server.py
# Add simple token-based auth:

from fastapi import Header, HTTPException

async def verify_token(authorization: str = Header(None)):
    if authorization != "Bearer YOUR_SECRET_TOKEN":
        raise HTTPException(status_code=401, detail="Unauthorized")
    return True

# Add to endpoints:
@app.post("/api/v1/command", dependencies=[Depends(verify_token)])
```

---

## Cost Estimation

**t3.medium (2 vCPU, 4GB RAM):**
- On-Demand: ~$0.0416/hour = ~$30/month
- Reserved (1 year): ~$0.025/hour = ~$18/month

**Data Transfer:**
- First 100 GB/month: Free
- Additional: $0.09/GB

**Storage:**
- 30 GB gp3: ~$2.40/month

**Total Estimated Cost:** ~$32-35/month

---

## Next Steps After Deployment

1. ✅ Test latency from your location
2. ✅ Test from Mexico VPN
3. ✅ Document results for Chris
4. ✅ Add to TODO list: HTTPS setup (Let's Encrypt)
5. ✅ Consider Elastic IP (static IP, $3.60/month)

---

## Quick Commands Reference

```bash
# Connect to server
ssh -i ~/.ssh/teleop-aws.pem ubuntu@YOUR_PUBLIC_IP

# View server logs
sudo journalctl -u teleop-server -f

# Restart services
sudo systemctl restart teleop-server teleop-web

# Stop services
sudo systemctl stop teleop-server teleop-web

# Check service status
sudo systemctl status teleop-server
sudo systemctl status teleop-web

# Update code
cd ~/teleop_system
git pull  # or re-upload files
sudo systemctl restart teleop-server teleop-web
```

---

## Success Checklist

After deployment, verify:
- [ ] Can SSH to server
- [ ] Web UI loads at http://IP:8080
- [ ] Can connect to robot (Connect button works)
- [ ] Safety activation works (press 1)
- [ ] Commands are being sent (statistics update)
- [ ] Latency test shows <100ms from your location
- [ ] Services auto-restart after reboot
- [ ] Logs are being written

---

## When to Contact Chris

✅ After successful deployment and initial testing
✅ After latency test results are ready
✅ If you need to increase budget for larger instance
⚠️ If latency is higher than expected (>150ms)
⚠️ If you encounter blockers

---

## Bonus: One-Line Deploy

If you want to try a super quick deploy:

```bash
# From your Mac
cd ~/teleop_system
./quick_deploy.sh YOUR_PUBLIC_IP && python client/latency_test_client.py --server ws://YOUR_PUBLIC_IP:8000/ws/v1/teleop?token=e4acc03b3dbb41fbb2608371491f414c
```

This deploys AND tests latency in one command! 🚀
