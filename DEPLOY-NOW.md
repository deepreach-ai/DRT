# 🚀 Deploy to AWS - Simple 5-Step Guide

## Prerequisites
- ✅ AWS account access (Chris gave you)
- ✅ Your Mac
- ✅ 30 minutes

---

## Step 1: Create EC2 Instance (AWS Console)

### 1.1 Go to AWS Console
```
https://console.aws.amazon.com/ec2/
```

### 1.2 Click "Launch Instance" (Orange button)

### 1.3 Fill in these settings:

**Name:** `teleop-server-prod`

**Application and OS Images:**
- Quick Start: Ubuntu
- Ubuntu Server 22.04 LTS (Free tier eligible)
- Architecture: 64-bit (x86)

**Instance type:** `t3.medium`
- 2 vCPU, 4 GiB Memory
- Cost: ~$30/month

**Key pair:**
- Click "Create new key pair"
- Name: `teleop-aws-key`
- Key pair type: RSA
- Private key format: .pem
- Click "Create key pair"
- **IMPORTANT:** Save the downloaded file to `~/.ssh/teleop-aws-key.pem`

**Network settings:**
Click "Edit" and configure:
- ✅ Allow SSH traffic from: Anywhere (0.0.0.0/0)
- ✅ Allow HTTP traffic from the internet
- ✅ Allow HTTPS traffic from the internet

Then click "Add security group rule" TWICE to add:
- Type: Custom TCP
  - Port: 8000
  - Source: Anywhere (0.0.0.0/0)

- Type: Custom TCP  
  - Port: 8080
  - Source: Anywhere (0.0.0.0/0)

**Configure storage:**
- 30 GiB gp3
- (Leave other settings default)

### 1.4 Click "Launch instance" (Orange button)

### 1.5 Wait ~1 minute, then:
- Click "View all instances"
- Find your instance
- Wait until "Instance state" shows "Running"
- Note the "Public IPv4 address" (e.g., 54.123.45.67)

---

## Step 2: Configure SSH Key

On your Mac:
```bash
# Move key to .ssh folder
mv ~/Downloads/teleop-aws-key.pem ~/.ssh/

# Set correct permissions (IMPORTANT!)
chmod 400 ~/.ssh/teleop-aws-key.pem

# Test connection (replace with YOUR IP)
ssh -i ~/.ssh/teleop-aws-key.pem ubuntu@YOUR_PUBLIC_IP

# You should see: ubuntu@ip-xxx-xxx-xxx-xxx:~$
# If it works, type: exit
```

---

## Step 3: Deploy Your Code

### Option A: Using the Quick Deploy Script (Recommended)

```bash
cd ~/teleop_system

# Make script executable
chmod +x quick_deploy.sh

# Deploy (replace with YOUR IP)
./quick_deploy.sh YOUR_PUBLIC_IP
```

### Option B: Manual Deploy (If script fails)

```bash
# Package code
cd ~/teleop_system
tar -czf /tmp/teleop.tar.gz --exclude='.git' --exclude='.venv' --exclude='__pycache__' .

# Upload
scp -i ~/.ssh/teleop-aws-key.pem /tmp/teleop.tar.gz ubuntu@YOUR_PUBLIC_IP:/tmp/

# SSH and setup
ssh -i ~/.ssh/teleop-aws-key.pem ubuntu@YOUR_PUBLIC_IP

# On server:
sudo apt-get update
sudo apt-get install -y python3 python3-pip python3-venv
mkdir -p ~/teleop_system
cd ~/teleop_system
tar -xzf /tmp/teleop.tar.gz
python3 -m venv venv
source venv/bin/activate
pip install -r server/requirements.txt
pip install -r client/requirements.txt

# Start servers (temporary)
nohup python run_server.py --backend mock --host 0.0.0.0 --port 8000 > /tmp/server.log 2>&1 &
nohup python client/web_server.py > /tmp/web.log 2>&1 &

# Exit SSH
exit
```

---

## Step 4: Test Your Deployment

### 4.1 Test in Browser
Open: `http://YOUR_PUBLIC_IP:8080`

You should see your web UI! 🎉

### 4.2 Test Connection
- Click "Connect to Robot"
- Press "1" key or click "⚡ Activate"
- Try moving with keyboard (W/A/S/D)
- Check statistics update

### 4.3 Test Latency
On your Mac:
```bash
cd ~/teleop_system

# First test network ping
ping YOUR_PUBLIC_IP

# Then test full system latency
python client/latency_test_client.py \
  --server "ws://YOUR_PUBLIC_IP:8000/ws/v1/teleop?token=e4acc03b3dbb41fbb2608371491f414c" \
  --samples 200
```

---

## Step 5: Document Results

Create a quick report:

```markdown
# AWS Deployment Results

Date: [Today's date]
Server: YOUR_PUBLIC_IP
Region: us-west-1 (or whichever you chose)

## Latency Results:
- Network Ping: XX ms
- Average Latency: XX ms
- P95 Latency: XX ms
- Assessment: [Excellent/Good/Acceptable]

## Status:
✅ Server deployed
✅ Web UI accessible
✅ WebSocket connection working
✅ Control commands functional

## URLs:
- Web UI: http://YOUR_PUBLIC_IP:8080
- API: http://YOUR_PUBLIC_IP:8000
- API Docs: http://YOUR_PUBLIC_IP:8000/docs
```

---

## ✅ Success Checklist

After completing all steps:
- [ ] EC2 instance running
- [ ] Can SSH to server
- [ ] Web UI loads in browser
- [ ] Can connect and control robot
- [ ] Latency test completed
- [ ] Results documented

---

## 🚨 Troubleshooting

### Can't SSH:
```bash
# Check key permissions
ls -la ~/.ssh/teleop-aws-key.pem
# Should show: -r-------- (400)

# If not:
chmod 400 ~/.ssh/teleop-aws-key.pem
```

### Can't access Web UI:
1. Check AWS Security Group has port 8080 open
2. SSH to server and check if service is running:
```bash
ssh -i ~/.ssh/teleop-aws-key.pem ubuntu@YOUR_PUBLIC_IP
ps aux | grep python
```

### WebSocket won't connect:
1. Check AWS Security Group has port 8000 open
2. Check browser console for errors (F12 → Console)

---

## 📊 What to Tell Chris

After successful deployment:

```
Hi Chris!

AWS deployment complete! 🎉

Server: YOUR_PUBLIC_IP
Region: us-west-1

Results:
✅ Web UI: http://YOUR_PUBLIC_IP:8080
✅ Latency: XX ms (under 100ms target)
✅ All systems functional

Ready for testing from Mexico location!

Cost: ~$30/month for t3.medium instance
```

---

## 🎯 Next Steps (After This Works)

1. [ ] Test from Mexico VPN or location
2. [ ] Add HTTPS/SSL (Let's Encrypt)
3. [ ] Set up monitoring (CloudWatch)
4. [ ] Create server backup/snapshot
5. [ ] Document for operators

---

## 💡 Pro Tips

- **Save your IP:** Add it to your notes, you'll need it often
- **Keep terminal open:** Useful for checking logs
- **Take screenshots:** Document your progress for Chris
- **Test thoroughly:** Better to find issues now than during demo

---

## 🆘 Need Help?

If stuck:
1. Check AWS Console → EC2 → Security Groups (ports open?)
2. SSH to server and check logs: `sudo journalctl -xe`
3. Restart services manually:
```bash
ssh -i ~/.ssh/teleop-aws-key.pem ubuntu@YOUR_PUBLIC_IP
cd ~/teleop_system
source venv/bin/activate
python run_server.py --backend mock --host 0.0.0.0 --port 8000
```

---

**Time estimate:** 20-30 minutes for first-time deployment

**Let's go! 🚀**
