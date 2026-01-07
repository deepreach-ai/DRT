# AWS Deployment - One-Page Cheat Sheet

## 🎯 Quick Deploy (3 Commands)

```bash
# 1. Fix key permissions
chmod 400 ~/.ssh/teleop-aws-key.pem

# 2. Deploy (replace YOUR_IP)
cd ~/drt && ./quick_deploy.sh YOUR_IP

# 3. Test latency
python client/latency_test_client.py --server ws://YOUR_IP:8000/ws/v1/teleop?token=e4acc03b3dbb41fbb2608371491f414c
```

---

## 📝 AWS Console Checklist

### Launch Instance Settings:
```
✅ Name: teleop-server-prod
✅ AMI: Ubuntu 22.04 LTS
✅ Instance: t3.medium
✅ Key: Create "teleop-aws-key.pem"
✅ Security: Allow 22, 8000, 8080, 80, 443
✅ Storage: 30 GB gp3
```

---

## 🔗 Important URLs (After Deploy)

```
Web UI:   http://YOUR_IP:8080
API:      http://YOUR_IP:8000
Docs:     http://YOUR_IP:8000/docs
SSH:      ssh -i ~/.ssh/teleop-aws-key.pem ubuntu@YOUR_IP
```

---

## 🚨 Quick Fixes

**Can't SSH:**
```bash
chmod 400 ~/.ssh/teleop-aws-key.pem
```

**Service not running:**
```bash
ssh -i ~/.ssh/teleop-aws-key.pem ubuntu@YOUR_IP
cd ~/drt && source venv/bin/activate
python run_server.py --backend mock --host 0.0.0.0 --port 8000 &
python client/web_server.py &
```

**Check logs:**
```bash
ssh -i ~/.ssh/teleop-aws-key.pem ubuntu@YOUR_IP 'tail -f /tmp/server.log'
```

---

## 📊 Expected Results

```
Local latency:  5.76ms ✅
Network ping:   ~40-60ms (Mexico → US West)
Total latency:  ~45-70ms ✅
Target:         <100ms ✅
```

---

## ✅ Done When:

- [ ] Web UI loads in browser
- [ ] Can connect and control
- [ ] Latency < 100ms
- [ ] Screenshot taken
- [ ] Chris notified

---

**Full guides:** 
- DEPLOY-NOW.md (detailed)
- AWS_DEPLOYMENT_GUIDE.md (complete)
