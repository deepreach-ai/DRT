# Deployment & Latency Testing - Detailed Tasks

## 🚀 Phase 1: Cloud Deployment Setup (Priority: CRITICAL)

### Infrastructure Setup
- [ ] **Choose cloud provider** (AWS vs Azure vs GCP)
  - [ ] Compare pricing for your use case
  - [ ] Check Mexico → USA data center locations
  - [ ] Verify GPU availability for video encoding (later)
  
- [ ] **Provision server instance**
  - [ ] Instance type: Recommend t3.medium (start small, scale up)
  - [ ] OS: Ubuntu 22.04 LTS
  - [ ] Region: US-East or US-West (depending on robot location)
  - [ ] Storage: 50GB+ SSD
  
- [ ] **Network configuration**
  - [ ] Static public IP address
  - [ ] DNS setup (optional but recommended)
  - [ ] Open required ports:
    - [ ] Port 8000 (FastAPI server)
    - [ ] Port 8080 (Web UI server)
    - [ ] Port 443 (HTTPS - future)
  
- [ ] **Security groups / Firewall**
  - [ ] Allow inbound: 8000, 8080, 443, 22 (SSH)
  - [ ] Restrict SSH to your IP only
  - [ ] Set up fail2ban for SSH protection

### Server Setup
- [ ] **Install dependencies**
  ```bash
  - [ ] Python 3.10+
  - [ ] Git
  - [ ] pip, venv
  - [ ] nginx (for reverse proxy - optional)
  - [ ] supervisor (for process management)
  ```

- [ ] **Clone and setup project**
  ```bash
  - [ ] git clone your repo
  - [ ] Create virtual environment
  - [ ] pip install requirements
  - [ ] Test local startup
  ```

- [ ] **Environment configuration**
  - [ ] Create .env file with production settings
  - [ ] Set backend type (mock for now)
  - [ ] Configure logging paths
  - [ ] Set allowed origins for CORS

- [ ] **Process management**
  - [ ] Create systemd service for teleop server
  - [ ] Create systemd service for web UI
  - [ ] Enable auto-restart on failure
  - [ ] Set up logging to /var/log/

### Domain & SSL (Optional but Recommended)
- [ ] **Domain setup**
  - [ ] Purchase domain or use subdomain
  - [ ] Point DNS to server IP
  - [ ] Example: teleop.yourcompany.com
  
- [ ] **SSL certificate**
  - [ ] Use Let's Encrypt (free)
  - [ ] Install certbot
  - [ ] Configure HTTPS
  - [ ] Auto-renewal setup

### Reverse Proxy (Recommended)
- [ ] **Nginx setup**
  - [ ] Install nginx
  - [ ] Configure reverse proxy for port 8000
  - [ ] Configure reverse proxy for port 8080
  - [ ] WebSocket proxy settings
  - [ ] Rate limiting
  - [ ] Request logging

---

## 📊 Phase 2: Latency Testing (Priority: CRITICAL)

### Pre-deployment Testing
- [ ] **Local baseline measurements**
  - [ ] Measure localhost WebSocket latency
  - [ ] Measure localhost command processing time
  - [ ] Document baseline metrics (should be <10ms)
  - [ ] Test under load (100+ commands/sec)

### Mexico → USA Latency Testing
- [ ] **Network latency measurement**
  - [ ] Use ping to measure ICMP latency
  - [ ] Use traceroute to identify hops
  - [ ] Test at different times of day
  - [ ] Document peak vs off-peak latency
  
- [ ] **WebSocket latency testing**
  - [ ] Create latency test client script
  - [ ] Send timestamped commands
  - [ ] Measure round-trip time
  - [ ] Test from Mexico location (or VPN)
  - [ ] Target: <100ms round-trip
  
- [ ] **Video streaming latency** (when camera available)
  - [ ] Measure camera capture → encoding → network → browser
  - [ ] Target: <150ms end-to-end
  - [ ] Test different encoding settings
  - [ ] Test different resolutions

### Load Testing
- [ ] **Single operator stress test**
  - [ ] 100 commands/second sustained
  - [ ] Memory usage monitoring
  - [ ] CPU usage monitoring
  - [ ] Network bandwidth usage
  
- [ ] **Multi-operator simulation**
  - [ ] 2 simultaneous operators
  - [ ] 5 simultaneous operators (future)
  - [ ] Measure latency degradation
  - [ ] Identify bottlenecks

### Latency Optimization
- [ ] **If latency > 100ms, try:**
  - [ ] Switch to closer data center
  - [ ] Optimize WebSocket frame size
  - [ ] Enable compression (zlib/gzip)
  - [ ] Reduce command frequency if needed
  - [ ] Use UDP for control (future - more complex)

---

## 🔧 Phase 3: Monitoring & Observability

### Server Monitoring
- [ ] **Install monitoring tools**
  - [ ] htop for CPU/memory
  - [ ] iotop for disk I/O
  - [ ] iftop for network
  - [ ] Or use cloud provider's monitoring
  
- [ ] **Application monitoring**
  - [ ] Log all WebSocket connections
  - [ ] Log all commands with timestamps
  - [ ] Log all errors/exceptions
  - [ ] Track command processing times
  
- [ ] **Metrics dashboard** (Optional but useful)
  - [ ] Install Prometheus + Grafana
  - [ ] Or use cloud monitoring (CloudWatch, Azure Monitor)
  - [ ] Track: latency, throughput, errors, uptime
  
### Alerting
- [ ] **Set up alerts for:**
  - [ ] Server down (ping failure)
  - [ ] High latency (>200ms)
  - [ ] High CPU/memory usage
  - [ ] Disk space low
  - [ ] Application crashes

---

## 🧪 Phase 4: Validation & Testing

### Functional Testing
- [ ] **Deploy checklist**
  - [ ] Web UI loads from public URL
  - [ ] WebSocket connects successfully
  - [ ] Commands are processed correctly
  - [ ] Safety system works
  - [ ] Statistics display correctly
  - [ ] Reconnection works after disconnect
  
- [ ] **Cross-browser testing**
  - [ ] Chrome (primary)
  - [ ] Firefox
  - [ ] Safari (if targeting Mac users)
  - [ ] Mobile browsers (iOS Safari, Chrome Android)
  
- [ ] **Network failure testing**
  - [ ] Simulate packet loss
  - [ ] Simulate high latency (>500ms)
  - [ ] Test auto-reconnect
  - [ ] Verify graceful degradation

### Performance Testing
- [ ] **Latency test results documentation**
  - [ ] Average latency: ____ ms
  - [ ] P95 latency: ____ ms
  - [ ] P99 latency: ____ ms
  - [ ] Max observed latency: ____ ms
  
- [ ] **Throughput testing**
  - [ ] Commands/second: ____
  - [ ] Concurrent users: ____
  - [ ] Bandwidth usage: ____ Mbps

---

## 📝 Phase 5: Documentation & Handoff

### Deployment Documentation
- [ ] **Write deployment guide**
  - [ ] Server setup steps
  - [ ] Configuration options
  - [ ] Environment variables
  - [ ] Port requirements
  - [ ] Troubleshooting common issues
  
- [ ] **Access credentials**
  - [ ] Server SSH keys
  - [ ] Domain access (if applicable)
  - [ ] Cloud console access
  - [ ] Share securely with team

### Operator Documentation
- [ ] **Write user manual**
  - [ ] How to access the system
  - [ ] How to connect
  - [ ] Control instructions
  - [ ] What to do if connection drops
  - [ ] Emergency stop procedure
  
- [ ] **Create video tutorial** (Optional)
  - [ ] Screen recording of full workflow
  - [ ] Narration in English/Spanish
  - [ ] Upload to shared location

---

## 🚨 Additional Critical Tasks

### Security Hardening
- [ ] **Basic security measures**
  - [ ] Change default SSH port
  - [ ] Disable root SSH login
  - [ ] Set up SSH key authentication only
  - [ ] Install fail2ban
  - [ ] Configure UFW firewall
  - [ ] Regular security updates (unattended-upgrades)
  
- [ ] **Application security**
  - [ ] Add rate limiting to prevent abuse
  - [ ] Input validation on all commands
  - [ ] CORS whitelist (don't use allow_origins=["*"] in production)
  - [ ] Add authentication (even basic auth is better than nothing)

### Backup & Recovery
- [ ] **Backup strategy**
  - [ ] Code: Git repository (already done)
  - [ ] Configuration: Backup .env files
  - [ ] Session logs: Backup to S3/Azure Blob
  - [ ] Database: If you add one later
  
- [ ] **Recovery plan**
  - [ ] Document how to restore from backup
  - [ ] Test recovery procedure
  - [ ] Keep backup of server setup script

### Cost Management
- [ ] **Monitor cloud costs**
  - [ ] Set up billing alerts
  - [ ] Track monthly spending
  - [ ] Optimize instance size if needed
  - [ ] Use reserved instances for savings (if long-term)

---

## 📊 Success Criteria

### Deployment Success
- ✅ Server accessible from public internet
- ✅ Web UI loads without errors
- ✅ WebSocket connection stable for 1+ hour
- ✅ No crashes during 10+ test sessions

### Latency Success
- ✅ Mexico → USA latency < 100ms (P95)
- ✅ Command processing < 50ms
- ✅ No packet loss under normal conditions
- ✅ Acceptable performance at different times of day

### Production Ready
- ✅ Auto-restart on failure
- ✅ Logs are being written
- ✅ Monitoring in place
- ✅ Documentation complete
- ✅ Chris can access and test independently

---

## 🎯 This Week Timeline

### Day 1-2: Deploy
- Set up cloud server
- Deploy application
- Test basic functionality

### Day 3: Latency Testing
- Run comprehensive latency tests
- Document results
- Identify issues

### Day 4: Optimization
- Fix any latency issues
- Harden security
- Set up monitoring

### Day 5: Validation & Demo
- Final testing
- Prepare demo for Chris
- Document everything

---

## 🛠️ Quick Start Commands

### Deploy to Server (Example)
```bash
# On your Mac - package the code
cd ~/drt
git push origin main  # or zip the directory

# On server - deploy
ssh user@your-server-ip
git clone your-repo
cd drt
python3 -m venv venv
source venv/bin/activate
pip install -r server/requirements.txt
pip install -r client/requirements.txt

# Start services
python run_server.py --backend mock &
python client/web_server.py &

# Or use systemd (better for production)
sudo systemctl start teleop-server
sudo systemctl start teleop-web
```

### Test Latency from Mexico
```bash
# From Mexico location (or use VPN)
ping your-server-ip  # Basic network latency

# WebSocket latency test (create this script)
python latency_test_client.py --server ws://your-server:8000/ws/v1/teleop
```

---

## 💡 Pro Tips

1. **Start with smallest instance** - You can always scale up
2. **Test from Mexico early** - VPN or ask Mexico colleague
3. **Monitor everything** - Logs are your friend
4. **Document as you go** - Don't wait until the end
5. **Backup before changes** - Snapshots are cheap
6. **Use screen/tmux** - Keep processes running during SSH disconnect

---

## 📞 When to Contact Chris

- ✅ After deployment success
- ✅ After latency test results
- ⚠️ If latency > 150ms (may need strategy change)
- ⚠️ If major blockers encountered
- ✅ When ready for him to test

---

**Priority Order:**
1. 🔴 Deploy to cloud (Days 1-2)
2. 🔴 Test Mexico → USA latency (Day 3)
3. 🟡 Security hardening (Day 4)
4. 🟡 Monitoring setup (Day 4)
5. 🟢 Documentation (Day 5)
