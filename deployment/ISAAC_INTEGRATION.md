# Isaac Sim Integration Guide

## 🎯 Overview

This guide covers integrating Isaac Sim with your teleoperation system on AWS EC2 g5.4xlarge.

## 📁 Architecture

```
┌─────────────────┐         ┌──────────────────┐
│ Keyboard Client │────────▶│ Teleop Server    │
│ (Local Machine) │  HTTP   │ (Port 8000)      │
└─────────────────┘         └────────┬─────────┘
                                     │ TCP
                                     │ Port 9000
                            ┌────────▼─────────┐
                            │ Isaac Sim Client │
                            │ (Franka Robot)   │
                            └──────────────────┘
```

## 🚀 Deployment Options

### Option 1: Full System with Docker Compose (Recommended)

**Start everything:**
```bash
cd /home/ubuntu/teleop_system/deployment
chmod +x start_full_system.sh
./start_full_system.sh
```

**What it does:**
- Starts teleop-server container (FastAPI + Isaac Backend)
- Starts isaac-sim container (NVIDIA Isaac Sim with Franka robot)
- Connects them via Docker network

**Check status:**
```bash
docker-compose -f docker-compose.full.yml ps
docker logs -f isaac-sim
curl http://localhost:8000/api/v1/status
```

**Stop:**
```bash
docker-compose -f docker-compose.full.yml down
```

---

### Option 2: Server Only (Mock Backend for Testing)

**Start mock server:**
```bash
cd /home/ubuntu/teleop_system/deployment
export TELEOP_BACKEND=mock
docker-compose up -d
```

**Test:**
```bash
curl http://localhost:8000/api/v1/status
```

---

### Option 3: Native Isaac Sim (Without Docker)

If you prefer to run Isaac Sim natively:

**1. Install Isaac Sim:**
```bash
# Download from https://developer.nvidia.com/isaac-sim
# Or use Omniverse Launcher
cd ~
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

**2. Start teleop server:**
```bash
cd /home/ubuntu/teleop_system
python run_server.py --backend isaac --host 0.0.0.0 --port 8000
```

**3. Start Isaac Sim client:**
```bash
# Use Isaac Sim's Python
~/isaac_sim/python.sh isaac_sim_client.py \
  --host localhost \
  --port 9000 \
  --robot_usd Franka/franka.usd \
  --ee_frame panda_hand \
  --headless
```

---

## 🔍 Verification Steps

### 1. Check Teleop Server
```bash
# Health check
curl http://localhost:8000/

# API documentation
curl http://localhost:8000/docs

# Current status
curl http://localhost:8000/api/v1/status
```

Expected response:
```json
{
  "is_active": false,
  "safety_gate_active": false,
  "robot_connected": true,
  "backend_type": "isaac",
  "workspace_violation": false,
  "velocity_violation": false
}
```

### 2. Check Isaac Backend Connection
```bash
# Check if port 9000 is listening
netstat -tuln | grep 9000

# Check Isaac Sim logs
docker logs isaac-sim | tail -20
```

Look for:
- `[IsaacBackend] Listening on 0.0.0.0:9000`
- `[IsaacBackend] Client connected from ...`

### 3. Send Test Command
```bash
curl -X POST http://localhost:8000/api/v1/command \
  -H "Content-Type: application/json" \
  -d '{
    "dx": 0.01,
    "dy": 0.0,
    "dz": 0.0,
    "droll": 0.0,
    "dpitch": 0.0,
    "dyaw": 0.0,
    "max_velocity": 0.1,
    "timestamp": '$(date +%s.%N)'
  }'
```

---

## 🎮 Connect from Local Machine

**Install client:**
```bash
cd /path/to/teleop_system
pip install -r client/requirements.txt
```

**Run keyboard client:**
```bash
python client/keyboard_client.py --server http://<EC2_PUBLIC_IP>:8000
```

**Controls:**
- `W/S` - Move forward/backward (X)
- `A/D` - Move left/right (Y)
- `Q/E` - Move up/down (Z)
- `I/K` - Pitch up/down
- `J/L` - Yaw left/right
- `U/O` - Roll left/right
- `Space` - Hold for safety (deadman switch)
- `R` - Reset position
- `ESC` - Quit

---

## ⚙️ Configuration

### Environment Variables

**For docker-compose:**
```bash
# .env file
TELEOP_BACKEND=isaac
TELEOP_ISAAC_HOST=0.0.0.0
TELEOP_ISAAC_PORT=9000
```

**For run_server.py:**
```bash
export TELEOP_BACKEND=isaac
python run_server.py
```

### Robot Models

Change robot model in `docker-compose.full.yml`:

```yaml
command: >
  python /workspace/isaac_sim_client.py 
    --robot_usd UR10/ur10.usd  # Change this
    --ee_frame tool0           # Change this
```

**Available robots:**
- `Franka/franka.usd` (end-effector: `panda_hand`)
- `UR10/ur10.usd` (end-effector: `tool0`)
- `Kuka/kuka.usd` (end-effector: `tool_link`)

---

## 🐛 Troubleshooting

### Issue: Isaac Sim container fails to start

**Check GPU:**
```bash
docker run --rm --gpus all nvidia/cuda:12.1.0-base-ubuntu22.04 nvidia-smi
```

**Fix NVIDIA Container Toolkit:**
```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### Issue: Connection refused on port 9000

**Check if backend is listening:**
```bash
docker exec teleop-server netstat -tuln | grep 9000
```

**Check firewall:**
```bash
sudo ufw status
sudo ufw allow 9000/tcp
```

### Issue: Isaac Sim crashes immediately

**Check logs:**
```bash
docker logs isaac-sim
```

**Common causes:**
- Insufficient GPU memory (g5.4xlarge has 16GB, should be OK)
- Missing NVIDIA drivers
- Wrong robot USD path

**Try with smaller cache:**
```bash
docker system prune -a  # Clean Docker cache
```

### Issue: High latency

**Check network:**
```bash
# On local machine
ping <EC2_IP>
time curl http://<EC2_IP>:8000/
```

**Expected latency:** < 50ms for nearby regions

---

## 📊 Performance Monitoring

**Check resource usage:**
```bash
# CPU and memory
docker stats

# GPU usage
nvidia-smi -l 1
```

**Expected usage:**
- Teleop Server: ~100MB RAM, <5% CPU
- Isaac Sim: ~2-4GB GPU memory, 30-60% GPU utilization

---

## 🔄 Updates and Maintenance

**Update code:**
```bash
cd /home/ubuntu/teleop_system
git pull
docker-compose -f deployment/docker-compose.full.yml down
docker-compose -f deployment/docker-compose.full.yml up -d --build
```

**Clean old images:**
```bash
docker system prune -a
```

**Backup configuration:**
```bash
tar -czf teleop_backup_$(date +%Y%m%d).tar.gz \
  server/ client/ deployment/ isaac_sim_client.py
```

---

## 📚 Additional Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
- [Isaac Core API](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html)
- [Franka Robot USD](https://docs.omniverse.nvidia.com/isaacsim/latest/features/robots/franka.html)

---

## 🆘 Support

If you encounter issues:

1. Check logs: `docker logs isaac-sim` and `docker logs teleop-server`
2. Verify GPU: `nvidia-smi`
3. Test connectivity: `curl http://localhost:8000/api/v1/status`
4. Review this guide's troubleshooting section
