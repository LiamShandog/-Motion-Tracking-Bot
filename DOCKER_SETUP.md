# Docker Setup for Motion Tracking Bot on Raspberry Pi 5

This guide explains how to containerize and deploy the motion_tracking_bot to a Raspberry Pi 5 using Docker.

## Prerequisites

### On Your Development Machine
- Docker installed (version 20.10+)
- Docker Buildx for cross-platform builds: `docker buildx create --use`
- (Optional) SSH access to your Raspberry Pi

### On Raspberry Pi 5
- Raspberry Pi OS (64-bit Debian-based)
- Docker installed: `curl -fsSL https://get.docker.com -o get-docker.sh && sudo bash get-docker.sh`
- Docker Compose: `sudo apt install docker-compose`
- pigpiod daemon running (optional, can run in container)
- User in docker group: `sudo usermod -aG docker $USER`

## Quick Start

### 1. Build the Docker Image

**For cross-compilation (recommended for development on x86/ARM laptops):**
```bash
chmod +x scripts/docker-build.sh
./scripts/docker-build.sh build
```

**For native build (if building on a Raspberry Pi):**
```bash
./scripts/docker-build.sh build-local
```

### 2. Run Locally (For Testing)

```bash
# Interactive mode (see logs in real-time)
./scripts/docker-build.sh run

# Or using docker-compose directly
docker-compose up --build

# Detached mode (run in background)
./scripts/docker-build.sh run-detached
```

### 3. Deploy to Raspberry Pi

**Option A: Using the deploy script (requires SSH)**
```bash
./scripts/docker-build.sh deploy pi@192.168.1.50
```

**Option B: Manual deployment**
```bash
# 1. Save image on your machine
./scripts/docker-build.sh save

# 2. Copy to Pi
scp motion_tracking_bot_pi5.tar.gz pi@192.168.1.50:/tmp/

# 3. SSH into Pi and load
ssh pi@192.168.1.50
docker load < /tmp/motion_tracking_bot_pi5.tar.gz

# 4. Run on Pi
docker-compose up -d
```

## File Structure

```
motion_tracking_bot/
├── Dockerfile                  # Multi-stage build for ARM64
├── docker-compose.yml          # Compose config with pigpiod service
├── .dockerignore              # Files to exclude from Docker build
├── scripts/
│   └── docker-build.sh        # Build and deployment helper script
├── motion_tracking_bot/       # ROS 2 package
│   ├── ir_sensor_node.py
│   └── speaker_node.py
├── config/
│   └── motion_tracking_bot.yaml
├── launch/
│   └── motion_tracking_bot.launch.py
└── setup.py
```

## Docker Configuration Details

### Dockerfile Features
- **Multi-stage build**: Reduces final image size
- **ARM64 support**: Targets Raspberry Pi 5 (and other ARM64 boards)
- **Optimized layers**: Caches dependencies separately from code
- **pigpiod health check**: Waits for GPIO daemon before starting nodes
- **ROS 2 Humble**: Latest stable ROS 2 distribution

### docker-compose.yml Features
- **Host network mode**: Direct access to host networking (required for ROS communication)
- **Privileged mode**: Allows access to `/dev/mem` and GPIO devices
- **Volume mounts**: 
  - Live code updates during development
  - Persistent ROS logs
  - Device access (`/dev`)
- **Auto-restart**: Container restarts on failure
- **Optional pigpiod service**: Runs pigpiod in container if needed

## Usage Examples

### View Real-Time Logs
```bash
./scripts/docker-build.sh logs
```

### Open Shell in Running Container
```bash
./scripts/docker-build.sh shell
```

### Run a Specific Node
```bash
docker-compose exec motion_tracking_bot ros2 run motion_tracking_bot ir_sensor_node
```

### Run Custom ROS 2 Command
```bash
docker-compose exec motion_tracking_bot ros2 topic echo /ir_sensor
```

### Stop Container
```bash
./scripts/docker-build.sh stop
```

### Clean Up
```bash
./scripts/docker-build.sh clean
```

## Environment Variables

Configure behavior via environment variables in `docker-compose.yml`:

- `ROS_DOMAIN_ID=0` — ROS 2 domain ID (for multi-machine communication)
- `ROS_LOCALHOST_ONLY=0` — Allow network communication
- `SKIP_PIGPIOD_CHECK=0` — Wait for pigpiod before starting (set to 1 to skip)

## Hardware Access

The container runs with `--privileged` mode to access GPIO:
- `/dev/mem` — Memory-mapped GPIO access
- `/dev/gpiomem` — Alternative GPIO access
- `pigpio` uses these to control hardware

### On Raspberry Pi
If running natively on a Raspberry Pi with Docker, ensure:
1. User is in `docker` group: `sudo usermod -aG gpio $USER`
2. pigpiod is enabled: `sudo systemctl enable --now pigpiod`

### On Non-Raspberry Pi Hosts
GPIO hardware won't be available, but you can test the ROS nodes with mock GPIO:
- Modify `ir_sensor_node.py` to skip GPIO in container
- Use test nodes: `ros2 run motion_tracking_bot ir_sensor_node_test`

## Troubleshooting

### "pigpiod not ready" Error
```bash
# Check if pigpiod is running
docker-compose exec motion_tracking_bot pigs t

# If in separate container, check its logs
docker logs pigpiod

# Restart pigpiod
docker-compose restart pigpiod
```

### "Cannot connect to Docker daemon"
```bash
# Add user to docker group
sudo usermod -aG docker $USER
# Log out and back in
```

### ROS 2 Nodes Not Communicating
```bash
# Verify ROS_DOMAIN_ID matches
docker-compose exec motion_tracking_bot env | grep ROS_DOMAIN

# Check if containers are on same network
docker-compose exec motion_tracking_bot ros2 node list
```

### Image Build Fails
```bash
# Clear build cache and rebuild
docker buildx prune -af
./scripts/docker-build.sh build
```

## Performance Optimization

### Build Time
- Use `--cache-from` to reuse layers on subsequent builds
- Pre-built base images speed up builds significantly

### Runtime Performance
- Mount source code as volume for development (not production)
- Remove volume mounts for production deployments
- Use `--cpus` and `--memory` flags to limit container resources

## Production Deployment

### Recommended Setup on Raspberry Pi
1. Build image on your development machine
2. Save and transfer to Pi
3. Use systemd service or cron to auto-start container on boot

Example systemd service (`/etc/systemd/system/motion-tracking-bot.service`):
```ini
[Unit]
Description=Motion Tracking Bot ROS 2 Container
After=docker.service
Requires=docker.service

[Service]
Type=simple
ExecStart=/usr/bin/docker-compose -f /path/to/docker-compose.yml up
ExecStop=/usr/bin/docker-compose -f /path/to/docker-compose.yml down
Restart=unless-stopped
User=pi

[Install]
WantedBy=multi-user.target
```

Then enable:
```bash
sudo systemctl enable --now motion-tracking-bot.service
```

## Next Steps

- Configure `config/motion_tracking_bot.yaml` for your hardware pins
- Test individual nodes with `ros2 run` commands
- Use `rqt_graph` to visualize node communication
- Set up systemd service for automatic startup on Pi boot

## Support

For issues:
1. Check Docker logs: `docker-compose logs -f`
2. Verify hardware connections and GPIO pins
3. Ensure pigpiod is running: `pigs t`
4. Consult ROS 2 documentation: https://docs.ros.org/en/humble/
