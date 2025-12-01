# Motion Tracking Bot

This project is a ROS2-based robotics system running on a Raspberry Pi. The system detects motion with an infrared sensor and reacts by waving a servo-driven hand and playing audio.

The goal is to build ROS2 skills through real hardware nodes, while keeping electronics simple.

## Project Features

### 1. IR Motion Detection
- An infrared motion sensor publishes motion events as ROS2 Bool messages
- Topic: `/ir_sensor` (True = motion detected, False = no motion)

### 2. Servo Hand Waving
- When motion is detected, a servo moves a 3D-printed hand
- Configurable wave count and delay via parameters

### 3. Audio Response
- Sound is played when motion is detected
- Audio file path is configurable via parameters

### 4. ROS2 Architecture
- All behavior developed with ROS2 nodes and pub/sub messaging
- Nodes: `ir_sensor_node`, `speaker_node`
- Configuration via YAML parameters

## Hardware

- Raspberry Pi 4 (or later)
- IR motion sensor (e.g., PIR sensor, BCM GPIO pin 17)
- Servo motor (e.g., SG90, BCM GPIO pin 18)
- Speaker (via Pi audio jack or USB)
- Breadboard, jumper wires, resistors

## ROS2 Nodes

### `ir_sensor_node`
- **Function:** Reads IR sensor on GPIO pin and publishes motion events
- **Publishes:** `/ir_sensor` (Bool: True/False)
- **Parameters:**
  - `ir_sensor_pin` (default: 17) — BCM GPIO pin for sensor

### `speaker_node`
- **Function:** Subscribes to IR sensor events and activates servo + audio
- **Subscribes:** `/ir_sensor` (Bool)
- **Parameters:**
  - `servo_pin` (default: 18) — BCM GPIO pin for servo
  - `audio_file` — path to WAV file to play
  - `wave_count` (default: 3) — number of hand waves
  - `wave_delay` (default: 0.3) — seconds between waves
  - `servo_pulse_center` (default: 1500) — servo center pulse in microseconds
  - `servo_pulse_left` (default: 1200) — left pulse
  - `servo_pulse_right` (default: 1800) — right pulse

### Run the system

**Option A: Launch both nodes together**
```bash
ros2 launch motion_tracking_bot motion_tracking_bot.launch.py
```

**Option B: Run nodes individually**
```bash
# Terminal 1: IR sensor node
ros2 run motion_tracking_bot ir_sensor_node

# Terminal 2: Speaker/servo node
ros2 run motion_tracking_bot speaker_node
```

### 6. Monitor the system

In a new terminal (sourced):
```bash
# See active nodes
ros2 node list

# See topics
ros2 topic list

# Watch sensor data in real-time
ros2 topic echo /ir_sensor

# View ROS graph (if rqt installed)
sudo apt install python3-rqt-graph
rqt_graph
```

## Troubleshooting

### Nodes fail with "pigpio not installed"
- Install: `sudo apt install -y pigpio python3-pigpio`
- Restart daemon: `sudo systemctl restart pigpiod`

### Nodes fail with "Failed to connect to pigpiod daemon"
- Check pigpiod is running: `sudo systemctl status pigpiod`
- Start it: `sudo systemctl start pigpiod`
- Test: `pigs t` (should print version info)

### Audio not working
- Check file exists: `ls -l /path/to/your/audio.wav`
- Test audio: `aplay /path/to/your/audio.wav`
- Verify Pi audio is configured (HDMI, headphone jack, or USB)

### Servo not moving
- Check BCM GPIO pin 18 is not in use by another process
- Verify servo connections and power
- Test servo pulse manually: `pigs sp 18 1500` (center), `pigs sp 18 1200` (left), `pigs sp 18 1800` (right)

### Permission errors on GPIO
Add your user to the gpio group (optional):
```bash
sudo usermod -aG gpio $USER
# Log out and log back in
```

## Project Goals
- Work with ROS publishers, subscribers, and parameters
- Learn to control hardware with ROS nodes
- Interface GPIO via pigpio library
- Build foundations in robotics development

To do:
- Set up test nodes to run on laptop (ie. ir_sensor_sim speaker_node_sim) that set fake values and don't depends on pi depencies
- Set up project in Docker container, to be compatible on 64-bit pi OS
- Was about to clone repo to pi