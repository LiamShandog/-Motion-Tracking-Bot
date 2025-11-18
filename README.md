# Motion Tracking Bot
This project is a ROS2 based basic robotics system running on a Pi. The system performs 360 degree spins scanning with an infrared sensor and reacts by waving a servo driven hand and playing audio. The entire sequence is initialized by a button press. 

The goal of this project is to build ROS2 skills through real hardware nodes, while keeping electronics simple.

## Project Features

### 1. Button Activation
- System stays idle until button press.
- After button press, ROS2 nodes begin running.

### 2. 360 IR scanning
- A DC motor continuously rotates an infrared sensor.
- A ROS2 node controls the motor and reads the sensor.
- Motion events are published as ROS2 messages.

### 3. Servo hand waving
- When motion is detected, a servo moves a 3D-printed hand.

### 4. Audio response
- Sound is played when motion is detected.

### 5. ROS2 based
- All behavior is developed with ROS nodes.

## ROS2 Nodes


## Hardware
 - Raspberry Pi
 - IR motion sensor
 - DC Motor
 - Servo Motor
 - Push Button
 - Speaker
 - 3D printed components

 ## Project Goals
 - Work with ROS publishers, subscribers, and parameters
 - Learn to control hardware with ROS
 - Work with Pi through GPIO
 - Build foundations in robotics


## Steps to Run
### Install system packages (ROS, GPIO tools, audio)
sudo apt update
sudo apt install -y pigpio python3-pigpio alsa-utils ros-humble-rclpy ros-humble-std-msgs python3-colcon-common-extensions

### Start pigpiod daemon
sudo systemctl enable --now pigpiod

### Install Python deps from requirements.txt
pip install -r requirements.txt

### Build and run
colcon build
source install/setup.bash
ros2 run motion_tracking_bot ir_sensor_node speaker_node

## To Do
- add sound input to yaml / speaker node files
- See if aplay works, may need to alter speaker node to give speaker an output pin