---
sidebar_position: 5
---

# Appendix A: Hardware Setup Guide

This appendix provides step-by-step instructions for setting up the hardware required to run Physical AI & Humanoid Robotics projects locally.

## The Economy Jetson Student Kit

### Components Overview

| Component        | Model                               | Price     | Purpose                              |
| ---------------- | ----------------------------------- | --------- | ------------------------------------ |
| **Brain**        | NVIDIA Jetson Orin Nano Super (8GB) | $249      | Inference engine for deployed models |
| **Eyes**         | Intel RealSense D435i               | $349      | RGB + Depth + IMU sensor             |
| **Microphone**   | ReSpeaker USB Mic Array v2.0        | $69       | Voice input for commands             |
| **Storage**      | 128GB microSD Card (high-endurance) | $30       | OS and model storage                 |
| **Power/Cables** | USB-C power supply + cables         | $50       | Power management                     |
| **TOTAL**        | —                                   | **~$750** | Complete AI-vision edge kit          |

---

## Setup Steps

### 1. Jetson Orin Nano Assembly

**What You Need:**

- Jetson Orin Nano dev kit (includes heat sink)
- 128GB microSD card (high-endurance recommended)
- Micro-USB power supply (5V, ≥4A)
- Ubuntu 22.04 LTS image for Jetson

**Steps:**

```bash
# 1. Flash microSD card on your main computer
# Download JetPack 6.0: https://developer.nvidia.com/jetpack

# 2. Use Balena Etcher to write image to card
# Download: https://www.balena.io/etcher/

# 3. Insert microSD into Jetson
# 4. Connect power
# 5. First boot takes 5-10 minutes (wait for LED to stabilize)

# 6. Connect via SSH
ssh ubuntu@192.168.1.X  # Find IP from your router
# Default password: ubuntu (change immediately!)
```

### 2. ROS 2 on Jetson

```bash
# Install ROS 2 Humble (native for Ubuntu 22.04)
sudo apt update
sudo apt install -y ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. RealSense Camera Setup

```bash
# Install librealsense2
sudo apt update
sudo apt install -y ros-humble-librealsense2-wrapper \
                     ros-humble-realsense2-camera

# Test camera connection
rs-enumerate-devices

# Launch ROS 2 driver
ros2 launch realsense2_camera rs_launch.py
```

### 4. ReSpeaker Microphone Setup

```bash
# Install audio drivers
sudo apt install -y alsa-utils pulseaudio
arecord -l  # List recording devices

# Test microphone
arecord -d 5 test.wav
aplay test.wav
```

---

## Hardware Troubleshooting

### Jetson Not Booting

- **Check:** Micro-USB power supply (5V, ≥4A)
- **Fix:** LED should be solid green when powered. If flashing, reboot.
- **Last resort:** Re-flash microSD card with JetPack image

### RealSense Camera Not Detected

```bash
# Check USB connection
lsusb | grep Intel

# Try different USB port (3.0 recommended)
# If still not detected, update firmware:
sudo apt install -y librealsense2-utils
rs-fw-update
```

### High CPU/Temperature

- Ensure heat sink is properly mounted
- Check ambient temperature
- Add heatsink compound if needed

---

## Performance Baseline

| Task                        | Hardware         | Performance     |
| --------------------------- | ---------------- | --------------- |
| ROS 2 node (simple pub/sub) | Jetson Nano 8GB  | 100+ Hz         |
| RealSense streaming         | RealSense D435i  | 30 FPS          |
| VSLAM (Isaac ROS)           | Jetson Orin Nano | 15-20 FPS       |
| LLM inference (small model) | Jetson Nano      | 5-10 tokens/sec |

---

## Deploying Your Code

```bash
# 1. Build ROS 2 package on Jetson
cd ~/ros2_ws
colcon build --packages-select my_robot_controller

# 2. Source and run
source install/setup.bash
ros2 run my_robot_controller controller_node

# 3. For production, use systemd service
sudo nano /etc/systemd/system/robot-controller.service

# [Unit]
# Description=My Robot Controller
# After=network.target
#
# [Service]
# Type=simple
# User=ubuntu
# ExecStart=/home/ubuntu/run_robot.sh
# Restart=on-failure
#
# [Install]
# WantedBy=multi-user.target
```

---

## Next Steps

- Deploy Module 1 (ROS 2) code to your Jetson
- Integrate Module 2 (Gazebo) simulations
- Run Module 3 (Isaac) inference models
- Test Module 4 (VLA) voice commands

---

**Last Updated:** December 7, 2025
