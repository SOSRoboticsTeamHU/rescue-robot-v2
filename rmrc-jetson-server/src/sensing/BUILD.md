# LiDAR Stream Node - Build and Run Instructions

## Prerequisites

- ROS2 Humble installed
- Python 3 (3.8+)
- Required Python packages (install via pip or rosdep):
  - `pyyaml`
  - `pyzmq`
  - `msgpack`
  - `rclpy` (ROS2 Python client library)

## Installing Dependencies

```bash
# Install system dependencies (if not already installed)
sudo apt update
sudo apt install python3-pip python3-yaml

# Install Python dependencies
pip3 install pyzmq msgpack pyyaml

# ROS2 rclpy should be installed with ROS2 Humble
# Verify: ros2 --version
```

## Setup ROS2 Workspace

If using ROS2 workspace structure (`rmrc_ws`):

```bash
# Create workspace (if not exists)
mkdir -p ~/rmrc_ws/src
cd ~/rmrc_ws/src

# If this repo is not in the workspace, create a symlink or copy:
# Option 1: Symlink (recommended)
ln -s /home/sosrobo/rmrc-jetson-server/src/sensing ~/rmrc_ws/src/rmrc_sensing

# Option 2: Copy (if needed)
# cp -r /home/sosrobo/rmrc-jetson-server/src/sensing ~/rmrc_ws/src/rmrc_sensing

# Build the package (if using ROS2 package structure)
cd ~/rmrc_ws
colcon build --packages-select rmrc_sensing

# Source the workspace
source ~/rmrc_ws/install/setup.bash
```

## Running the Node

### Option 1: Direct Python execution (current setup)

The node can be run directly as a Python script:

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash
source ~/rmrc_ws/install/setup.bash  # If using workspace

# Run the node
cd /home/sosrobo/rmrc-jetson-server
python3 src/sensing/lidar_stream.py
```

### Option 2: ROS2 run command (if installed as ROS2 package)

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash
source ~/rmrc_ws/install/setup.bash

# Run via ros2 run
ros2 run rmrc_sensing lidar_stream.py
```

### Option 3: Via systemd service (after setup)

```bash
# Start service
sudo systemctl start lidar-stream.service

# Check status
sudo systemctl status lidar-stream.service

# View logs
journalctl -u lidar-stream.service -f
```

## Configuration

The node loads configuration from `config/network.yaml`:

```yaml
lidar_port: 5556
bind_interface: "*"
```

Default values:
- `lidar_port`: 5556 (if config file not found)
- `bind_interface`: "*" (bind to all interfaces)

## Verification

### Check if node is running

```bash
# List ROS2 nodes
ros2 node list
# Should show: /lidar_stream

# List topics
ros2 topic list
# Should show: /unilidar/cloud

# Check topic info
ros2 topic info /unilidar/cloud
# Should show: sensor_msgs/msg/PointCloud2

# Monitor topic (optional)
ros2 topic echo /unilidar/cloud --once
```

### Check ZeroMQ socket

```bash
# On another machine or terminal, test ZeroMQ connection
python3 -c "
import zmq
context = zmq.Context()
subscriber = context.socket(zmq.SUB)
subscriber.connect('tcp://<JETSON_IP>:5556')
subscriber.setsockopt(zmq.SUBSCRIBE, b'lidar')
print('Connected to ZeroMQ PUB socket')
msg = subscriber.recv_multipart()
print(f'Received message: {len(msg[1])} bytes')
"
```

## Troubleshooting

### Node won't start

```bash
# Check ROS2 installation
ros2 --version

# Check Python dependencies
python3 -c "import rclpy; import zmq; import msgpack; import yaml; print('All dependencies OK')"

# Check config file
cat config/network.yaml
```

### No point cloud messages

```bash
# Check if LiDAR driver is running
ros2 topic list | grep unilidar

# Check if messages are being published
ros2 topic hz /unilidar/cloud
# Should show ~12 Hz

# Check topic data
ros2 topic echo /unilidar/cloud --once
```

### ZeroMQ connection issues

```bash
# Check if port is in use
sudo netstat -tulpn | grep 5556

# Check firewall
sudo ufw status

# Test bind address
python3 -c "import zmq; ctx = zmq.Context(); sock = ctx.socket(zmq.PUB); sock.bind('tcp://*:5556'); print('Bind OK')"
```

### Permission errors

```bash
# Check dialout group (for USB devices)
groups sosrobo

# Add to dialout group if needed
sudo usermod -a -G dialout sosrobo
# (Logout/login required)
```

## Log Output

The node logs at INFO level:
- Configuration loading status
- ZeroMQ socket binding status
- Point cloud send statistics (every 10th message)
- Error messages (with retry logic)

Example log output:
```
[INFO] [lidar_stream]: Loaded config: lidar_port=5556, bind_interface=*
[INFO] [lidar_stream]: ZeroMQ PUB socket bound to tcp://*:5556
[INFO] [lidar_stream]: LiDAR stream node initialized. Subscribed to /unilidar/cloud
[INFO] [lidar_stream]: Point cloud sent: count=10, size=245678 bytes
```
