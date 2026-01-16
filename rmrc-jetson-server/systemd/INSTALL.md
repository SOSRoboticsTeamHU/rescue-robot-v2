# Systemd Service Installation Guide

## 1. Unilidar Driver Service

### Copy file
```bash
sudo cp systemd/unilidar-driver.service /etc/systemd/system/
```

### Enable and start service
```bash
# Reload systemd configuration
sudo systemctl daemon-reload

# Enable service at boot
sudo systemctl enable unilidar-driver.service

# Start service now
sudo systemctl start unilidar-driver.service

# Check status
sudo systemctl status unilidar-driver.service

# View logs
journalctl -u unilidar-driver.service -f
```

## 2. LiDAR Stream Service

### Copy file
```bash
sudo cp systemd/lidar-stream.service /etc/systemd/system/
```

### Enable and start service
```bash
# Reload systemd configuration
sudo systemctl daemon-reload

# Enable service at boot
sudo systemctl enable lidar-stream.service

# Start service now
sudo systemctl start lidar-stream.service

# Check status
sudo systemctl status lidar-stream.service

# View logs
journalctl -u lidar-stream.service -f
```

## 3. Main Server Service

### Copy file
```bash
sudo cp systemd/main-server.service /etc/systemd/system/
```

### Enable and start service
```bash
# Reload systemd configuration
sudo systemctl daemon-reload

# Enable service at boot
sudo systemctl enable main-server.service

# Start service now
sudo systemctl start main-server.service

# Check status
sudo systemctl status main-server.service

# View logs
journalctl -u main-server.service -f
```

## 4. Check all services together

### Status of all services
```bash
sudo systemctl status unilidar-driver.service lidar-stream.service main-server.service
```

### View logs together
```bash
journalctl -u unilidar-driver.service -u lidar-stream.service -u main-server.service -f
```

### Restart services (if needed)
```bash
sudo systemctl restart unilidar-driver.service
sudo systemctl restart lidar-stream.service
sudo systemctl restart main-server.service
```

### Stop services
```bash
sudo systemctl stop unilidar-driver.service
sudo systemctl stop lidar-stream.service
sudo systemctl stop main-server.service
```

### Disable services (do not start at boot)
```bash
sudo systemctl disable unilidar-driver.service
sudo systemctl disable lidar-stream.service
sudo systemctl disable main-server.service
```

## Important notes

1. **User and group**: Services run as user `sosrobo` and group `dialout` (USB permissions).

2. **ROS2 workspace**: Services automatically source:
   - `/opt/ros/humble/setup.bash`
   - `~/rmrc_ws/install/setup.bash`

3. **Dependencies**: 
   - `lidar-stream.service` only starts after `unilidar-driver.service` (`After=` and `Requires=`).
   - `main-server.service` only starts after `lidar-stream.service` (`After=` and `Requires=`).

4. **Automatic restart**: All services have `Restart=always` configured (with 5 second delay).

5. **Working directory**: `lidar-stream.service` and `main-server.service` run from `/home/sosrobo/rmrc-jetson-server` directory.

## Troubleshooting

### Service won't start
```bash
# Detailed logs
journalctl -u unilidar-driver.service -n 50
journalctl -u lidar-stream.service -n 50
journalctl -u main-server.service -n 50

# Manual execution check (as sosrobo user)
su - sosrobo
source /opt/ros/humble/setup.bash
source ~/rmrc_ws/install/setup.bash

# Test LiDAR driver
ros2 launch unitree_lidar_ros2 launch.py

# Test main server (in separate terminal)
cd /home/sosrobo/rmrc-jetson-server
python3 src/main_server.py
```

### Permission errors
```bash
# Check dialout group
groups sosrobo

# If not in group, add:
sudo usermod -a -G dialout sosrobo
# (Logout/login required)
```

### USB device not found
```bash
# Check /dev/ttyACM0
ls -l /dev/ttyACM0

# Check dialout permissions
sudo chmod 666 /dev/ttyACM0  # Temporary solution
```
