# Rescue Robot v2 - Distributed Robotics Suite

High-performance distributed robotics suite with hardware failure handling and visual feedback for competition judges.

## Architecture

The system is divided into three main components:

1. **Mac GUI (`mac/gui_master.py`)** - PyQt6 + OpenGL interface with dark "Cyberpunk-Military" theme
2. **Jetson Hardware Bridge (`jetson/hardware_bridge.py`)** - Sensor monitoring, motion constraints, ZMQ communication
3. **Shared Protocol (`shared/protocol.py`)** - ZMQ message definitions and data structures

## Features

### GUI & UX (Mac)
- **Telemetry HUD**: Real-time gauges for CPU/GPU load, temperature (jtop), and LiDAR health
- **Decision Feed**: Scrolling log showing AI reasoning
- **Status Indicators**: RGB indicators for sensors (Green=OK, Red=DISCONNECTED)
- **3D Engine**: OpenGL-based point cloud renderer with "Ghost Arm" overlay
- **Branding**: Team logo and mission timer display

### Hardware Failsafe & Fallbacks
- **Sensor Watchdog**: Automatic "Safe-Manual" mode on camera/LiDAR failure
- **Soft Limits**: Software-defined constraints to prevent arm collisions
- **Virtual Tether**: Heartbeat-based connectivity monitoring with auto safe-stop

### Mission Reporting
- **PDF Export**: Professional branded reports with:
  - QR code detections + SLAM coordinates
  - Hazmat/Landolt-C image crops
  - Autonomous vs. Manual time ratio
  - Performance metrics

## Installation

### Mac (GUI)
```bash
pip install -r requirements.txt
```

Note: `jtop` is only required on Jetson hardware, not on Mac.

### Jetson
```bash
pip install -r requirements.txt
# jtop may require additional setup on Jetson
```

## Usage

### Mac GUI (Mock Mode - for testing without Jetson)
```bash
python mac/gui_master.py --mock
```

### Mac GUI (Connected to Jetson)
```bash
python mac/gui_master.py
```

### Jetson Hardware Bridge (Simulation Mode)
```bash
python jetson/hardware_bridge.py --simulation
```

### Jetson Hardware Bridge (Production)
```bash
python jetson/hardware_bridge.py
```

## Code Structure

```
rescue-robot-v2/
├── shared/
│   ├── protocol.py          # ZMQ message definitions
│   └── mission_report.py    # Mission report generator
├── mac/
│   └── gui_master.py        # PyQt6 GUI with OpenGL
├── jetson/
│   └── hardware_bridge.py   # Sensor monitoring & ZMQ publisher
├── config/
│   └── default_config.py    # Configuration settings
├── requirements.txt
└── README.md
```

## Configuration

Edit `config/default_config.py` to adjust:
- ZMQ ports and heartbeat intervals
- Sensor timeout thresholds
- Soft limits for arm motion
- GUI update rates and styling

## Mock/Simulation Mode

Both the GUI and hardware bridge support mock/simulation modes for testing without physical hardware:

- **GUI Mock Mode**: Generates simulated telemetry and decision logs
- **Bridge Simulation Mode**: Runs without requiring actual sensors or jtop

## Mission Reports

Mission reports are automatically generated on mission end and include:
- Performance metrics (autonomous/manual ratio)
- All QR code detections with SLAM coordinates
- Hazmat and Landolt-C detections with image crops
- Timestamps and confidence scores

## Development

The codebase follows a modular design:
- **Communication Layer**: ZMQ for distributed messaging
- **UI Layer**: PyQt6 with OpenGL for 3D visualization
- **Hardware Abstraction**: Watchdog and soft limits for safety

## Notes

- GUI runs at 60 FPS target with async camera stream decoding
- Heartbeat timeout: 200ms triggers emergency safe-stop
- Soft limits prevent arm from hitting chassis or ground
- All sensor failures trigger visual/vocal alarms

## License

[Your License Here]