"""
Default configuration for distributed robotics suite
"""

# ZMQ Configuration
ZMQ_CONFIG = {
    "pub_port": 5555,
    "sub_port": 5556,
    "heartbeat_interval": 0.1,  # 100ms
    "heartbeat_timeout": 0.2,  # 200ms
    "packet_loss_threshold": 0.1,  # 100ms packet loss triggers safe-stop
}

# Sensor Watchdog Configuration
SENSOR_CONFIG = {
    "camera_timeout": 0.2,  # 200ms
    "lidar_timeout": 0.5,  # 500ms
    "imu_timeout": 0.3,  # 300ms
}

# Soft Limits Configuration (meters)
SOFT_LIMITS_CONFIG = {
    "x_min": -0.5,
    "x_max": 0.5,
    "y_min": -0.5,
    "y_max": 0.5,
    "z_min": 0.1,  # Prevent hitting ground
    "z_max": 1.0,
    "chassis_center": (0.0, 0.0, 0.2),
    "chassis_radius": 0.3,  # meters
}

# GUI Configuration
GUI_CONFIG = {
    "update_rate": 60,  # FPS target
    "decision_feed_max_lines": 1000,
    "telemetry_update_interval": 0.1,  # 100ms
    "team_name": "TEAM LOGO",
    "brand_color": "#00FF64",  # Cyberpunk green
}

# Mission Reporting Configuration
MISSION_CONFIG = {
    "auto_generate_report": True,
    "report_output_dir": "reports/",
    "save_detection_crops": True,
    "crop_output_dir": "detections/",
}

# Camera Configuration
CAMERA_CONFIG = {
    "num_cameras": 4,
    "stream_decode_threads": 4,
    "image_width": 1920,
    "image_height": 1080,
    "fps": 30,
}

# Mock/Simulation Mode
SIMULATION_CONFIG = {
    "enable_mock_data": False,
    "mock_telemetry_rate": 10,  # Hz
    "mock_decision_rate": 1,  # Hz
}