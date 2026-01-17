"""
Mac GUI Master - PyQt6 + Modern OpenGL
Dark "Cyberpunk-Military" themed interface for robotics competition
"""

import sys
import time
import threading
from datetime import datetime, timedelta
from typing import Dict, List, Optional
from collections import deque
from pathlib import Path

import PyQt6.QtCore as QtCore
from PyQt6.QtCore import QThread, pyqtSignal, QTimer
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QTextEdit, QPushButton, QFrame
)
from PyQt6.QtGui import QFont, QColor, QPalette, QPixmap, QPainter, QIcon
from PyQt6.QtOpenGLWidgets import QOpenGLWidget
from PyQt6.QtOpenGL import QOpenGLContext

try:
    import zmq
except ImportError:
    zmq = None
    print("WARNING: zmq not installed. Running in mock mode.")

# Import shared protocol
sys.path.insert(0, str(Path(__file__).parent.parent))
from shared.protocol import (
    ZMQMessage, MessageType, TelemetryData, LiDARHealth,
    DecisionLog, SensorState, SensorStatus, ArmPosition,
    OperationMode, ZMQ_PUB_PORT, HEARTBEAT_TIMEOUT
)


# Cyberpunk-Military Color Scheme
COLORS = {
    "bg_dark": QColor(10, 12, 18),
    "bg_medium": QColor(20, 24, 32),
    "bg_light": QColor(30, 36, 48),
    "accent_green": QColor(0, 255, 100),
    "accent_red": QColor(255, 50, 50),
    "accent_blue": QColor(0, 150, 255),
    "accent_yellow": QColor(255, 200, 0),
    "text_primary": QColor(220, 220, 220),
    "text_secondary": QColor(150, 150, 150),
    "border": QColor(60, 70, 90),
    "glow": QColor(0, 255, 100, 100)
}


class OpenGLViewer(QOpenGLWidget):
    """3D Point Cloud Renderer with Ghost Arm Overlay"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(800, 600)
        self.point_cloud = []  # List of [x, y, z] points
        self.arm_positions = []  # List of ArmPosition objects
        self.camera_pos = [0, 0, 5]
        
    def initializeGL(self):
        """Initialize OpenGL context"""
        from OpenGL import GL
        GL.glClearColor(0.05, 0.05, 0.08, 1.0)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glPointSize(2.0)
        
    def paintGL(self):
        """Render point cloud and ghost arm"""
        from OpenGL import GL
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        
        # TODO: Implement actual OpenGL rendering
        # For now, this is a placeholder that will be enhanced with:
        # - Point cloud rendering from LiDAR data
        # - Ghost arm visualization from ArmPosition data
        # - Coordinate axes and grid
        
    def resizeGL(self, width, height):
        """Handle window resize"""
        from OpenGL import GL
        GL.glViewport(0, 0, width, height)
        
    def update_point_cloud(self, points: List[List[float]]):
        """Update point cloud data"""
        self.point_cloud = points
        self.update()
        
    def update_arm_position(self, arm_pos: Optional[ArmPosition]):
        """Update ghost arm position"""
        if arm_pos:
            self.arm_positions.append(arm_pos)
            # Keep only last 100 positions for trail effect
            if len(self.arm_positions) > 100:
                self.arm_positions = self.arm_positions[-100:]
        self.update()


class TelemetryGauge(QWidget):
    """Real-time gauge widget for CPU/GPU/Temperature"""
    
    def __init__(self, label: str, unit: str = "", parent=None):
        super().__init__(parent)
        self.label = label
        self.unit = unit
        self.value = 0.0
        self.max_value = 1.0
        self.setMinimumSize(150, 120)
        
    def paintEvent(self, event):
        """Draw gauge"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        # Background
        painter.fillRect(self.rect(), COLORS["bg_medium"])
        painter.setPen(QColor(COLORS["border"]))
        painter.drawRect(self.rect().adjusted(0, 0, -1, -1))
        
        # Gauge arc (simplified bar gauge)
        bar_height = 60
        bar_width = self.width() - 20
        bar_rect = QtCore.QRect(10, 40, bar_width, bar_height)
        
        # Background bar
        painter.fillRect(bar_rect, COLORS["bg_dark"])
        
        # Value bar
        fill_width = int(bar_width * (self.value / self.max_value))
        fill_color = COLORS["accent_green"]
        if self.value > 0.8:
            fill_color = COLORS["accent_red"]
        elif self.value > 0.6:
            fill_color = COLORS["accent_yellow"]
            
        fill_rect = QtCore.QRect(10, 40, fill_width, bar_height)
        painter.fillRect(fill_rect, fill_color)
        
        # Text
        painter.setPen(COLORS["text_primary"])
        font = QFont("Courier New", 10, QFont.Weight.Bold)
        painter.setFont(font)
        painter.drawText(10, 20, self.label)
        
        value_str = f"{self.value:.1f}{self.unit}"
        painter.drawText(10, self.height() - 10, value_str)
        
    def set_value(self, value: float):
        """Update gauge value"""
        self.value = value
        self.update()


class StatusIndicator(QWidget):
    """RGB status indicator widget"""
    
    def __init__(self, label: str, parent=None):
        super().__init__(parent)
        self.label = label
        self.status = SensorStatus.DISCONNECTED
        self.setMinimumSize(120, 60)
        
    def paintEvent(self, event):
        """Draw status indicator"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        # Background
        painter.fillRect(self.rect(), COLORS["bg_medium"])
        
        # Status LED
        led_size = 30
        led_x = (self.width() - led_size) // 2
        led_y = 10
        
        color = COLORS["accent_red"]  # Default: DISCONNECTED
        if self.status == SensorStatus.OK:
            color = COLORS["accent_green"]
        elif self.status == SensorStatus.DEGRADED:
            color = COLORS["accent_yellow"]
            
        painter.setBrush(color)
        painter.setPen(QColor(color.red(), color.green(), color.blue(), 200))
        painter.drawEllipse(led_x, led_y, led_size, led_size)
        
        # Glow effect for OK status
        if self.status == SensorStatus.OK:
            glow_pen = QColor(color.red(), color.green(), color.blue(), 50)
            painter.setPen(glow_pen)
            painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
            painter.drawEllipse(led_x - 5, led_y - 5, led_size + 10, led_size + 10)
        
        # Label
        painter.setPen(COLORS["text_secondary"])
        font = QFont("Courier New", 8)
        painter.setFont(font)
        text_rect = QtCore.QRect(0, led_y + led_size + 5, self.width(), 20)
        painter.drawText(text_rect, QtCore.Qt.AlignmentFlag.AlignCenter, self.label)
        
    def set_status(self, status: SensorStatus):
        """Update status indicator"""
        self.status = status
        self.update()


class DecisionFeed(QTextEdit):
    """Scrolling decision log widget"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMaximumBlockCount(1000)  # Keep last 1000 entries
        self.setStyleSheet(f"""
            QTextEdit {{
                background-color: {COLORS["bg_dark"].name()};
                color: {COLORS["text_primary"].name()};
                border: 1px solid {COLORS["border"].name()};
                font-family: 'Courier New', monospace;
                font-size: 10pt;
            }}
        """)
        
    def add_decision(self, decision: DecisionLog):
        """Add decision log entry"""
        timestamp = datetime.fromtimestamp(decision.timestamp).strftime("%H:%M:%S.%f")[:-3]
        decision_type = decision.decision_type
        reasoning = decision.reasoning
        
        color_map = {
            "EXPLORE": COLORS["accent_blue"],
            "NAVIGATE": COLORS["accent_green"],
            "AVOID": COLORS["accent_yellow"],
            "EMERGENCY": COLORS["accent_red"]
        }
        color = color_map.get(decision_type, COLORS["text_primary"])
        
        self.setTextColor(color)
        self.append(f"[{timestamp}] [{decision_type}]: {reasoning}")
        
        # Auto-scroll to bottom
        scrollbar = self.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())


class ZMQReceiverThread(QThread):
    """Thread for receiving ZMQ messages"""
    
    telemetry_received = pyqtSignal(dict, dict)  # telemetry, lidar
    decision_received = pyqtSignal(object)  # DecisionLog
    sensor_state_received = pyqtSignal(list)  # List[SensorState]
    arm_position_received = pyqtSignal(object)  # ArmPosition
    emergency_received = pyqtSignal(str)  # reason
    heartbeat_received = pyqtSignal(float)  # timestamp
    
    def __init__(self, port: int = ZMQ_PUB_PORT, mock_mode: bool = False):
        super().__init__()
        self.port = port
        self.mock_mode = mock_mode
        self.running = False
        self.context = None
        self.socket = None
        
    def run(self):
        """Run ZMQ receiver loop"""
        if self.mock_mode or zmq is None:
            self.run_mock_mode()
            return
            
        try:
            self.context = zmq.Context()
            self.socket = self.context.socket(zmq.SUB)
            self.socket.connect(f"tcp://localhost:{self.port}")
            self.socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all
            
            self.running = True
            last_heartbeat = time.time()
            
            while self.running:
                try:
                    # Non-blocking receive with timeout
                    if self.socket.poll(timeout=100):  # 100ms timeout
                        raw_msg = self.socket.recv(zmq.NOBLOCK)
                        msg = ZMQMessage.deserialize(raw_msg)
                        self.process_message(msg)
                        if msg.msg_type == MessageType.HEARTBEAT:
                            last_heartbeat = time.time()
                    else:
                        # Check for heartbeat timeout
                        if time.time() - last_heartbeat > HEARTBEAT_TIMEOUT:
                            self.emergency_received.emit("Heartbeat timeout - connection lost")
                            last_heartbeat = time.time()  # Reset to prevent spam
                            
                except zmq.Again:
                    pass
                except Exception as e:
                    print(f"Error processing message: {e}")
                    
        except Exception as e:
            print(f"ZMQ receiver error: {e}")
            self.emergency_received.emit(f"ZMQ connection error: {e}")
        finally:
            if self.socket:
                self.socket.close()
            if self.context:
                self.context.term()
                
    def process_message(self, msg: ZMQMessage):
        """Process received ZMQ message"""
        if msg.msg_type == MessageType.TELEMETRY:
            data = msg.data
            telemetry = TelemetryData.from_dict(data["telemetry"])
            lidar = LiDARHealth.from_dict(data["lidar"])
            self.telemetry_received.emit(telemetry.to_dict(), lidar.to_dict())
            
        elif msg.msg_type == MessageType.DECISION:
            decision = DecisionLog.from_dict(msg.data["decision"])
            self.decision_received.emit(decision)
            
        elif msg.msg_type == MessageType.SENSOR_STATE:
            sensors = [SensorState.from_dict(s) for s in msg.data["sensors"]]
            self.sensor_state_received.emit(sensors)
            
        elif msg.msg_type == MessageType.ARM_POSITION:
            arm_pos = ArmPosition.from_dict(msg.data["arm_position"])
            self.arm_position_received.emit(arm_pos)
            
        elif msg.msg_type == MessageType.EMERGENCY:
            reason = msg.data.get("reason", "Unknown emergency")
            self.emergency_received.emit(reason)
            
        elif msg.msg_type == MessageType.HEARTBEAT:
            self.heartbeat_received.emit(msg.timestamp)
            
    def run_mock_mode(self):
        """Run in mock mode for testing without Jetson"""
        import random
        self.running = True
        
        while self.running:
            time.sleep(0.1)  # 10 Hz update rate
            
            # Generate mock telemetry
            telemetry_data = {
                "timestamp": time.time(),
                "cpu_load": random.uniform(0.3, 0.8),
                "gpu_load": random.uniform(0.2, 0.7),
                "temperature": random.uniform(45, 75),
                "memory_usage": random.uniform(0.4, 0.8),
                "power_usage": random.uniform(10, 25)
            }
            lidar_data = {
                "timestamp": time.time(),
                "status": "OK",
                "point_count": random.randint(5000, 30000),
                "scan_rate": random.uniform(10, 20)
            }
            self.telemetry_received.emit(telemetry_data, lidar_data)
            
            # Generate occasional mock decisions
            if random.random() < 0.1:  # 10% chance
                decision_types = ["EXPLORE", "NAVIGATE", "AVOID"]
                reasons = [
                    "Path blocked by 'Gravel' obstacle -> recalculating...",
                    "Waypoint reached -> proceeding to next target",
                    "Object detected -> avoiding collision"
                ]
                decision = DecisionLog(
                    timestamp=time.time(),
                    decision_type=random.choice(decision_types),
                    reasoning=random.choice(reasons),
                    confidence=random.uniform(0.7, 0.95)
                )
                self.decision_received.emit(decision)
                
    def stop(self):
        """Stop the receiver thread"""
        self.running = False


class MainWindow(QMainWindow):
    """Main GUI Window"""
    
    def __init__(self, mock_mode: bool = False):
        super().__init__()
        self.mock_mode = mock_mode
        self.mission_start_time = None
        self.mission_timer = QTimer()
        self.mission_timer.timeout.connect(self.update_mission_timer)
        
        # ZMQ receiver thread
        self.zmq_thread = ZMQReceiverThread(mock_mode=mock_mode)
        self.zmq_thread.telemetry_received.connect(self.update_telemetry)
        self.zmq_thread.decision_received.connect(self.add_decision)
        self.zmq_thread.sensor_state_received.connect(self.update_sensor_states)
        self.zmq_thread.arm_position_received.connect(self.update_arm_position)
        self.zmq_thread.emergency_received.connect(self.handle_emergency)
        
        self.init_ui()
        
    def init_ui(self):
        """Initialize UI components"""
        self.setWindowTitle("Rescue Robot Control Interface")
        self.setGeometry(100, 100, 1920, 1080)
        
        # Apply dark theme
        palette = QPalette()
        palette.setColor(QPalette.ColorRole.Window, COLORS["bg_dark"])
        palette.setColor(QPalette.ColorRole.WindowText, COLORS["text_primary"])
        self.setPalette(palette)
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # Left panel: Telemetry and Status
        left_panel = self.create_left_panel()
        main_layout.addWidget(left_panel, 1)
        
        # Center panel: 3D Viewer
        center_panel = self.create_center_panel()
        main_layout.addWidget(center_panel, 2)
        
        # Right panel: Decision Feed and Branding
        right_panel = self.create_right_panel()
        main_layout.addWidget(right_panel, 1)
        
        # Start ZMQ receiver
        self.zmq_thread.start()
        
    def create_left_panel(self) -> QWidget:
        """Create left panel with telemetry gauges and status indicators"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Title
        title = QLabel("TELEMETRY HUD")
        title.setStyleSheet(f"color: {COLORS['accent_green'].name()}; font-size: 16pt; font-weight: bold;")
        layout.addWidget(title)
        
        # Telemetry gauges
        self.cpu_gauge = TelemetryGauge("CPU Load", "%")
        self.gpu_gauge = TelemetryGauge("GPU Load", "%")
        self.temp_gauge = TelemetryGauge("Temperature", "°C")
        self.mem_gauge = TelemetryGauge("Memory", "%")
        
        gauges_layout = QHBoxLayout()
        gauges_layout.addWidget(self.cpu_gauge)
        gauges_layout.addWidget(self.gpu_gauge)
        layout.addLayout(gauges_layout)
        
        gauges_layout2 = QHBoxLayout()
        gauges_layout2.addWidget(self.temp_gauge)
        gauges_layout2.addWidget(self.mem_gauge)
        layout.addLayout(gauges_layout2)
        
        # Status indicators
        status_title = QLabel("SENSOR STATUS")
        status_title.setStyleSheet(f"color: {COLORS['accent_blue'].name()}; font-size: 12pt; margin-top: 20px;")
        layout.addWidget(status_title)
        
        status_layout = QHBoxLayout()
        self.camera_indicator = StatusIndicator("Camera")
        self.lidar_indicator = StatusIndicator("LiDAR")
        self.imu_indicator = StatusIndicator("IMU")
        status_layout.addWidget(self.camera_indicator)
        status_layout.addWidget(self.lidar_indicator)
        status_layout.addWidget(self.imu_indicator)
        layout.addLayout(status_layout)
        
        layout.addStretch()
        return panel
        
    def create_center_panel(self) -> QWidget:
        """Create center panel with 3D viewer"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        title = QLabel("3D POINT CLOUD & GHOST ARM")
        title.setStyleSheet(f"color: {COLORS['accent_blue'].name()}; font-size: 16pt; font-weight: bold;")
        layout.addWidget(title)
        
        self.viewer = OpenGLViewer()
        layout.addWidget(self.viewer)
        
        return panel
        
    def create_right_panel(self) -> QWidget:
        """Create right panel with decision feed and branding"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Decision Feed
        title = QLabel("DECISION FEED")
        title.setStyleSheet(f"color: {COLORS['accent_yellow'].name()}; font-size: 16pt; font-weight: bold;")
        layout.addWidget(title)
        
        self.decision_feed = DecisionFeed()
        layout.addWidget(self.decision_feed, 1)
        
        # Branding section
        branding_frame = QFrame()
        branding_frame.setStyleSheet(f"""
            QFrame {{
                background-color: {COLORS["bg_medium"].name()};
                border: 2px solid {COLORS["border"].name()};
                padding: 10px;
            }}
        """)
        branding_layout = QVBoxLayout(branding_frame)
        
        logo_label = QLabel("TEAM LOGO")
        logo_label.setStyleSheet(f"color: {COLORS['text_primary'].name()}; font-size: 14pt; font-weight: bold;")
        logo_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        branding_layout.addWidget(logo_label)
        
        self.mission_timer_label = QLabel("Mission Time: 00:00:00")
        self.mission_timer_label.setStyleSheet(f"color: {COLORS['accent_green'].name()}; font-size: 12pt; font-family: monospace;")
        self.mission_timer_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        branding_layout.addWidget(self.mission_timer_label)
        
        layout.addWidget(branding_frame)
        
        return panel
        
    def update_telemetry(self, telemetry_data: Dict, lidar_data: Dict):
        """Update telemetry gauges"""
        self.cpu_gauge.set_value(telemetry_data["cpu_load"])
        self.gpu_gauge.set_value(telemetry_data["gpu_load"])
        self.temp_gauge.set_value(telemetry_data["temperature"])
        self.temp_gauge.max_value = 100.0  # Temperature in Celsius
        self.mem_gauge.set_value(telemetry_data["memory_usage"])
        
        # Update LiDAR status
        lidar_status = SensorStatus(lidar_data["status"])
        self.lidar_indicator.set_status(lidar_status)
        
    def add_decision(self, decision: DecisionLog):
        """Add decision to feed"""
        self.decision_feed.add_decision(decision)
        
    def update_sensor_states(self, sensors: List[SensorState]):
        """Update sensor status indicators"""
        sensor_map = {
            "CAMERA": self.camera_indicator,
            "LIDAR": self.lidar_indicator,
            "IMU": self.imu_indicator
        }
        
        for sensor in sensors:
            if sensor.sensor_type in sensor_map:
                sensor_map[sensor.sensor_type].set_status(sensor.status)
                
    def update_arm_position(self, arm_pos: ArmPosition):
        """Update ghost arm position in 3D viewer"""
        self.viewer.update_arm_position(arm_pos)
        
    def handle_emergency(self, reason: str):
        """Handle emergency signal"""
        self.decision_feed.add_decision(DecisionLog(
            timestamp=time.time(),
            decision_type="EMERGENCY",
            reasoning=reason,
            confidence=1.0
        ))
        # TODO: Trigger visual/vocal alarm
        
    def update_mission_timer(self):
        """Update mission timer display"""
        if self.mission_start_time:
            elapsed = time.time() - self.mission_start_time
            hours = int(elapsed // 3600)
            minutes = int((elapsed % 3600) // 60)
            seconds = int(elapsed % 60)
            self.mission_timer_label.setText(f"Mission Time: {hours:02d}:{minutes:02d}:{seconds:02d}")
            
    def start_mission(self):
        """Start mission timer"""
        self.mission_start_time = time.time()
        self.mission_timer.start(1000)  # Update every second
        
    def stop_mission(self):
        """Stop mission timer"""
        self.mission_timer.stop()
        
    def closeEvent(self, event):
        """Handle window close"""
        self.zmq_thread.stop()
        self.zmq_thread.wait()
        event.accept()


def main():
    """Main entry point"""
    import argparse
    parser = argparse.ArgumentParser(description="Rescue Robot GUI Master")
    parser.add_argument("--mock", action="store_true", help="Run in mock mode (no Jetson connection)")
    args = parser.parse_args()
    
    app = QApplication(sys.argv)
    window = MainWindow(mock_mode=args.mock)
    window.show()
    
    if args.mock:
        window.start_mission()  # Auto-start for testing
        
    sys.exit(app.exec())


if __name__ == "__main__":
    main()