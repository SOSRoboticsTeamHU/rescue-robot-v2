"""
ZMQ Protocol Definitions for Distributed Robotics Suite
Defines message formats for communication between Mac GUI and Jetson hardware bridge.
"""

import json
import time
from enum import Enum
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Any
from datetime import datetime


class MessageType(Enum):
    """Message types for ZMQ communication"""
    TELEMETRY = "TELEMETRY"
    DECISION = "DECISION"
    STATUS = "STATUS"
    SENSOR_STATE = "SENSOR_STATE"
    EMERGENCY = "EMERGENCY"
    HEARTBEAT = "HEARTBEAT"
    MISSION_START = "MISSION_START"
    MISSION_END = "MISSION_END"
    QR_CODE = "QR_CODE"
    HAZMAT_DETECTION = "HAZMAT_DETECTION"
    LANDOLT_DETECTION = "LANDOLT_DETECTION"
    ARM_POSITION = "ARM_POSITION"
    SLAM_UPDATE = "SLAM_UPDATE"


class SensorStatus(Enum):
    """Sensor status enumeration"""
    OK = "OK"
    DISCONNECTED = "DISCONNECTED"
    ERROR = "ERROR"
    DEGRADED = "DEGRADED"


class OperationMode(Enum):
    """Robot operation mode"""
    AUTONOMOUS = "AUTONOMOUS"
    MANUAL = "MANUAL"
    SAFE_MANUAL = "SAFE_MANUAL"
    EMERGENCY_STOP = "EMERGENCY_STOP"


@dataclass
class TelemetryData:
    """CPU/GPU telemetry from Jetson"""
    timestamp: float
    cpu_load: float  # 0.0 - 1.0
    gpu_load: float  # 0.0 - 1.0
    temperature: float  # Celsius
    memory_usage: float  # 0.0 - 1.0
    power_usage: float  # Watts
    
    def to_dict(self) -> Dict:
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: Dict):
        return cls(**data)


@dataclass
class LiDARHealth:
    """LiDAR sensor health data"""
    timestamp: float
    status: SensorStatus
    point_count: int
    scan_rate: float  # Hz
    error_code: Optional[int] = None
    
    def to_dict(self) -> Dict:
        return {**asdict(self), "status": self.status.value}
    
    @classmethod
    def from_dict(cls, data: Dict):
        data["status"] = SensorStatus(data["status"])
        return cls(**data)


@dataclass
class DecisionLog:
    """AI decision reasoning log entry"""
    timestamp: float
    decision_type: str  # e.g., "EXPLORE", "NAVIGATE", "AVOID"
    reasoning: str
    confidence: float  # 0.0 - 1.0
    context: Optional[Dict[str, Any]] = None
    
    def to_dict(self) -> Dict:
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: Dict):
        return cls(**data)


@dataclass
class SensorState:
    """State of a single sensor"""
    sensor_id: str
    sensor_type: str  # "CAMERA", "LIDAR", "IMU", etc.
    status: SensorStatus
    last_update: float
    metadata: Optional[Dict[str, Any]] = None
    
    def to_dict(self) -> Dict:
        return {**asdict(self), "status": self.status.value}
    
    @classmethod
    def from_dict(cls, data: Dict):
        data["status"] = SensorStatus(data["status"])
        return cls(**data)


@dataclass
class ArmPosition:
    """Robot arm position in 3D space (Ghost Arm overlay)"""
    timestamp: float
    joint_angles: List[float]  # Radians
    end_effector_pos: List[float]  # [x, y, z] in meters
    end_effector_rot: List[float]  # [roll, pitch, yaw] in radians
    is_valid: bool  # True if position doesn't violate soft limits
    
    def to_dict(self) -> Dict:
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: Dict):
        return cls(**data)


@dataclass
class QRCodeDetection:
    """Detected QR code information"""
    timestamp: float
    content: str
    slam_coordinates: List[float]  # [x, y, z] in meters
    confidence: float
    
    def to_dict(self) -> Dict:
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: Dict):
        return cls(**data)


@dataclass
class HazmatDetection:
    """Detected Hazmat label information"""
    timestamp: float
    label_type: str
    confidence: float
    image_crop_path: str  # Path to saved crop
    slam_coordinates: List[float]
    
    def to_dict(self) -> Dict:
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: Dict):
        return cls(**data)


@dataclass
class LandoltCDetection:
    """Detected Landolt-C target information"""
    timestamp: float
    orientation: str  # "UP", "DOWN", "LEFT", "RIGHT"
    confidence: float
    image_crop_path: str
    slam_coordinates: List[float]
    
    def to_dict(self) -> Dict:
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: Dict):
        return cls(**data)


@dataclass
class SLAMMapUpdate:
    """SLAM map coordinate update"""
    timestamp: float
    robot_position: List[float]  # [x, y, z]
    robot_orientation: List[float]  # [roll, pitch, yaw]
    obstacle_map: Optional[List[List[float]]] = None  # List of [x, y, z] obstacle points
    
    def to_dict(self) -> Dict:
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: Dict):
        return cls(**data)


@dataclass
class ZMQMessage:
    """Wrapper for all ZMQ messages"""
    msg_type: MessageType
    timestamp: float
    data: Dict[str, Any]
    sequence_id: Optional[int] = None
    
    def serialize(self) -> bytes:
        """Serialize message to JSON bytes"""
        payload = {
            "msg_type": self.msg_type.value,
            "timestamp": self.timestamp,
            "data": self.data,
            "sequence_id": self.sequence_id
        }
        return json.dumps(payload).encode('utf-8')
    
    @classmethod
    def deserialize(cls, raw: bytes):
        """Deserialize message from JSON bytes"""
        payload = json.loads(raw.decode('utf-8'))
        return cls(
            msg_type=MessageType(payload["msg_type"]),
            timestamp=payload["timestamp"],
            data=payload["data"],
            sequence_id=payload.get("sequence_id")
        )
    
    @classmethod
    def create_telemetry(cls, telemetry: TelemetryData, lidar: LiDARHealth, sequence_id: Optional[int] = None):
        """Create a telemetry message"""
        return cls(
            msg_type=MessageType.TELEMETRY,
            timestamp=time.time(),
            data={
                "telemetry": telemetry.to_dict(),
                "lidar": lidar.to_dict()
            },
            sequence_id=sequence_id
        )
    
    @classmethod
    def create_decision(cls, decision: DecisionLog, sequence_id: Optional[int] = None):
        """Create a decision log message"""
        return cls(
            msg_type=MessageType.DECISION,
            timestamp=time.time(),
            data={"decision": decision.to_dict()},
            sequence_id=sequence_id
        )
    
    @classmethod
    def create_sensor_state(cls, sensors: List[SensorState], sequence_id: Optional[int] = None):
        """Create a sensor state message"""
        return cls(
            msg_type=MessageType.SENSOR_STATE,
            timestamp=time.time(),
            data={"sensors": [s.to_dict() for s in sensors]},
            sequence_id=sequence_id
        )
    
    @classmethod
    def create_heartbeat(cls, sequence_id: Optional[int] = None):
        """Create a heartbeat message"""
        return cls(
            msg_type=MessageType.HEARTBEAT,
            timestamp=time.time(),
            data={},
            sequence_id=sequence_id
        )
    
    @classmethod
    def create_emergency(cls, reason: str, sequence_id: Optional[int] = None):
        """Create an emergency message"""
        return cls(
            msg_type=MessageType.EMERGENCY,
            timestamp=time.time(),
            data={"reason": reason},
            sequence_id=sequence_id
        )
    
    @classmethod
    def create_arm_position(cls, arm_pos: ArmPosition, sequence_id: Optional[int] = None):
        """Create an arm position message"""
        return cls(
            msg_type=MessageType.ARM_POSITION,
            timestamp=time.time(),
            data={"arm_position": arm_pos.to_dict()},
            sequence_id=sequence_id
        )


# ZMQ Connection Configuration
ZMQ_PUB_PORT = 5555  # Jetson publishes to this port
ZMQ_SUB_PORT = 5556  # Jetson subscribes to this port (if bidirectional needed)
HEARTBEAT_INTERVAL = 0.1  # 100ms
HEARTBEAT_TIMEOUT = 0.2  # 200ms timeout for missed heartbeats