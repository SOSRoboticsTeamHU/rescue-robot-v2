"""
Jetson Hardware Bridge
Handles sensor monitoring, motion constraints, and ZMQ communication
"""

import sys
import time
import threading
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from collections import deque
from dataclasses import dataclass

try:
    import zmq
except ImportError:
    zmq = None
    print("WARNING: zmq not installed. Running in simulation mode.")

try:
    import jtop
except ImportError:
    jtop = None
    print("WARNING: jtop not installed. CPU/GPU telemetry will be simulated.")

# Import shared protocol
sys.path.insert(0, str(Path(__file__).parent.parent))
from shared.protocol import (
    ZMQMessage, MessageType, TelemetryData, LiDARHealth,
    DecisionLog, SensorState, SensorStatus, ArmPosition,
    OperationMode, ZMQ_PUB_PORT, HEARTBEAT_INTERVAL,
    QRCodeDetection, HazmatDetection, LandoltCDetection, SLAMMapUpdate
)


@dataclass
class SoftLimits:
    """Software-defined motion constraints for robot arm"""
    x_min: float = -0.5  # meters
    x_max: float = 0.5
    y_min: float = -0.5
    y_max: float = 0.5
    z_min: float = 0.1  # Prevent hitting ground
    z_max: float = 1.0
    # Additional chassis avoidance constraints
    chassis_center: Tuple[float, float, float] = (0.0, 0.0, 0.2)
    chassis_radius: float = 0.3  # meters
    
    def is_valid_position(self, x: float, y: float, z: float) -> bool:
        """Check if position is within soft limits"""
        # Check basic bounds
        if not (self.x_min <= x <= self.x_max):
            return False
        if not (self.y_min <= y <= self.y_max):
            return False
        if not (self.z_min <= z <= self.z_max):
            return False
            
        # Check chassis avoidance (simple sphere check)
        cx, cy, cz = self.chassis_center
        dist_sq = (x - cx)**2 + (y - cy)**2 + (z - cz)**2
        if dist_sq < self.chassis_radius**2:
            return False
            
        return True


class SensorWatchdog:
    """Monitors sensor health and triggers safe modes on failure"""
    
    def __init__(self, camera_timeout: float = 0.2, lidar_timeout: float = 0.5):
        self.camera_timeout = camera_timeout
        self.lidar_timeout = lidar_timeout
        self.last_updates: Dict[str, float] = {}
        self.sensor_states: Dict[str, SensorStatus] = {}
        self.callbacks: List[callable] = []
        
    def update_sensor(self, sensor_id: str, sensor_type: str):
        """Update sensor heartbeat"""
        self.last_updates[sensor_id] = time.time()
        if sensor_id not in self.sensor_states:
            self.sensor_states[sensor_id] = SensorStatus.OK
            
    def check_sensors(self) -> List[Tuple[str, SensorStatus, str]]:
        """Check all sensors and return failures"""
        failures = []
        current_time = time.time()
        
        # Check cameras
        camera_ids = [sid for sid in self.last_updates.keys() if sid.startswith("camera")]
        for cam_id in camera_ids:
            if current_time - self.last_updates.get(cam_id, 0) > self.camera_timeout:
                old_status = self.sensor_states.get(cam_id, SensorStatus.OK)
                self.sensor_states[cam_id] = SensorStatus.DISCONNECTED
                if old_status != SensorStatus.DISCONNECTED:
                    failures.append((cam_id, SensorStatus.DISCONNECTED, "Camera timeout"))
                    
        # Check LiDAR
        lidar_ids = [sid for sid in self.last_updates.keys() if sid.startswith("lidar")]
        for lidar_id in lidar_ids:
            if current_time - self.last_updates.get(lidar_id, 0) > self.lidar_timeout:
                old_status = self.sensor_states.get(lidar_id, SensorStatus.OK)
                self.sensor_states[lidar_id] = SensorStatus.DISCONNECTED
                if old_status != SensorStatus.DISCONNECTED:
                    failures.append((lidar_id, SensorStatus.DISCONNECTED, "LiDAR timeout"))
                    
        return failures
        
    def get_sensor_states(self) -> List[SensorState]:
        """Get current sensor states"""
        states = []
        for sensor_id, last_update in self.last_updates.items():
            sensor_type = "CAMERA" if sensor_id.startswith("camera") else "LIDAR" if sensor_id.startswith("lidar") else "UNKNOWN"
            status = self.sensor_states.get(sensor_id, SensorStatus.OK)
            states.append(SensorState(
                sensor_id=sensor_id,
                sensor_type=sensor_type,
                status=status,
                last_update=last_update
            ))
        return states


class HardwareBridge:
    """Main hardware bridge for Jetson"""
    
    def __init__(self, simulation_mode: bool = False):
        self.simulation_mode = simulation_mode
        self.running = False
        
        # ZMQ setup
        self.zmq_context = None
        self.zmq_socket = None
        self.sequence_id = 0
        
        # Components
        self.soft_limits = SoftLimits()
        self.watchdog = SensorWatchdog()
        self.current_mode = OperationMode.MANUAL
        self.mission_start_time: Optional[float] = None
        self.autonomous_time = 0.0
        self.manual_time = 0.0
        self.last_mode_change = time.time()
        
        # Data storage for mission reporting
        self.qr_codes: List[QRCodeDetection] = []
        self.hazmat_detections: List[HazmatDetection] = []
        self.landolt_detections: List[LandoltCDetection] = []
        self.slam_updates: List[SLAMMapUpdate] = []
        
        # Threads
        self.telemetry_thread: Optional[threading.Thread] = None
        self.watchdog_thread: Optional[threading.Thread] = None
        self.heartbeat_thread: Optional[threading.Thread] = None
        
    def initialize(self):
        """Initialize hardware connections"""
        if self.simulation_mode:
            print("Running in SIMULATION MODE")
            return
            
        # Initialize ZMQ
        if zmq is None:
            print("ERROR: zmq not available. Cannot initialize.")
            return
            
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.PUB)
        self.zmq_socket.bind(f"tcp://*:{ZMQ_PUB_PORT}")
        print(f"ZMQ publisher bound to port {ZMQ_PUB_PORT}")
        
        # Initialize jtop for telemetry
        if jtop is None:
            print("WARNING: jtop not available. Using simulated telemetry.")
            
    def start(self):
        """Start hardware bridge threads"""
        self.running = True
        
        # Start telemetry thread
        self.telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self.telemetry_thread.start()
        
        # Start watchdog thread
        self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self.watchdog_thread.start()
        
        # Start heartbeat thread
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.heartbeat_thread.start()
        
        print("Hardware bridge started")
        
    def stop(self):
        """Stop hardware bridge"""
        self.running = False
        
        if self.telemetry_thread:
            self.telemetry_thread.join(timeout=2.0)
        if self.watchdog_thread:
            self.watchdog_thread.join(timeout=2.0)
        if self.heartbeat_thread:
            self.heartbeat_thread.join(timeout=2.0)
            
        if self.zmq_socket:
            self.zmq_socket.close()
        if self.zmq_context:
            self.zmq_context.term()
            
        print("Hardware bridge stopped")
        
    def _telemetry_loop(self):
        """Telemetry publishing loop"""
        jetson = None
        
        if not self.simulation_mode and jtop is not None:
            try:
                jetson = jtop.jtop()
                jetson.start()
            except Exception as e:
                print(f"Failed to initialize jtop: {e}")
                
        while self.running:
            try:
                # Get telemetry
                if jetson and jetson.ok():
                    cpu_load = jetson.cpu['total'] / 100.0
                    gpu_load = jetson.gpu['val'] / 100.0 if jetson.gpu else 0.0
                    temperature = jetson.temperature['CPU']
                    memory_usage = (jetson.memory['used'] / jetson.memory['tot']) if jetson.memory else 0.0
                    power_usage = jetson.power['tot']['cur'] / 1000.0 if jetson.power else 0.0
                else:
                    # Simulated telemetry
                    import random
                    cpu_load = random.uniform(0.3, 0.8)
                    gpu_load = random.uniform(0.2, 0.7)
                    temperature = random.uniform(45, 75)
                    memory_usage = random.uniform(0.4, 0.8)
                    power_usage = random.uniform(10, 25)
                    
                telemetry = TelemetryData(
                    timestamp=time.time(),
                    cpu_load=cpu_load,
                    gpu_load=gpu_load,
                    temperature=temperature,
                    memory_usage=memory_usage,
                    power_usage=power_usage
                )
                
                # Get LiDAR health (simulated for now)
                lidar_health = self._get_lidar_health()
                
                # Publish telemetry
                msg = ZMQMessage.create_telemetry(telemetry, lidar_health, self.sequence_id)
                self._publish_message(msg)
                
                # Update mode time tracking
                self._update_mode_time()
                
            except Exception as e:
                print(f"Telemetry loop error: {e}")
                
            time.sleep(0.1)  # 10 Hz
            
        if jetson:
            jetson.close()
            
    def _get_lidar_health(self) -> LiDARHealth:
        """Get LiDAR health status"""
        # TODO: Integrate with actual LiDAR hardware
        # For now, simulate based on watchdog status
        lidar_states = [s for s in self.watchdog.get_sensor_states() if s.sensor_type == "LIDAR"]
        if lidar_states:
            status = lidar_states[0].status
        else:
            status = SensorStatus.OK  # Assume OK if no watchdog data
            
        import random
        return LiDARHealth(
            timestamp=time.time(),
            status=status,
            point_count=random.randint(5000, 30000) if status == SensorStatus.OK else 0,
            scan_rate=random.uniform(10, 20) if status == SensorStatus.OK else 0.0
        )
        
    def _watchdog_loop(self):
        """Sensor watchdog monitoring loop"""
        while self.running:
            try:
                failures = self.watchdog.check_sensors()
                
                for sensor_id, status, reason in failures:
                    print(f"SENSOR FAILURE: {sensor_id} - {reason}")
                    
                    # Trigger safe-manual mode on critical sensor failure
                    if status == SensorStatus.DISCONNECTED:
                        self.set_mode(OperationMode.SAFE_MANUAL)
                        
                        # Publish emergency message
                        emergency_msg = ZMQMessage.create_emergency(
                            f"Sensor failure: {sensor_id} - {reason}",
                            self.sequence_id
                        )
                        self._publish_message(emergency_msg)
                        
                # Publish sensor states
                sensor_states = self.watchdog.get_sensor_states()
                if sensor_states:
                    msg = ZMQMessage.create_sensor_state(sensor_states, self.sequence_id)
                    self._publish_message(msg)
                    
            except Exception as e:
                print(f"Watchdog loop error: {e}")
                
            time.sleep(0.1)  # 10 Hz
            
    def _heartbeat_loop(self):
        """Heartbeat publishing loop"""
        while self.running:
            try:
                msg = ZMQMessage.create_heartbeat(self.sequence_id)
                self._publish_message(msg)
                self.sequence_id += 1
            except Exception as e:
                print(f"Heartbeat loop error: {e}")
                
            time.sleep(HEARTBEAT_INTERVAL)
            
    def _update_mode_time(self):
        """Update autonomous/manual time tracking"""
        current_time = time.time()
        elapsed = current_time - self.last_mode_change
        
        if self.current_mode == OperationMode.AUTONOMOUS:
            self.autonomous_time += elapsed
        elif self.current_mode in [OperationMode.MANUAL, OperationMode.SAFE_MANUAL]:
            self.manual_time += elapsed
            
        self.last_mode_change = current_time
        
    def _publish_message(self, msg: ZMQMessage):
        """Publish ZMQ message"""
        if self.zmq_socket:
            try:
                serialized = msg.serialize()
                self.zmq_socket.send(serialized)
            except Exception as e:
                print(f"Failed to publish message: {e}")
        elif self.simulation_mode:
            print(f"[SIM] Would publish: {msg.msg_type.value}")
            
    def set_mode(self, mode: OperationMode):
        """Set robot operation mode"""
        if self.current_mode != mode:
            self._update_mode_time()  # Finalize previous mode time
            self.current_mode = mode
            self.last_mode_change = time.time()
            print(f"Mode changed to: {mode.value}")
            
    def check_arm_position(self, x: float, y: float, z: float) -> Tuple[bool, Optional[str]]:
        """Check if arm position is valid (soft limits)"""
        if not self.soft_limits.is_valid_position(x, y, z):
            reason = f"Position ({x:.3f}, {y:.3f}, {z:.3f}) violates soft limits"
            return False, reason
        return True, None
        
    def publish_arm_position(self, joint_angles: List[float], end_effector_pos: List[float],
                             end_effector_rot: List[float]):
        """Publish current arm position"""
        x, y, z = end_effector_pos
        is_valid, reason = self.check_arm_position(x, y, z)
        
        arm_pos = ArmPosition(
            timestamp=time.time(),
            joint_angles=joint_angles,
            end_effector_pos=end_effector_pos,
            end_effector_rot=end_effector_rot,
            is_valid=is_valid
        )
        
        msg = ZMQMessage.create_arm_position(arm_pos, self.sequence_id)
        self._publish_message(msg)
        
        if not is_valid:
            print(f"WARNING: {reason}")
            
    def publish_decision(self, decision_type: str, reasoning: str, confidence: float = 1.0):
        """Publish AI decision log"""
        decision = DecisionLog(
            timestamp=time.time(),
            decision_type=decision_type,
            reasoning=reasoning,
            confidence=confidence
        )
        msg = ZMQMessage.create_decision(decision, self.sequence_id)
        self._publish_message(msg)
        
    def record_qr_code(self, content: str, slam_coords: List[float], confidence: float = 1.0):
        """Record QR code detection for mission reporting"""
        qr = QRCodeDetection(
            timestamp=time.time(),
            content=content,
            slam_coordinates=slam_coords,
            confidence=confidence
        )
        self.qr_codes.append(qr)
        
    def record_hazmat(self, label_type: str, confidence: float, image_crop_path: str,
                      slam_coords: List[float]):
        """Record Hazmat detection for mission reporting"""
        hazmat = HazmatDetection(
            timestamp=time.time(),
            label_type=label_type,
            confidence=confidence,
            image_crop_path=image_crop_path,
            slam_coordinates=slam_coords
        )
        self.hazmat_detections.append(hazmat)
        
    def record_landolt_c(self, orientation: str, confidence: float, image_crop_path: str,
                        slam_coords: List[float]):
        """Record Landolt-C detection for mission reporting"""
        landolt = LandoltCDetection(
            timestamp=time.time(),
            orientation=orientation,
            confidence=confidence,
            image_crop_path=image_crop_path,
            slam_coordinates=slam_coords
        )
        self.landolt_detections.append(landolt)
        
    def get_mission_stats(self) -> Dict:
        """Get mission statistics for reporting"""
        self._update_mode_time()  # Finalize current mode time
        
        total_time = self.autonomous_time + self.manual_time
        autonomous_ratio = self.autonomous_time / total_time if total_time > 0 else 0.0
        
        return {
            "mission_start_time": self.mission_start_time,
            "total_time": total_time,
            "autonomous_time": self.autonomous_time,
            "manual_time": self.manual_time,
            "autonomous_ratio": autonomous_ratio,
            "qr_codes_count": len(self.qr_codes),
            "hazmat_count": len(self.hazmat_detections),
            "landolt_count": len(self.landolt_detections)
        }


def main():
    """Main entry point for hardware bridge"""
    import argparse
    parser = argparse.ArgumentParser(description="Jetson Hardware Bridge")
    parser.add_argument("--simulation", action="store_true", help="Run in simulation mode")
    args = parser.parse_args()
    
    bridge = HardwareBridge(simulation_mode=args.simulation)
    bridge.initialize()
    
    try:
        bridge.start()
        bridge.mission_start_time = time.time()
        
        print("Hardware bridge running. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()