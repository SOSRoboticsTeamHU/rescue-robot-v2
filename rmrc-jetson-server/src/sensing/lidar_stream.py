#!/usr/bin/env python3
"""
LiDAR Stream Node (ROS2 to ZeroMQ)
Subscribes to /unilidar/cloud topic (sensor_msgs/PointCloud2) and publishes via ZeroMQ PUB socket.
"""

import sys
import os
import yaml
import zmq
import msgpack
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.serialization import serialize_message


class LidarStreamNode(Node):
    """
    ROS2 node that subscribes to LiDAR point cloud topic and streams it via ZeroMQ PUB socket.
    """
    
    def __init__(self):
        super().__init__('lidar_stream')
        
        # Load network configuration
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
            'config', 'network.yaml'
        )
        
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                self.lidar_port = config.get('lidar_port', 5556)
                self.bind_interface = config.get('bind_interface', '*')
            
            self.get_logger().info(f'Loaded config: lidar_port={self.lidar_port}, bind_interface={self.bind_interface}')
        except Exception as e:
            self.get_logger().warn(f'Failed to load config from {config_path}: {e}. Using defaults.')
            self.lidar_port = 5556
            self.bind_interface = '*'
        
        # Initialize ZeroMQ PUB socket
        self.context = zmq.Context()
        self.publisher = self.context.socket(zmq.PUB)
        bind_address = f'tcp://{self.bind_interface}:{self.lidar_port}'
        
        try:
            self.publisher.bind(bind_address)
            self.get_logger().info(f'ZeroMQ PUB socket bound to {bind_address}')
        except Exception as e:
            self.get_logger().error(f'Failed to bind ZeroMQ socket to {bind_address}: {e}')
            raise
        
        # Subscribe to LiDAR point cloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/unilidar/cloud',
            self.cloud_callback,
            10  # QoS depth
        )
        
        self.get_logger().info('LiDAR stream node initialized. Subscribed to /unilidar/cloud')
        
        # Statistics
        self.message_count = 0
        self.error_count = 0
    
    def cloud_callback(self, msg: PointCloud2):
        """
        Callback function called when a new point cloud message is received.
        Serializes the message, compresses it with msgpack, and publishes via ZeroMQ.
        
        Args:
            msg: sensor_msgs/PointCloud2 message
        """
        try:
            # Serialize ROS2 message to bytes
            serialized_msg = serialize_message(msg)
            
            # Compress with msgpack
            packed_data = msgpack.packb(serialized_msg, use_bin_type=True)
            
            # Send via ZeroMQ PUB socket (topic prefix: 'lidar')
            self.publisher.send_multipart([b'lidar', packed_data], zmq.NOBLOCK)
            
            # Update statistics and log
            self.message_count += 1
            data_size = len(packed_data)
            
            # Log every 10th message to reduce log spam (~12 Hz -> ~1.2 log/sec)
            if self.message_count % 10 == 0:
                self.get_logger().info(
                    f'Point cloud sent: count={self.message_count}, size={data_size} bytes'
                )
        
        except zmq.Again:
            # ZeroMQ send buffer full (non-blocking send failed)
            # This is not critical, just means we're sending faster than receiving
            self.error_count += 1
            if self.error_count % 100 == 0:
                self.get_logger().warn(
                    f'ZeroMQ send buffer full (non-blocking). Error count: {self.error_count}'
                )
        
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'Error processing point cloud message: {e}')
            
            # Don't crash on errors, just log them
            if self.error_count > 1000:
                self.get_logger().error('Too many errors, shutting down node.')
                rclpy.shutdown()
                sys.exit(1)
    
    def destroy_node(self):
        """
        Cleanup on node shutdown.
        """
        self.get_logger().info(f'Shutting down. Total messages sent: {self.message_count}, errors: {self.error_count}')
        
        if self.publisher:
            self.publisher.close()
        if self.context:
            self.context.term()
        
        super().destroy_node()


def main(args=None):
    """
    Main entry point for the LiDAR stream node.
    """
    rclpy.init(args=args)
    
    node = LidarStreamNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutdown requested by user')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
