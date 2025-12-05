#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import struct
import math

class LidarDriver(Node):
    def __init__(self):
        super().__init__('lidar_driver')
        
        self.serial = serial.Serial('/dev/ttyUSB0', 230400, timeout=1)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.01, self.read_data)
        self.buffer = bytearray()
        
        self.get_logger().info('LiDAR Ready')
    
    def read_data(self):
        if self.serial.in_waiting > 0:
            self.buffer.extend(self.serial.read(self.serial.in_waiting))
            self.process_buffer()
    
    def process_buffer(self):
        while len(self.buffer) >= 47:
            if self.buffer[0] == 0x54:
                self.parse_packet(self.buffer[:47])
                self.buffer = self.buffer[47:]
            else:
                self.buffer.pop(0)
    
    def parse_packet(self, packet):
        if len(packet) != 47:
            return
        
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi
        scan.angle_increment = (2.0 * math.pi) / 360.0
        scan.range_min = 0.02
        scan.range_max = 12.0
        
        ranges = [float('inf')] * 360
        
        for i in range(12):
            offset = 4 + i * 3
            distance = struct.unpack('<H', packet[offset:offset+2])[0]
            angle_index = (i * 30) % 360
            if distance > 0:
                ranges[angle_index] = distance / 1000.0
        
        scan.ranges = ranges
        self.scan_pub.publish(scan)
    
    def destroy_node(self):
        self.serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LidarDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
