#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import board
import busio
import adafruit_vl6180x
import math

class CliffDetector(Node):
    def __init__(self):
        super().__init__('cliff_detector')
        
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_vl6180x.VL6180X(i2c)
        
        self.sensor_height = 0.085
        self.sensor_angle = 45.0
        angle_rad = math.radians(self.sensor_angle)
        self.expected_distance = self.sensor_height / math.sin(angle_rad)
        self.cliff_threshold = 0.05
        
        self.cliff_pub = self.create_publisher(Bool, 'cliff_detected', 10)
        self.timer = self.create_timer(0.05, self.check_cliff)
        
        self.cliff_detected = False
        self.consecutive = 0
        
        self.get_logger().info('Cliff Detector Ready')
    
    def check_cliff(self):
        distance = self.sensor.range / 1000.0
        
        is_cliff = False
        if distance > (self.expected_distance + self.cliff_threshold):
            is_cliff = True
        elif self.sensor.range >= 255:
            is_cliff = True
        
        if is_cliff:
            self.consecutive += 1
            if self.consecutive >= 3:
                if not self.cliff_detected:
                    self.get_logger().warn('CLIFF DETECTED')
                    self.cliff_detected = True
        else:
            self.consecutive = 0
            if self.cliff_detected:
                self.get_logger().info('Ground Safe')
                self.cliff_detected = False
        
        self.cliff_pub.publish(Bool(data=self.cliff_detected))

def main(args=None):
    rclpy.init(args=args)
    node = CliffDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
