#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import time
import math

class IMUDriver(Node):
    def __init__(self):
        super().__init__('imu_driver')
        
        self.bus = smbus2.SMBus(1)
        self.addr = 0x69
        
        self.bus.write_byte_data(self.addr, 0x7E, 0x11)
        time.sleep(0.05)
        self.bus.write_byte_data(self.addr, 0x7E, 0x15)
        time.sleep(0.1)
        
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.02, self.publish_imu)
        
        self.get_logger().info('IMU Ready')
    
    def read_raw(self, addr):
        low = self.bus.read_byte_data(self.addr, addr)
        high = self.bus.read_byte_data(self.addr, addr + 1)
        value = (high << 8) | low
        if value > 32768:
            value -= 65536
        return value
    
    def publish_imu(self):
        acc_x = self.read_raw(0x12) / 16384.0 * 9.81
        acc_y = self.read_raw(0x14) / 16384.0 * 9.81
        acc_z = self.read_raw(0x16) / 16384.0 * 9.81
        
        gyro_x = (self.read_raw(0x0C) / 131.0) * (math.pi / 180.0)
        gyro_y = (self.read_raw(0x0E) / 131.0) * (math.pi / 180.0)
        gyro_z = (self.read_raw(0x10) / 131.0) * (math.pi / 180.0)
        
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        imu_msg.orientation.w = 1.0
        imu_msg.orientation_covariance[0] = -1.0
        
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z
        
        imu_msg.linear_acceleration.x = acc_x
        imu_msg.linear_acceleration.y = acc_y
        imu_msg.linear_acceleration.z = acc_z
        
        self.imu_pub.publish(imu_msg)
    
    def destroy_node(self):
        self.bus.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = IMUDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
