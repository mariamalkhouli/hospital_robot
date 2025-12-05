#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
import math

class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        
        self.wheel_separation = 0.20
        self.wheel_radius = 0.0325
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0
        
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.last_time = self.get_clock().now()
        
        self.left_sub = self.create_subscription(Float32, 'wheel/left/speed', self.left_callback, 10)
        self.right_sub = self.create_subscription(Float32, 'wheel/right/speed', self.right_callback, 10)
        
        self.odom_pub = self.create_publisher(Odometry, 'wheel/odom', 10)
        #self.tf_broadcaster = TransformBroadcaster(self)
        
        self.timer = self.create_timer(0.02, self.update_odom)
        self.get_logger().info('Wheel Odometry Ready')
    
    def left_callback(self, msg):
        self.left_speed = msg.data
    
    def right_callback(self, msg):
        self.right_speed = msg.data
    
    def update_odom(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        linear = (self.right_speed + self.left_speed) / 2.0
        angular = (self.right_speed - self.left_speed) / self.wheel_separation
        
        delta_x = linear * math.cos(self.theta) * dt
        delta_y = linear * math.sin(self.theta) * dt
        delta_theta = angular * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        self.vx = linear
        self.vth = angular
        
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        q = self.euler_to_quat(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth
        
        self.odom_pub.publish(odom)
        
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        #self.tf_broadcaster.sendTransform(t)
        
        self.last_time = current_time
    
    def euler_to_quat(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=None)
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
