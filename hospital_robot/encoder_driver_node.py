import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import time
import math
# IMPORTANT: gpiozero is highly recommended for interrupt-driven counting on SBCs
from gpiozero import DigitalInputDevice 

# --- HARDWARE PARAMETERS ---
ENCODER_LEFT_PIN = 17  
ENCODER_RIGHT_PIN = 27 

# Kinematics & Encoder Constants
WHEEL_RADIUS_M = 0.0325  # R = 3.25 cm
WHEEL_SEPARATION_M = 0.30  # L = 30 cm
TICKS_PER_REVOLUTION = 15  # PPR = 15 (placeholder)
METERS_PER_TICK = (2 * math.pi * WHEEL_RADIUS_M) / TICKS_PER_REVOLUTION

class EncoderDriverNode(Node):
    def __init__(self):
        super().__init__('encoder_driver_node')

        # 1. ROS 2 Setup
        self.odom_publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster_ = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Odometry calculation timer (e.g., 50 Hz)
        self.timer_period = 0.02 
        self.odom_timer_ = self.create_timer(self.timer_period, self.odom_calculate_and_publish)

        # 2. State Variables
        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        
        self.x = 0.0  # Current x position
        self.y = 0.0  # Current y position
        self.theta = 0.0 # Current orientation
        self.last_time = self.get_clock().now()

        # CRITICAL for single-channel encoder: tracks direction from motor command
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0

        # 3. GPIO Setup (Interrupts)
        try:
            # Set up the inputs using pull_up=True if they are connected as active-low
            self.encoder_left = DigitalInputDevice(ENCODER_LEFT_PIN, pull_up=True)
            self.encoder_right = DigitalInputDevice(ENCODER_RIGHT_PIN, pull_up=True)

            # Assign callback functions to be called on *any* edge detection (LOW->HIGH or HIGH->LOW)
            self.encoder_left.when_activated = self.left_tick_callback
            self.encoder_left.when_deactivated = self.left_tick_callback
            self.encoder_right.when_activated = self.right_tick_callback
            self.encoder_right.when_deactivated = self.right_tick_callback
            self.get_logger().info("Encoder inputs initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize GPIO inputs: {e}")

    # --- GPIO Interrupt Callbacks (Fast, low-level counting) ---
    def left_tick_callback(self):
        # We only count, direction is handled in the main Odom loop
        self.left_ticks += 1

    def right_tick_callback(self):
        self.right_ticks += 1

    # --- ROS 2 Callbacks (Slow, for direction) ---
    def cmd_vel_callback(self, msg):
        # Store the received command velocity to infer direction for single-channel encoders
        self.current_linear_x = msg.linear.x
        self.current_angular_z = msg.angular.z

    # --- Odometry Calculation and Publishing (Main loop) ---
    def odom_calculate_and_publish(self):
        current_time = self.get_clock().now()
        dt_sec = (current_time - self.last_time).nanoseconds / 1e9

        if dt_sec == 0:
            return

        # 1. Calculate Ticks and Sign (The single-channel assumption)
        d_ticks_left = self.left_ticks - self.last_left_ticks
        d_ticks_right = self.right_ticks - self.last_right_ticks
        
        # Apply Direction: If linear x velocity is negative, assume backward motion for ticks
        left_sign = 1.0 if self.current_linear_x >= 0 else -1.0
        right_sign = 1.0 if self.current_linear_x >= 0 else -1.0
        
        # Handle turn-in-place (angular velocity dominates)
        # Crude correction: If turning sharply, the inner wheel's direction may be opposite.
        if abs(self.current_linear_x) < 0.05 and abs(self.current_angular_z) > 0.1:
            if self.current_angular_z > 0: # Turn left (Right wheel forward, Left wheel backward)
                left_sign = -1.0
            else: # Turn right (Left wheel forward, Right wheel backward)
                right_sign = -1.0

        # 2. Calculate Distance
        dl = d_ticks_left * METERS_PER_TICK * left_sign
        dr = d_ticks_right * METERS_PER_TICK * right_sign
        
        # 3. Kinematics (Differential Drive Model)
        dc = (dr + dl) / 2.0  # Center distance traveled
        dtheta = (dr - dl) / WHEEL_SEPARATION_M # Change in angle
        
        vx = dc / dt_sec # Linear velocity in x
        vth = dtheta / dt_sec # Angular velocity in z
        
        # Integrate (Update Pose)
        self.x += dc * math.cos(self.theta + dtheta / 2.0)
        self.y += dc * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta

        # 4. Prepare and Publish Messages
        
        # Convert yaw (theta) to quaternion
        odom_quat = self.e_to_q(0, 0, self.theta)

        # Build TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = odom_quat[0]
        t.transform.rotation.y = odom_quat[1]
        t.transform.rotation.z = odom_quat[2]
        t.transform.rotation.w = odom_quat[3]
        self.tf_broadcaster_.sendTransform(t)

        # Build Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = t.transform.rotation # Use the same quaternion
        
        # Velocity
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        
        self.odom_publisher_.publish(odom)

        # 5. Update state for next cycle
        self.last_left_ticks = self.left_ticks
        self.last_right_ticks = self.right_ticks
        self.last_time = current_time

    # Helper function to convert Euler (yaw) to Quaternion
    def e_to_q(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0.0] * 4
        q[0] = sr * cp * cy - cr * sp * sy # x
        q[1] = cr * sp * cy + sr * cp * sy # y
        q[2] = cr * cp * sy - sr * sp * cy # z
        q[3] = cr * cp * cy + sr * sp * sy # w
        return q

def main(args=None):
    rclpy.init(args=args)
    encoder_driver_node = EncoderDriverNode()
    rclpy.spin(encoder_driver_node)
    encoder_driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()