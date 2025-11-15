import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String
# Changed to standard topic type for navigation stack compatibility
from geometry_msgs.msg import Twist 
import time
import math
import os
from typing import Tuple

# --- Use gpiozero library for cleaner hardware control ---
try:
    from gpiozero import Motor
    _HAS_GPIOZERO = True
except Exception as e:
    # This will catch missing libraries on systems without GPIO
    print(f"GPIOZero library failed to import: {e}")
    _HAS_GPIOZERO = False

# Utility
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # ---- Kinematics Parameters ----
        # TUNE THESE FOR YOUR ROBOT
        self.declare_parameter('wheel_radius', 0.0325)  # 65mm / 2 = 32.5mm = 0.0325m
        self.declare_parameter('wheel_separation', 0.30)  # Distance between centers of the two drive wheels (in meters)
        
        self.WHEEL_RADIUS = self.get_parameter('wheel_radius').value
        self.WHEEL_SEP = self.get_parameter('wheel_separation').value
        
        # ---- Hardware Parameters ----
        self.declare_parameter('left_pwm', 12)
        self.declare_parameter('left_dir', 24)
        self.declare_parameter('right_pwm', 13)
        self.declare_parameter('right_dir', 23)
        self.declare_parameter('max_linear_vel', 1.0) # Max Linear Velocity in m/s (TUNE)
        self.declare_parameter('max_angular_vel', 1.0) # Max Angular Velocity in rad/s (TUNE)
        self.declare_parameter('max_pwm_percent', 100.0) # Max PWM % (MDD10A supports 100%)
        self.declare_parameter('heartbeat_timeout', 0.5) # Reduced timeout for safety
        self.declare_parameter('use_simulation', False)

        self.LEFT_PWM     = int(self.get_parameter('left_pwm').value)
        self.LEFT_DIR     = int(self.get_parameter('left_dir').value)
        self.RIGHT_PWM    = int(self.get_parameter('right_pwm').value)
        self.RIGHT_DIR    = int(self.get_parameter('right_dir').value)
        self.MAX_LIN_VEL  = float(self.get_parameter('max_linear_vel').value)
        self.MAX_ANG_VEL  = float(self.get_parameter('max_angular_vel').value)
        self.MAX_PWM_PCT  = float(self.get_parameter('max_pwm_percent').value)
        self.HEARTBEAT_TIMEOUT = float(self.get_parameter('heartbeat_timeout').value)
        self.SIM          = bool(self.get_parameter('use_simulation').value)

        self.get_logger().info(f"motor_control_node starting (gpiozero={_HAS_GPIOZERO}, sim={self.SIM})")
        self.get_logger().info(f"Kinematics: R={self.WHEEL_RADIUS}, Sep={self.WHEEL_SEP}")

        self._init_hardware()

        # Subscribe to the standard Twist message from Nav2
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 10)
        self.sub_estop = self.create_subscription(Bool, '/emergency_stop', self.cb_emergency_stop, 1)
        self.pub_state = self.create_publisher(String, '/motor_state', 1)

        self._current_state_str = "L0.0,R0.0"
        self._last_cmd_time = self.get_clock().now()
        self._estop_engaged = False
        self._last_linear_x = 0.0
        self._last_angular_z = 0.0

        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self._watch_callback)
        self.get_logger().info("ROS 2 Watch Timer started")

    def _init_hardware(self):
        if self.SIM or not _HAS_GPIOZERO:
            if not _HAS_GPIOZERO:
                self.get_logger().error("gpiozero library not found. Running in simulation.")
            else:
                self.get_logger().warning("SIMULATION MODE: not touching GPIO")
            self.SIM = True
            self.left_motor = None
            self.right_motor = None
            return

        try:
            self.left_motor = Motor(forward=self.LEFT_PWM, backward=self.LEFT_DIR)
            self.right_motor = Motor(forward=self.RIGHT_PWM, backward=self.RIGHT_DIR)
            self.left_motor.stop()
            self.right_motor.stop()
            self.get_logger().info("gpiozero motors initialized successfully.")
            self.hw = 'gpiozero'
        except Exception as e:
            self.get_logger().error(f"Failed to initialize gpiozero motors: {e}")
            self.SIM = True
            self.left_motor = None
            self.right_motor = None
            self.hw = None

    def cb_cmd_vel(self, msg: Twist):
        # Nav2 sends Twist messages: linear.x (forward/backward) and angular.z (rotation)
        linear_x = clamp(msg.linear.x, -self.MAX_LIN_VEL, self.MAX_LIN_VEL)
        angular_z = clamp(msg.angular.z, -self.MAX_ANG_VEL, self.MAX_ANG_VEL)
        
        self.get_logger().info(f"Received Twist: V={linear_x:.2f} m/s, W={angular_z:.2f} rad/s")

        self._last_cmd_time = self.get_clock().now()
        self._last_linear_x = linear_x
        self._last_angular_z = angular_z
        
        if not self._estop_engaged:
            self._calculate_and_apply_speeds(linear_x, angular_z)
        else:
            if linear_x != 0.0 or angular_z != 0.0:
                 self.get_logger().warn("ESTOP engaged; ignoring Twist and commanding 0.")
            self._apply_pwm_percent(0.0, 0.0) 

    def cb_emergency_stop(self, msg: Bool):
        val = bool(msg.data)
        self.get_logger().info(f"Emergency stop message received: {val}. Stopping motors.")
        self._estop_engaged = val
        self._apply_pwm_percent(0.0, 0.0)
        self._last_linear_x = 0.0
        self._last_angular_z = 0.0


    def _calculate_and_apply_speeds(self, linear_x: float, angular_z: float):
        # --- Inverse Kinematics ---
        # Convert robot linear/angular velocity to required wheel speeds (m/s)
        # Vr = V + (W * L) / 2
        # Vl = V - (W * L) / 2
        V_r = linear_x + (angular_z * self.WHEEL_SEP) / 2.0
        V_l = linear_x - (angular_z * self.WHEEL_SEP) / 2.0

        # Convert wheel linear speed (m/s) to wheel angular velocity (rad/s)
        # W_wheel = V / R
        W_l = V_l / self.WHEEL_RADIUS
        W_r = V_r / self.WHEEL_RADIUS
        
        # --- PWM Mapping ---
        # The JGB37-555 motors are 267 RPM @ 12V. 
        # Convert RPM to max angular velocity (rad/s): (267 * 2 * pi) / 60 = 27.98 rad/s
        MAX_ANGULAR_WHEEL_SPEED = (267.0 * 2.0 * math.pi) / 60.0 
        
        # Calculate the required percentage of max physical speed (which translates to PWM duty cycle)
        # Note: We clamp here to ensure we don't try to exceed the motor's physical limit.
        left_pct = clamp((W_l / MAX_ANGULAR_WHEEL_SPEED) * self.MAX_PWM_PCT, 
                         -self.MAX_PWM_PCT, self.MAX_PWM_PCT)
        right_pct = clamp((W_r / MAX_ANGULAR_WHEEL_SPEED) * self.MAX_PWM_PCT, 
                          -self.MAX_PWM_PCT, self.MAX_PWM_PCT)

        self.get_logger().info(
            f"Kinematics Result: L={W_l:.2f} rad/s ({left_pct:.1f}%) | R={W_r:.2f} rad/s ({right_pct:.1f}%)"
        )
        
        self._apply_pwm_percent(left_pct, right_pct)

    def _apply_pwm_percent(self, left_pct: float, right_pct: float):
        # gpiozero.Motor requires a value between -1.0 (full reverse) and 1.0 (full forward)
        scale_factor = 1.0 / self.MAX_PWM_PCT
        left_speed_val = left_pct * scale_factor
        right_speed_val = right_pct * scale_factor # CORRECTED: Use right_pct
        
        if self.SIM or self.hw is None:
            return

        if self.hw == 'gpiozero':
            try:
                self.left_motor.value = left_speed_val
                self.right_motor.value = right_speed_val
            except Exception as e:
                self.get_logger().error(f"gpiozero write failed in _apply_speeds: {e}")
                self._cleanup_motors()
                self.hw = None 
                return
        
        s = f"L{left_pct:.1f},R{right_pct:.1f}"
        self._current_state_str = s
        self.pub_state.publish(String(data=s))

    def _watch_callback(self):
        try:
            now = self.get_clock().now()
            time_diff_sec = (now - self._last_cmd_time).nanoseconds / 1e9 

            if time_diff_sec > self.HEARTBEAT_TIMEOUT:
                if self._current_state_str != "L0.0,R0.0":
                    self.get_logger().warn("Motor command timeout. Stopping motors.")
                    self._apply_pwm_percent(0.0, 0.0)
        except Exception as e:
            self.get_logger().error(f"Watch timer callback exception: {e}") 

    def _cleanup_motors(self):
        if self.left_motor:
            self.left_motor.close()
        if self.right_motor:
            self.right_motor.close()

    def destroy_node(self):
        self.get_logger().info("Shutting down motor_control node")
        try:
            self._apply_pwm_percent(0.0, 0.0)
            self.timer.cancel()
            
            if self.hw == 'gpiozero':
                self._cleanup_motors()
                
        except Exception as e:
            self.get_logger().warn(f"Error during cleanup: {e}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()