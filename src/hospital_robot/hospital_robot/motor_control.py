import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray, Bool, String
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

        # ---- Parameters (default pins use BCM numbering) ----
        # NOTE: gpiozero.Motor uses a forward_pin and a backward_pin.
        # Since the MDD10A uses PWM+DIR, we use the PWM pin as the "forward" pin
        # and the DIR pin as the "backward" pin. The Motor object is smart enough
        # to apply PWM to one and hold the other high/low for direction control.
        # Pin assignment for MDD10A:
        # LEFT_PWM (Speed) -> gpiozero Motor "forward" pin (Pin 18 BCM)
        # LEFT_DIR (Direction) -> gpiozero Motor "backward" pin (Pin 23 BCM)

        self.declare_parameter('left_pwm', 12)
        self.declare_parameter('left_dir', 24)
        self.declare_parameter('right_pwm', 13)
        self.declare_parameter('right_dir', 23)
        self.declare_parameter('estop_gpio', 5)
        self.declare_parameter('pwm_frequency', 1000) # This setting is ignored by gpiozero, which uses pigpio's default
        self.declare_parameter('max_speed_pct', 100.0)
        self.declare_parameter('heartbeat_timeout', 1.0)
        self.declare_parameter('use_simulation', False)

        self.LEFT_PWM   = int(self.get_parameter('left_pwm').value)
        self.LEFT_DIR   = int(self.get_parameter('left_dir').value)
        self.RIGHT_PWM  = int(self.get_parameter('right_pwm').value)
        self.RIGHT_DIR  = int(self.get_parameter('right_dir').value)
        self.ESTOP_GPIO = int(self.get_parameter('estop_gpio').value)
        self.PWM_FREQ   = int(self.get_parameter('pwm_frequency').value)
        self.MAX_SPEED  = float(self.get_parameter('max_speed_pct').value)
        self.HEARTBEAT_TIMEOUT = float(self.get_parameter('heartbeat_timeout').value)
        self.SIM = bool(self.get_parameter('use_simulation').value)

        self.get_logger().info(f"motor_control_node starting (gpiozero={_HAS_GPIOZERO}, sim={self.SIM})")
        self.get_logger().info(f"Pins L(PWM,DIR)={self.LEFT_PWM},{self.LEFT_DIR}   R(PWM,DIR)={self.RIGHT_PWM},{self.RIGHT_DIR}")

        self._init_hardware()

        self.sub_cmd = self.create_subscription(Float32MultiArray, '/motor_speeds', self.cb_motor_speeds, 10)
        self.sub_estop = self.create_subscription(Bool, '/emergency_stop', self.cb_emergency_stop, 10)
        self.pub_state = self.create_publisher(String, '/motor_state', 1)

        self._current_state_str = "L0.0,R0.0"
        self._last_cmd_time = self.get_clock().now()
        self._estop_engaged = False

        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self._watch_callback)
        self.get_logger().info("ROS 2 Watch Timer started")

    def _init_hardware(self):
        if self.SIM:
            self.get_logger().warning("SIMULATION MODE: not touching GPIO")
            self.left_motor = None
            self.right_motor = None
            return

        if not _HAS_GPIOZERO:
            self.get_logger().error("gpiozero library not available. Running in simulation.")
            self.SIM = True
            return

        try:
            # Initialize motors using BCM pin numbers
            # This implicitly initializes the underlying pigpio library
            # NOTE: For Cytron MDD10A (PWM+DIR), we use the Motor class. 
            # The first pin (forward) is typically the PWM pin, and the second (backward) is the DIR pin.
            # We are using the pins defined in the parameters.
            self.left_motor = Motor(forward=self.LEFT_PWM, backward=self.LEFT_DIR)
            self.right_motor = Motor(forward=self.RIGHT_PWM, backward=self.RIGHT_DIR)
            
            # Start stopped
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
            return


    def cb_motor_speeds(self, msg: Float32MultiArray):
        try:
            data = msg.data
            if len(data) < 2:
                self.get_logger().warn("motor_speeds msg malformed (need 2 values)")
                return
            left_pct = float(data[0])
            right_pct = float(data[1])
        except Exception as e:
            self.get_logger().error(f"Failed parse motor_speeds: {e}")
            return

        # Clamp speed to MAX_SPEED (-100 to 100 default)
        left_pct = clamp(left_pct, -self.MAX_SPEED, self.MAX_SPEED)
        right_pct = clamp(right_pct, -self.MAX_SPEED, self.MAX_SPEED)
        
        # Log the raw speeds received
        self.get_logger().info(f"Received speeds (clamped): L={left_pct:.1f}, R={right_pct:.1f}")
        
        self._last_cmd_time = self.get_clock().now()
        
        if not self._estop_engaged:
            self._apply_speeds(left_pct, right_pct)
        else:
            if left_pct != 0.0 or right_pct != 0.0:
                self.get_logger().warn("Received motor_speeds but ESTOP is engaged; ignoring and commanding 0.")
            self._apply_speeds(0.0, 0.0) 

    def cb_emergency_stop(self, msg: Bool):
        val = bool(msg.data)
        self.get_logger().info(f"Emergency stop message received: {val}")
        self._estop_engaged = val
        if val:
            self._apply_speeds(0.0, 0.0)

    def _apply_speeds(self, left_pct: float, right_pct: float):
        # gpiozero.Motor requires a value between -1.0 (full reverse) and 1.0 (full forward)
        # We scale the incoming percentage (e.g., -50 to 50) to this range.
        scale_factor = 1.0 / self.MAX_SPEED
        left_speed_val = left_pct * scale_factor
        right_speed_val = 100 * scale_factor
        
        # CRITICAL DEBUGGING LOG: Log the scaled values being sent to gpiozero
        self.get_logger().info(
            f"Applying HW Speeds (gpiozero scale): L={left_speed_val:.3f} | R={right_speed_val:.3f}"
        )

        if self.SIM or self.hw is None:
            return

        if self.hw == 'gpiozero':
            try:
                # The value is passed to the Motor.value property, which handles direction and PWM duty cycle internally
                self.left_motor.value = left_speed_val
                self.right_motor.value = right_speed_val
            except Exception as e:
                self.get_logger().error(f"gpiozero write failed in _apply_speeds: {e}")
                # Clean up and disable hardware interaction if an error occurs
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
                    self._apply_speeds(0.0, 0.0)
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
            self._apply_speeds(0.0, 0.0)
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
