import rclpy
from rclpy.node import Node
# Import Time for consistent ROS 2 clock usage
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray, Bool, String
import time # Kept for utility, but removed from watchdog logic
import math
import os
from typing import Tuple

# --- Use the compatible lgpio library ---
try:
    import lgpio
    _HAS_LGPIO = True
except Exception:
    _HAS_LGPIO = False

# Utility
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # ---- Parameters (default pins use BCM numbering) ----
        self.declare_parameter('left_pwm', 18)
        self.declare_parameter('left_dir', 23)
        self.declare_parameter('right_pwm', 13)
        self.declare_parameter('right_dir', 25)
        self.declare_parameter('estop_gpio', 5)
        self.declare_parameter('pwm_frequency', 1000)
        self.declare_parameter('pwm_range', 255)
        self.declare_parameter('max_speed_pct', 100.0)
        self.declare_parameter('heartbeat_timeout', 1.0)
        self.declare_parameter('use_simulation', False)

        self.LEFT_PWM   = int(self.get_parameter('left_pwm').value)
        self.LEFT_DIR   = int(self.get_parameter('left_dir').value)
        self.RIGHT_PWM  = int(self.get_parameter('right_pwm').value)
        self.RIGHT_DIR  = int(self.get_parameter('right_dir').value)
        self.ESTOP_GPIO = int(self.get_parameter('estop_gpio').value)
        self.PWM_FREQ   = int(self.get_parameter('pwm_frequency').value)
        self.PWM_RANGE  = int(self.get_parameter('pwm_range').value)
        self.MAX_SPEED  = float(self.get_parameter('max_speed_pct').value)
        self.HEARTBEAT_TIMEOUT = float(self.get_parameter('heartbeat_timeout').value)
        self.SIM = bool(self.get_parameter('use_simulation').value)

        self.get_logger().info(f"motor_control_node starting (lgpio={_HAS_LGPIO}, sim={self.SIM})")
        self.get_logger().info(f"Pins L(PWM,DIR)={self.LEFT_PWM},{self.LEFT_DIR}  R(PWM,DIR)={self.RIGHT_PWM},{self.RIGHT_DIR}")

        self._init_hardware()

        self.sub_cmd = self.create_subscription(Float32MultiArray, '/motor_speeds', self.cb_motor_speeds, 10)
        self.sub_estop = self.create_subscription(Bool, '/emergency_stop', self.cb_emergency_stop, 10)
        self.pub_state = self.create_publisher(String, '/motor_state', 1)

        # FIX 1: Initialize the last command time using the ROS 2 Clock
        self._last_cmd_time = self.get_clock().now()
        self._estop_engaged = False

        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self._watch_callback)
        self.get_logger().info("ROS 2 Watch Timer started")

        self.left_ticks = 0
        self.right_ticks = 0

    def _init_hardware(self):
        if self.SIM:
            self.get_logger().warning("SIMULATION MODE: not touching GPIO")
            self.hw = None
            return

        if not _HAS_LGPIO:
            self.get_logger().error("lgpio library not available. Running in simulation.")
            self.SIM = True
            self.hw = None
            return

        try:
            # Note: We successfully solved the permission issue to get here!
            self.lg_h = lgpio.gpiochip_open(0)
            if self.lg_h < 0:
                raise RuntimeError(f"Could not open lgpio chip (handle: {self.lg_h})")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize lgpio: {e}")
            self.SIM = True
            self.hw = None
            return

        # Configure direction pins as outputs
        output_pins = [p for p in (self.LEFT_DIR, self.RIGHT_DIR) if p >= 0]
        for p in output_pins:
            lgpio.gpio_claim_output(self.lg_h, p)
            lgpio.gpio_write(self.lg_h, p, 0)

        # Configure PWM pins: start at 0% duty using tx_pwm
        lgpio.tx_pwm(self.lg_h, self.LEFT_PWM, self.PWM_FREQ, 0)
        lgpio.tx_pwm(self.lg_h, self.RIGHT_PWM, self.PWM_FREQ, 0)

        # Configure ESTOP pin as input with pull-up
        # if self.ESTOP_GPIO >= 0:
        #    lgpio.gpio_claim_input(self.lg_h, self.ESTOP_GPIO, lgpio.SET_PULL_UP)
            
        self.get_logger().info("lgpio initialized successfully for Cytron driver")
        self.hw = 'lgpio'

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

        left_pct = clamp(left_pct, -self.MAX_SPEED, self.MAX_SPEED)
        right_pct = clamp(right_pct, -self.MAX_SPEED, self.MAX_SPEED)
        
        # FIX 2: Reset the timer using the ROS 2 Clock
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
        left_dir = left_pct >= 0
        right_dir = right_pct >= 0
        left_duty = int(abs(left_pct) * self.PWM_RANGE / 100.0)
        right_duty = int(abs(right_pct) * self.PWM_RANGE / 100.0)

        if self.SIM or self.hw is None:
            # If in simulation, log the command but don't publish the state,
            # so we only publish the state on actual hardware change.
            self.get_logger().info(f"SIMULATING: L: {left_pct:.1f}% ({left_duty}), R: {right_pct:.1f}% ({right_duty})")
            return

        if self.hw == 'lgpio':
            try:
                lgpio.gpio_write(self.lg_h, self.LEFT_DIR, 1 if left_dir else 0)
                lgpio.gpio_write(self.lg_h, self.RIGHT_DIR, 1 if right_dir else 0)
                
                # PWM duty cycle is expected as a percentage (0 to 100) by lgpio.tx_pwm
                lgpio.tx_pwm(self.lg_h, self.LEFT_PWM, self.PWM_FREQ, left_duty / self.PWM_RANGE * 100)
                lgpio.tx_pwm(self.lg_h, self.RIGHT_PWM, self.PWM_FREQ, right_duty / self.PWM_RANGE * 100)
            except Exception as e:
                self.get_logger().error(f"lgpio write failed in _apply_speeds: {e}")
                self.hw = None # Disable hardware interaction if an error occurs
                self._apply_speeds(0.0, 0.0) # Apply zero speed again after disabling
                return
        
        s = f"L{left_pct:.1f},R{right_pct:.1f}"
        self.pub_state.publish(String(data=s))

    def _watch_callback(self):
        try:
            # FIX 3: Get current time using the ROS 2 Clock
            now = self.get_clock().now()
            
            # Calculate difference in seconds
            time_diff_sec = (now - self._last_cmd_time).nanoseconds / 1e9 

            if time_diff_sec > self.HEARTBEAT_TIMEOUT:
                self.get_logger().warn("Motor command timeout. Stopping motors.")
                self._apply_speeds(0.0, 0.0)
            # (Omitted ESTOP check which is currently commented out)
        except Exception as e:
            self.get_logger().error(f"Watch timer callback exception: {e}")

    def destroy_node(self):
        self.get_logger().info("Shutting down motor_control node")
        try:
            self._apply_speeds(0.0, 0.0)
            self.timer.cancel()
            if self.hw == 'lgpio' and self.lg_h >= 0:
                # Clean up PWM on pins
                lgpio.tx_pwm(self.lg_h, self.LEFT_PWM, 0, 0)
                lgpio.tx_pwm(self.lg_h, self.RIGHT_PWM, 0, 0)
                lgpio.gpiochip_close(self.lg_h)
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
