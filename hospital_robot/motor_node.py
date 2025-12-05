import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# --- Hardware Imports ---
try:
    from gpiozero import PhaseEnableMotor 
    from gpiozero.pins.lgpio import LGPIOFactory
    _HAS_GPIO = True
except ImportError:
    _HAS_GPIO = False

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # ==========================================================
        #        SPEED & TRIM CONFIGURATION (Tune these!)
        # ==========================================================
        self.declare_parameter('speed_calibration', 1.0) # 1.0 cmd = 100% PWM
        
        # If robot drifts RIGHT -> Left wheel is too fast -> Set left_trim to 0.9 or 0.95
        # If robot drifts LEFT  -> Right wheel is too fast -> Set right_trim to 0.9 or 0.95
        self.declare_parameter('left_trim', 3.0)   
        self.declare_parameter('right_trim', 3.0)
        # ==========================================================

        self.CALIBRATION_FACTOR = self.get_parameter('speed_calibration').value
        self.LEFT_TRIM = self.get_parameter('left_trim').value
        self.RIGHT_TRIM = self.get_parameter('right_trim').value

        # Hardware Pins (Cytron MDD10A)
        self.declare_parameter('left_pwm_pin', 12)
        self.declare_parameter('left_dir_pin', 17)
        self.declare_parameter('right_pwm_pin', 13)
        self.declare_parameter('right_dir_pin', 23)
        self.declare_parameter('wheel_separation', 0.30) 

        self.WHEEL_SEP = self.get_parameter('wheel_separation').value
        self.LEFT_PWM = self.get_parameter('left_pwm_pin').value
        self.LEFT_DIR = self.get_parameter('left_dir_pin').value
        self.RIGHT_PWM = self.get_parameter('right_pwm_pin').value
        self.RIGHT_DIR = self.get_parameter('right_dir_pin').value
        
        self.SIM = False
        self._init_hardware()

        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 10)
        self.sub_estop = self.create_subscription(Bool, '/emergency_stop', self.cb_estop, 1)
        self.estop_triggered = False
        
        self.get_logger().info(f"Motor Node Ready. Trim: L={self.LEFT_TRIM}, R={self.RIGHT_TRIM}")

    def _init_hardware(self):
        if not _HAS_GPIO:
            self.SIM = True
            return
        try:
            factory = LGPIOFactory()
            self.left_motor = PhaseEnableMotor(phase=self.LEFT_DIR, enable=self.LEFT_PWM, pin_factory=factory)
            self.right_motor = PhaseEnableMotor(phase=self.RIGHT_DIR, enable=self.RIGHT_PWM, pin_factory=factory)
        except Exception:
            self.SIM = True

    def cb_cmd_vel(self, msg: Twist):
        if self.estop_triggered: return

        # 1. Kinematics
        linear = msg.linear.x
        angular = msg.angular.z
        
        v_left = linear - (angular * self.WHEEL_SEP / 2.0)
        v_right = linear + (angular * self.WHEEL_SEP / 2.0)

        # 2. Calibration (Convert m/s to PWM 0.0-1.0)
        raw_pwm_left = v_left / self.CALIBRATION_FACTOR
        raw_pwm_right = v_right / self.CALIBRATION_FACTOR

        # 3. APPLY TRIM (Fixes the drifting issue)
        final_left = raw_pwm_left * self.LEFT_TRIM
        final_right = raw_pwm_right * self.RIGHT_TRIM

        self.set_motors(final_left, final_right)

    def set_motors(self, l_val, r_val):
        # Clamp result to ensure we don't crash the driver with >1.0
        l_val = clamp(l_val, -1.0, 1.0)
        r_val = clamp(r_val, -1.0, 1.0)

        if not self.SIM:
            self.left_motor.value = l_val
            self.right_motor.value = r_val

    def cb_estop(self, msg: Bool):
        if msg.data:
            self.estop_triggered = True
            self.set_motors(0, 0)
        else:
            self.estop_triggered = False

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


