import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from smbus2 import SMBus
import math

class BMI160Node(Node):
    def __init__(self):
        super().__init__('imu_node')

        # --- I2C Configuration ---
        self.bus_num = 1
        self.address = 0x68 # Check i2cdetect! Might be 0x69
        
        # BMI160 Registers
        self.REG_CMD = 0x7E
        self.REG_ACC_CONF = 0x40
        self.REG_ACC_RANGE = 0x41
        self.REG_GYR_CONF = 0x42
        self.REG_GYR_RANGE = 0x43
        self.REG_DATA = 0x04 # Starting address for data

        # Init Sensor
        try:
            self.bus = SMBus(self.bus_num)
            self._init_sensor()
            self.get_logger().info(f"BMI160 initialized on bus {self.bus_num}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to IMU: {e}")
            self.bus = None

        # Publisher
        self.publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.05, self.read_imu) # 20Hz

    def _init_sensor(self):
        # 1. Soft Reset
        self.bus.write_byte_data(self.address, self.REG_CMD, 0xB6)
        # 2. Set Accelerometer to Normal Mode, 100Hz
        self.bus.write_byte_data(self.address, self.REG_CMD, 0x11) 
        # 3. Set Gyroscope to Normal Mode, 100Hz
        self.bus.write_byte_data(self.address, self.REG_CMD, 0x15)
        # 4. Set Range: Accel +/- 2g, Gyro +/- 250 dps
        # (Default settings usually fine, but explicit is better)

    def read_imu(self):
        if self.bus is None: return

        try:
            # Read 12 bytes: Gx, Gy, Gz, Ax, Ay, Az (2 bytes each)
            data = self.bus.read_i2c_block_data(self.address, self.REG_DATA, 12)

            # --- CONVERT RAW BYTES TO NUMBERS ---
            # Helper to convert two bytes to signed integer
            def to_signed(lo, hi):
                val = (hi << 8) + lo
                if val >= 0x8000: return -((65535 - val) + 1)
                else: return val

            gx = to_signed(data[0], data[1])
            gy = to_signed(data[2], data[3])
            gz = to_signed(data[4], data[5])
            ax = to_signed(data[6], data[7])
            ay = to_signed(data[8], data[9])
            az = to_signed(data[10], data[11])

            # --- SCALING (Based on Defaults) ---
            # Gyro (250dps range): Raw / 131.2 = degrees/sec
            # Accel (2g range): Raw / 16384.0 = g
            
            # ROS needs Rad/s and m/s^2
            deg_to_rad = math.pi / 180.0
            g_to_ms2 = 9.81

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu_link"

            # Angular Velocity (Rad/s)
            msg.angular_velocity.x = (gx / 131.2) * deg_to_rad
            msg.angular_velocity.y = (gy / 131.2) * deg_to_rad
            msg.angular_velocity.z = (gz / 131.2) * deg_to_rad

            # Linear Acceleration (m/s^2)
            msg.linear_acceleration.x = (ax / 16384.0) * g_to_ms2
            msg.linear_acceleration.y = (ay / 16384.0) * g_to_ms2
            msg.linear_acceleration.z = (az / 16384.0) * g_to_ms2

            # Orientation (Quaternions)
            # We leave this empty (0,0,0,0) or identity because 
            # we want the EKF (robot_localization) to calculate it from the gyro.
            # But we must ensure covariance is set so ROS knows to trust/ignore specific values.
            
            # Covariance (The "Trust" Factor)
            # -1 means "I don't have this data"
            msg.orientation_covariance[0] = -1.0 
            
            # We trust Gyro and Accel
            msg.angular_velocity_covariance[0] = 0.01
            msg.linear_acceleration_covariance[0] = 0.01

            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"IMU Read Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BMI160Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()