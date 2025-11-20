import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class Ultrasonic4xNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_4x_node')
        self.port = '/dev/ttyAMA0'
        self.baud = 9600
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        self.get_logger().info(f"Opened serial port: {self.port} @ {self.baud} baud")
        self.publisher = self.create_publisher(Float32MultiArray, 'ultrasonic/distance_array', 10)
        self.timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.read_and_publish)

    def read_and_publish(self):
        try:
            # Synchronize to the 0xFF header
            while True:
                header = self.ser.read(1)
                if not header:
                    return
                if header[0] == 0xFF:
                    # Got frame header, read the next 7 bytes for frame data
                    raw = self.ser.read(7)
                    if len(raw) < 7:
                        self.get_logger().warn("Incomplete frame.")
                        return
                    break
            # Now raw[0:7] are your data bytes. Modify parsing according to your sensor's datasheet!
            d1 = raw[0] + (raw[1] << 8)
            d2 = raw[2] + (raw[3] << 8)
            d3 = raw[4] + (raw[5] << 8)
            d4 = raw[6]
            distances = [float(d1), float(d2), float(d3), float(d4)]
            msg = Float32MultiArray()
            msg.data = distances
            self.publisher.publish(msg)
            self.get_logger().info(f"Published: {distances}")

        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Ultrasonic4xNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
