import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Range
import serial
import time

class Ultrasonic4xNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_4x_node')
        
        # Configuration
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.port = self.get_parameter('serial_port').value
        self.baud = 9600
        
        # Connect to Hardware
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"Connected to {self.port}")
        except Exception as e:
            self.get_logger().error(f"Hardware Fail: {e}")
            self.ser = None

        # --- PUBLISHERS ---
        # 1. Keep the Array for easy debugging (shows all 4 at once)
        self.array_pub = self.create_publisher(Float32MultiArray, 'ultrasonic/raw_array', 10)
        
        # 2. Standard ROS Range Publishers (For Nav2 / Obstacle Avoidance)
        self.range_pubs = [
            self.create_publisher(Range, 'ultrasonic/sensor_1', 10),
            self.create_publisher(Range, 'ultrasonic/sensor_2', 10),
            self.create_publisher(Range, 'ultrasonic/sensor_3', 10),
            self.create_publisher(Range, 'ultrasonic/sensor_4', 10)
        ]
        
        # Frame IDs (Must match your URDF later!)
        self.frames = ['us_link_1', 'us_link_2', 'us_link_3', 'us_link_4']

        self.timer = self.create_timer(0.05, self.read_data)

    def read_data(self):
        if not self.ser: return

        try:
            if self.ser.in_waiting > 0:
                byte = self.ser.read(1)
                if byte == b'\xff': 
                    # Attempt to read the rest of the packet
                    raw = self.ser.read(10) 
                    if len(raw) >= 7:
                        self.parse_and_publish(raw)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def parse_and_publish(self, raw):
        # High Byte First Logic
        d = [0, 0, 0, 0]
        d[0] = (raw[0] << 8) + raw[1]
        d[1] = (raw[2] << 8) + raw[3]
        d[2] = (raw[4] << 8) + raw[5] if len(raw) >= 6 else 0
        d[3] = (raw[6] << 8) + raw[7] if len(raw) >= 8 else 0

        # Convert to Meters
        ranges_m = [x / 1000.0 for x in d]

        # 1. Publish Raw Array (For you to see in logs)
        arr_msg = Float32MultiArray()
        arr_msg.data = ranges_m
        self.array_pub.publish(arr_msg)

        # 2. Publish Standard Range Messages (For the Robot Brain)
        current_time = self.get_clock().now().to_msg()
        
        for i in range(4):
            range_msg = Range()
            range_msg.header.stamp = current_time
            range_msg.header.frame_id = self.frames[i]
            
            # Sensor Properties (Adjust based on your specific sensor model)
            range_msg.radiation_type = Range.ULTRASOUND
            range_msg.field_of_view = 0.52  # approx 30 degrees
            range_msg.min_range = 0.03      # 3cm
            range_msg.max_range = 4.00      # 4m
            
            range_msg.range = ranges_m[i]
            
            self.range_pubs[i].publish(range_msg)

        # Log to console for sanity check
        self.get_logger().info(f"Ranges(m): {ranges_m}")

def main(args=None):
    rclpy.init(args=args)
    node = Ultrasonic4xNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()