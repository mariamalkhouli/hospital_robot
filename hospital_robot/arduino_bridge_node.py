import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import serial
import threading

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        self.declare_parameter('port', '/dev/ttyACM0')
        self.port = self.get_parameter('port').value
        self.baud = 9600
        
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.get_logger().info("Arduino Bridge Connected")
        except:
            self.ser = None

        # --- SUBSCRIBERS (Commands from Brain/Web) ---
        
        # 1. Start Medicine Workflow (No data needed, just a trigger)
        # Topic: /delivery/start_medicine
        self.create_subscription(Bool, '/delivery/start_medicine', self.start_med_callback, 10)
        
        # 2. Start Drawer Workflow
        # Topic: /delivery/start_drawer
        self.create_subscription(Bool, '/delivery/start_drawer', self.start_drawer_callback, 10)

        # --- PUBLISHERS (Status Updates) ---
        self.status_pub = self.create_publisher(String, '/delivery/status', 10)

        self.thread = threading.Thread(target=self.read_serial)
        self.thread.daemon = True
        self.thread.start()

    def start_med_callback(self, msg):
        if msg.data:
            self.send_serial("<CMD:MED>")
            self.get_logger().info("Sent: Start Medicine Mode")

    def start_drawer_callback(self, msg):
        if msg.data:
            self.send_serial("<CMD:DRAWER>")
            self.get_logger().info("Sent: Start Drawer Mode")

    def send_serial(self, cmd):
        if self.ser: self.ser.write(cmd.encode('utf-8'))

    def read_serial(self):
        while rclpy.ok() and self.ser:
            if self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line.startswith("<STATUS:"):
                        # Format: <STATUS:DONE_MED>
                        status = line.split(':')[1].replace('>', '')
                        
                        msg = String()
                        msg.data = status
                        self.status_pub.publish(msg)
                        self.get_logger().info(f"Task Finished: {status}")
                except:
                    pass

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()