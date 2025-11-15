import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from flask import Flask, render_template, request, jsonify, send_file
import threading
import time
import os
from io import BytesIO
from PIL import Image
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
import json # <-- ADDED IMPORT

# --- Configuration ---
WEB_SERVER_PORT = 5000 

# Get the absolute paths for Flask resources using the ROS package index
try:
    pkg_share_dir = get_package_share_directory('hospital_robot')
    TEMPLATES_DIR = os.path.join(pkg_share_dir, 'templates')
    STATIC_DIR = os.path.join(pkg_share_dir, 'static')
except Exception as e:
    # Fallback/Debug path if ament index fails (less reliable)
    print(f"ROS package index failed, using relative path fallback. Error: {e}")
    WEB_INTERFACE_DIR = os.path.dirname(os.path.abspath(__file__))
    TEMPLATES_DIR = os.path.join(WEB_INTERFACE_DIR, 'templates')
    STATIC_DIR = os.path.join(WEB_INTERFACE_DIR, 'static')

# Flask initialization now points to the correct template/static paths
app = Flask(
    __name__, 
    template_folder=TEMPLATES_DIR,
    static_folder=STATIC_DIR
)

# --- Global Flask App and ROS Node Instance ---
ros_node = None

class WebServerNode(Node):
    """
    ROS 2 Node responsible for handling web requests, publishing motor commands,
    and managing the delivery pipeline.
    """
    def __init__(self):
        super().__init__('web_server_ros_node')
        
        # Publishers
        # 1. Standard ROS 2 Velocity Command Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 2. Delivery Order Publisher
        self.order_pub = self.create_publisher(String, '/delivery_orders', 10) 
        
        self.get_logger().info('ROS Node for Web Server initialized.')
        
        # Simple In-Memory Order Queue (Non-persistent, for demonstration)
        self.order_queue = []

    def publish_twist_command(self, linear_x, angular_z):
        """Publishes a Twist message for manual control."""
        # CHANGED: Reverted debugging ERROR log to a less verbose DEBUG log
        self.get_logger().debug(f"ATTEMPTING TO PUBLISH: L={linear_x}, A={angular_z}")
        twist = Twist()
        # Linear velocity (forward/backward)
        twist.linear.x = float(linear_x)
        # Angular velocity (turning)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)
        self.get_logger().debug(f"Published Twist: L:{linear_x:.2f}, A:{angular_z:.2f}")

    def publish_stop_command(self):
        """Publishes a zero Twist message (Emergency Stop)."""
        self.publish_twist_command(0.0, 0.0)
        self.get_logger().warn('EMERGENCY STOP command published (Twist zeroed).')
    
    def publish_new_order(self, order_data):
        """Publishes a new delivery order request."""
        # Note: You should check battery status here as discussed previously
        
        order_str = f"DEST:{order_data['destination']}|PATIENT:{order_data['patient']}|DRAWER:{order_data['drawer']}|CONTENTS:{order_data['contents']}"
        
        msg = String()
        msg.data = order_str
        self.order_pub.publish(msg)

        # Add to local queue model for display
        self.order_queue.append({
            'id': time.time(),
            'status': 'QUEUED',
            'data': order_data
        })
        self.get_logger().info(f"New Order published: {order_data['destination']}")
        return True

# --- Flask Routes (Web Interface) ---

@app.route('/', methods=['GET'])
def index():
    """Serves the main control dashboard (index.html)."""
    # Pass a copy of the queue to avoid modification issues
    return render_template('index.html', orders=ros_node.order_queue.copy())

@app.route('/manual', methods=['GET'])
def manual_control():
    """Serves the manual control page (manual_control.html)."""
    return render_template('manual_control.html')

# --- API Endpoints ---

@app.route('/api/orders', methods=['POST'])
def create_order():
    """API endpoint to receive new order data from the dashboard."""
    data = request.get_json()
    if not all(k in data for k in ('destination', 'patient', 'drawer', 'contents')):
        return jsonify({'ok': False, 'error': 'Missing required fields'}), 400
    
    success = ros_node.publish_new_order(data)
    
    if success:
        return jsonify({'ok': True, 'message': 'Order successfully queued via ROS topic.'}), 200
    else:
        return jsonify({'ok': False, 'error': 'ROS publication failed.'}), 500

@app.route('/api/stop', methods=['POST'])
def emergency_stop():
    """API endpoint for the Emergency Stop button."""
    ros_node.publish_stop_command()
    return jsonify({'ok': True, 'message': 'Stop command sent via /cmd_vel topic.'}), 200

@app.route('/api/move', methods=['POST'])
def manual_move():
    """API endpoint to receive linear/angular commands for manual drive."""
    raw_data = request.data
    # CHANGED: Reverted debugging ERROR log to a less verbose DEBUG log
    ros_node.get_logger().debug(f"RAW DATA RECEIVED (bytes): {raw_data}")

    try:
        # 2. Manually decode the raw bytes data into a JSON dictionary
        data = json.loads(raw_data.decode('utf-8'))
        # CHANGED: Reverted debugging ERROR log to a less verbose DEBUG log
        ros_node.get_logger().debug(f"DECODED JSON DICT: {data}")
        
        # 3. Proceed with extracting values (Keys are correct now)
        linear_x = float(data.get('linear_x', 0.0))
        angular_z = float(data.get('angular_z', 0.0))
        
    except Exception as e:
        # Keep this as an ERROR, as a failure to decode JSON is a real problem
        ros_node.get_logger().error(f"JSON DECODING FAILED: {e}")
        linear_x = 0.0
        angular_z = 0.0
        return jsonify({'ok': False, 'error': f'JSON/Parsing Error: {e}'}), 400

    # CHANGED: Reverted debugging ERROR log to a less verbose INFO log
    ros_node.get_logger().info(f"Publishing Command: L={linear_x:.2f}, A={angular_z:.2f}")
    
    # 4. Publish the command
    ros_node.publish_twist_command(linear_x, angular_z)
    
    return jsonify({'ok': True, 'message': 'Twist command published.', 'linear': linear_x, 'angular': angular_z}), 200


# Placeholder for map image
@app.route('/map.png')
def serve_map():
    """Serves a map image (placeholder)."""
    
    if os.path.exists(STATIC_DIR) and os.path.exists(os.path.join(STATIC_DIR, 'map_current.png')):
        # Serve the actual map if it exists in the install path
        map_path = os.path.join(STATIC_DIR, 'map_current.png')
        return send_file(map_path, mimetype='image/png')
    else:
        # Create and return a 1x1 transparent image placeholder if not found
        img = Image.new('RGBA', (1, 1), (0, 0, 0, 0))
        buffer = BytesIO()
        img.save(buffer, format='PNG')
        buffer.seek(0)
        return buffer.getvalue(), 200, {'Content-Type': 'image/png'}

# --- Main Entry Points ---

def run_flask_app():
    """Runs the Flask server in a separate thread."""
    # Use '0.0.0.0' to allow access from other devices on the network
    app.run(host='0.0.0.0', port=WEB_SERVER_PORT)

def main(args=None):
    global ros_node
    os.environ['ROS_DOMAIN_ID'] = '12'
    # 1. Initialize ROS 2
    rclpy.init(args=args)
    ros_node = WebServerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)

    # 2. Start Flask Server in a background thread
    flask_thread = threading.Thread(target=run_flask_app)
    flask_thread.daemon = True 
    flask_thread.start()
    ros_node.get_logger().info(f"Web Server started on http://0.0.0.0:{WEB_SERVER_PORT}")

    # 3. Spin ROS node to listen for shutdowns
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 4. Shutdown ROS 2
        executor.shutdown()
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()