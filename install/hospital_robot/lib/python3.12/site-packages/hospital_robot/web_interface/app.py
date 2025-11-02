import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
# Changed import to include render_template
from flask import Flask, render_template, request, jsonify, send_file
import threading
import time
import os
from io import BytesIO
# Added PIL for map placeholder, needed if a real map file doesn't exist
from PIL import Image 

# --- Configuration ---
# Flask automatically looks for 'templates' and 'static' relative to the instance path.
# We set the template_folder path correctly relative to the web_interface directory.
WEB_INTERFACE_DIR = os.path.dirname(os.path.abspath(__file__))
TEMPLATES_DIR = os.path.join(WEB_INTERFACE_DIR, 'templates')
STATIC_DIR = os.path.join(WEB_INTERFACE_DIR, 'static')

# Flask initialization now points to the correct template/static paths
app = Flask(
    __name__, 
    template_folder=TEMPLATES_DIR,
    static_folder=STATIC_DIR
)
WEB_SERVER_PORT = 5000 

# --- Global Flask App and ROS Node Instance ---
ros_node = None

class WebServerNode(Node):
    """
    ROS 2 Node responsible for handling web requests and publishing motor commands.
    """
    def __init__(self):
        super().__init__('web_server_ros_node')
        
        # Publishers
        self.motor_pub = self.create_publisher(Float32MultiArray, '/motor_speeds', 10)
        self.order_pub = self.create_publisher(String, '/delivery_orders', 10) 
        
        self.get_logger().info('ROS Node for Web Server initialized.')
        
        # Simple In-Memory Order Queue (Non-persistent, for demonstration)
        self.order_queue = []

    def publish_stop_command(self):
        """Publishes zero speeds to the motor control node (Emergency Stop)."""
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0]
        self.motor_pub.publish(msg)
        self.get_logger().warn('EMERGENCY STOP command published.')
    
    def publish_new_order(self, order_data):
        """Publishes a new delivery order request."""
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
    """Serves the main control dashboard (index.html) and passes the order queue."""
    # Flask's render_template looks in the configured templates_folder
    return render_template('index.html', orders=ros_node.order_queue)

@app.route('/manual', methods=['GET'])
def manual_control():
    # Placeholder for a manual control page (assume manual_control.html exists in templates)
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
    return jsonify({'ok': True, 'message': 'Stop command sent via /motor_speeds topic.'}), 200

# Placeholder for map image
@app.route('/map.png')
def serve_map():
    # In a real system, this would load the latest map PNG.
    map_path = os.path.join(STATIC_DIR, 'map_current.png')
    
    if os.path.exists(map_path):
        # Serve the actual map if it exists
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
    app.run(host='0.0.0.0', port=WEB_SERVER_PORT)

def main(args=None):
    global ros_node
    
    # 1. Initialize ROS 2
    rclpy.init(args=args)
    ros_node = WebServerNode()

    # 2. Start Flask Server in a background thread
    flask_thread = threading.Thread(target=run_flask_app)
    flask_thread.daemon = True 
    flask_thread.start()
    ros_node.get_logger().info(f"Web Server started on http://0.0.0.0:{WEB_SERVER_PORT}")

    # 3. Spin ROS node to listen for shutdowns
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 4. Shutdown ROS 2
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
