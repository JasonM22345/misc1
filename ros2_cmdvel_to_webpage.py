import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from flask import Flask, jsonify
import threading
import time

class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )
        self.current_twist = Twist()
        self.lock = threading.Lock()

    def listener_callback(self, msg):
        with self.lock:
            self.current_twist = msg

    def get_current_twist(self):
        with self.lock:
            return self.current_twist

# Initialize Flask web server
app = Flask(__name__)
cmd_vel_data = {'linear_x': 0.0, 'linear_y': 0.0, 'linear_z': 0.0, 'angular_x': 0.0, 'angular_y': 0.0, 'angular_z': 0.0}

@app.route('/')
def home():
    return jsonify(cmd_vel_data)

def run_web_server():
    app.run(host='0.0.0.0', port=5000, debug=False)

def main():
    rclpy.init()
    node = CmdVelSubscriber()

    # Start the Flask web server in a separate thread
    web_server_thread = threading.Thread(target=run_web_server)
    web_server_thread.daemon = True
    web_server_thread.start()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)

            # Update cmd_vel data for the web server
            twist = node.get_current_twist()
            cmd_vel_data['linear_x'] = twist.linear.x
            cmd_vel_data['linear_y'] = twist.linear.y
            cmd_vel_data['linear_z'] = twist.linear.z
            cmd_vel_data['angular_x'] = twist.angular.x
            cmd_vel_data['angular_y'] = twist.angular.y
            cmd_vel_data['angular_z'] = twist.angular.z

            time.sleep(1)  # Update every 1 second
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
