"""
This script integrates ROS2 and a Flask web server to provide a real-time interface for viewing the current map
data from a ROS2 `map` topic. It subscribes to the `map` topic (OccupancyGrid messages), processes the map data, 
and displays it on a webpage. The map updates dynamically as the data on the ROS2 topic changes. The Flask web 
server provides routes for retrieving the map image and metadata in JSON format.

Key features:
- Subscribes to the ROS2 `map` topic to receive occupancy grid data.
- Hosts a Flask web server to provide a webpage interface for visualizing the map.
- Dynamically encodes and displays map updates in real-time.
- Runs the ROS2 node and Flask web server concurrently using multithreading.
"""

import rclpy  # ROS2 Python library for node creation and communication
from rclpy.node import Node  # Base class for creating ROS2 nodes
from nav_msgs.msg import OccupancyGrid  # ROS2 message type for occupancy grids (maps)
from flask import Flask, render_template, jsonify  # Flask web framework
import threading  # Used for running the web server and ROS2 node concurrently
import numpy as np  # Numerical operations for processing map data
import base64  # Encoding binary data into Base64 for web transmission
import cv2  # OpenCV for image processing (imported but not used here)
from io import BytesIO  # In-memory binary streams for efficient data handling
from PIL import Image  # For image processing and encoding map data as PNG

class MapSubscriber(Node):
    """
    ROS2 node that subscribes to the `map` topic and processes the occupancy grid data.
    Stores the map data and metadata for retrieval by the Flask server.
    """
    def __init__(self):
        super().__init__('map_subscriber')  # Initialize the node with the name 'map_subscriber'
        self.subscription = self.create_subscription(
            OccupancyGrid,  # Message type to subscribe to
            'map',  # Topic name
            self.listener_callback,  # Callback function for incoming messages
            10  # Queue size for storing messages
        )
        self.map_data = None  # Stores the latest map data
        self.map_meta = None  # Stores the latest map metadata
        self.lock = threading.Lock()  # Lock for thread-safe access to map data

    def listener_callback(self, msg):
        """
        Callback function for the `map` topic. Processes and stores the map data and metadata.
        """
        with self.lock:  # Ensure thread-safe access
            self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))  # Reshape map data
            self.map_meta = {  # Store metadata
                'resolution': msg.info.resolution,
                'origin': {
                    'x': msg.info.origin.position.x,
                    'y': msg.info.origin.position.y,
                    'z': msg.info.origin.position.z
                }
            }

    def get_current_map(self):
        """
        Retrieve the latest map data and metadata.
        Returns:
            tuple: (map_data, map_meta)
        """
        with self.lock:
            return self.map_data, self.map_meta

# Initialize Flask web server
app = Flask(__name__)
map_subscriber_node = None  # Global reference to the ROS2 node

@app.route('/')
def home():
    """
    Serves the main webpage for viewing the map.
    """
    return render_template('index.html')

@app.route('/map')
def get_map():
    """
    API endpoint to retrieve the current map image and metadata.
    Returns:
        JSON: Encoded map image and metadata.
    """
    if map_subscriber_node is None:
        return jsonify({'error': 'ROS2 node is not initialized'}), 500

    map_data, map_meta = map_subscriber_node.get_current_map()

    if map_data is None:
        return jsonify({'error': 'No map data available yet'}), 500

    # Convert map data to an image
    normalized_map = ((map_data + 1) * 127.5).astype(np.uint8)  # Normalize values to 0-255
    img = Image.fromarray(normalized_map)  # Create an image from the data
    buffered = BytesIO()  # Create an in-memory binary stream
    img.save(buffered, format="PNG")  # Save the image as PNG in the stream
    img_str = base64.b64encode(buffered.getvalue()).decode("utf-8")  # Encode image as Base64

    return jsonify({
        'map_image': f'data:image/png;base64,{img_str}',
        'metadata': map_meta
    })

@app.route('/update', methods=['POST'])
def update_map():
    """
    API endpoint to indicate that the map will update automatically.
    """
    return jsonify({'status': 'Map will update as changes occur in ROS2 topic'})

def run_web_server():
    """
    Starts the Flask web server on a separate thread.
    """
    app.run(host='0.0.0.0', port=5000, debug=False)

def main():
    """
    Main entry point for the script. Initializes the ROS2 node and starts the Flask web server.
    """
    global map_subscriber_node

    rclpy.init()  # Initialize ROS2
    map_subscriber_node = MapSubscriber()  # Create the ROS2 node

    # Start the Flask web server in a separate thread
    web_server_thread = threading.Thread(target=run_web_server)
    web_server_thread.daemon = True  # Ensure the thread closes when the main program exits
    web_server_thread.start()

    try:
        while rclpy.ok():
            rclpy.spin_once(map_subscriber_node)  # Process ROS2 events
    except KeyboardInterrupt:
        pass
    finally:
        map_subscriber_node.destroy_node()  # Destroy the ROS2 node
        rclpy.shutdown()  # Shutdown ROS2

if __name__ == '__main__':
    main()
