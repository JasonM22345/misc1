import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State, GlobalPositionTarget
import time

class GPSPathPlanner(Node):
    def __init__(self):
        super().__init__('gps_path_planner')

        # GPS Subscriber
        self.create_subscription(NavSatFix, '/ublox_gps/fix', self.gps_callback, 10)
        
        # PX4 State Subscriber
        self.create_subscription(State, '/mavros/state', self.state_callback, 10)

        # PX4 Global Position Publisher
        self.pos_pub = self.create_publisher(GlobalPositionTarget, '/mavros/setpoint_position/global', 10)
        
        # MAVROS Services
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Waypoints (Lat, Lon, Alt)
        self.waypoints = [
            (37.7749, -122.4194, 10.0),  # Example GPS coordinates
            (37.7755, -122.4200, 10.0),
            (37.7760, -122.4210, 10.0)
        ]

        self.current_gps = None
        self.current_state = None

        self.timer = self.create_timer(1.0, self.control_loop)

    def gps_callback(self, msg):
        """ Updates current GPS location. """
        self.current_gps = (msg.latitude, msg.longitude, msg.altitude)

    def state_callback(self, msg):
        """ Updates current PX4 state. """
        self.current_state = msg

    def set_mode(self, mode):
        """ Sets PX4 flight mode. """
        while not self.mode_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("Waiting for mode service...")
        request = SetMode.Request()
        request.custom_mode = mode
        future = self.mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def arm_drone(self):
        """ Arms the PX4. """
        while not self.arm_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("Waiting for arm service...")
        request = CommandBool.Request()
        request.value = True
        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def publish_waypoint(self, lat, lon, alt):
        """ Publishes a waypoint to PX4. """
        msg = GlobalPositionTarget()
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pos_pub.publish(msg)

    def control_loop(self):
        """ Main control loop for navigating waypoints. """
        if not self.current_gps or not self.current_state:
            self.get_logger().info("Waiting for GPS and PX4 state data...")
            return

        # Set PX4 to GUIDED Mode
        if self.current_state.mode != "OFFBOARD":
            self.set_mode("OFFBOARD")
            self.get_logger().info("Setting mode to OFFBOARD")

        # Arm the drone
        if not self.current_state.armed:
            self.arm_drone()
            self.get_logger().info("Arming drone...")

        # Follow waypoints
        for wp in self.waypoints:
            self.get_logger().info(f"Navigating to: {wp}")
            self.publish_waypoint(*wp)
            time.sleep(5)  # Wait before sending next waypoint

        self.get_logger().info("Path completed.")

def main():
    rclpy.init()
    node = GPSPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
