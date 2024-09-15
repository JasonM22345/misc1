import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Range  # For IR sensor
import random


class TTBObstacleAvoider(Node):

    def __init__(self):
        super().__init__('ttb_obstacle_avoider')

        # Publisher for Twist msg to control movement
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for Joy input
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscriber_ = self.create_subscription(Joy, '/joy', self.joy_callback, qos_profile)

        # Subscriber for IR sensor input
        self.ir_subscriber_ = self.create_subscription(Range, '/ir_sensor', self.ir_callback, qos_profile)

        self.timer = self.create_timer(0.1, self.controller_callback)

        self.lin_x = 0.2  # Default forward speed
        self.ang_z = 0.0
        self.obstacle_detected = False

    def controller_callback(self):
        # Check for obstacles and decide if we need to turn
        msg = Twist()
        if self.obstacle_detected:
            # Turn randomly if an obstacle is detected
            self.ang_z = random.uniform(0, 2 * 3.1415)  # Turn randomly between 0 and 360 degrees
            self.lin_x = 0.0  # Stop moving forward while turning
        else:
            # Move forward in a straight line
            self.lin_x = 0.2
            self.ang_z = 0.0

        self.publisher_.publish(msg)

    def joy_callback(self, msg):
        # Could be used for manual overrides, if needed
        pass

    def ir_callback(self, ir_msg):
        # Check if the obstacle is within 0.1 meters
        distance = ir_msg.range
        if distance < 0.1:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False


def main(args=None):
    rclpy.init(args=args)

    ttb_obstacle_avoider = TTBObstacleAvoider()

    rclpy.spin(ttb_obstacle_avoider)

    ttb_obstacle_avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()