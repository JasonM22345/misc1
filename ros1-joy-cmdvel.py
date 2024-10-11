#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TTBJoyController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('ttb_joy', anonymous=True)

        # Publisher for /cmd_vel topic
        self.publisher = rospy.Publisher('/TTB02/cmd_vel', Twist, queue_size=10)

        # Subscriber for /joy topic
        self.subscriber = rospy.Subscriber('/TTB02/joy', Joy, self.joy_callback)

        # Controller loop at 10Hz
        self.rate = rospy.Rate(10)  # 10Hz
        self.lin_x = 0.0
        self.ang_z = 0.0

    def joy_callback(self, msg):
        # Process the Joy message
        joy_axes = msg.axes
        joy_buttons = msg.buttons

        # Example: linear velocity controlled by buttons, angular velocity by axes
        if len(joy_buttons) >= 8 and len(joy_axes) >= 1:
            # Button 7 (increase speed), Button 6 (decrease speed)
            self.lin_x = float(joy_buttons[7] - joy_buttons[6])

            # Axis 0 controls angular velocity (left/right stick)
            self.ang_z = joy_axes[0]

    def run(self):
        while not rospy.is_shutdown():
            # Create a Twist message for robot control
            twist = Twist()
            twist.linear.x = self.lin_x
            twist.angular.z = self.ang_z

            # Publish the Twist message
            self.publisher.publish(twist)

            # Sleep to maintain loop rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = TTBJoyController()
        controller.run()
    except rospy.ROSInterruptException:
        pass