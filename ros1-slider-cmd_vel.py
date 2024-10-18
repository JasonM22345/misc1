#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from python_qt_binding.QtWidgets import QWidget, QSlider, QVBoxLayout, QLabel
from python_qt_binding.QtCore import Qt


class VelocityControlWidget(QWidget):
    def __init__(self):
        super(VelocityControlWidget, self).__init__()

        # ROS publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Linear Velocity Slider
        self.linear_slider = QSlider(Qt.Horizontal)
        self.linear_slider.setMinimum(-100)  # Minimum linear velocity
        self.linear_slider.setMaximum(100)   # Maximum linear velocity
        self.linear_slider.setValue(0)       # Default value
        self.linear_slider.valueChanged.connect(self.update_velocities)

        self.linear_label = QLabel('Linear Velocity: 0.0')

        # Angular Velocity Slider
        self.angular_slider = QSlider(Qt.Horizontal)
        self.angular_slider.setMinimum(-100)  # Minimum angular velocity
        self.angular_slider.setMaximum(100)   # Maximum angular velocity
        self.angular_slider.setValue(0)       # Default value
        self.angular_slider.valueChanged.connect(self.update_velocities)

        self.angular_label = QLabel('Angular Velocity: 0.0')

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.linear_label)
        layout.addWidget(self.linear_slider)
        layout.addWidget(self.angular_label)
        layout.addWidget(self.angular_slider)

        self.setLayout(layout)

        # Initial twist message
        self.twist = Twist()

    def update_velocities(self):
        # Get the slider values
        linear_velocity = self.linear_slider.value() / 100.0  # Scale to a reasonable range
        angular_velocity = self.angular_slider.value() / 100.0

        # Update labels
        self.linear_label.setText(f'Linear Velocity: {linear_velocity:.2f}')
        self.angular_label.setText(f'Angular Velocity: {angular_velocity:.2f}')

        # Update the Twist message
        self.twist.linear.x = linear_velocity
        self.twist.angular.z = angular_velocity

        # Publish the Twist message
        self.cmd_vel_pub.publish(self.twist)


if __name__ == '__main__':
    rospy.init_node('velocity_control', anonymous=True)

    app = QWidget()
    velocity_control_widget = VelocityControlWidget()
    velocity_control_widget.show()

    rospy.spin()  # Keep the node running
