To detect AR tags, print the robot’s pose, the tag data, and calculate the distance between the robot and the tag in ROS1, you’ll need to use the ar_track_alvar package, which is specifically for AR tag detection in ROS. We’ll create a Python script that subscribes to the detected AR tag poses and computes the distance between the robot and each tag.

Step 1: Install Dependencies

Ensure that you have ar_track_alvar installed. If it’s not already installed, run:

sudo apt-get install ros-noetic-ar-track-alvar

Step 2: Create a ROS Package

	1.	Navigate to your ROS workspace:

cd ~/catkin_ws/src


	2.	Create a new package:

catkin_create_pkg ar_tag_detection rospy std_msgs geometry_msgs tf


	3.	Navigate to the package:

cd ar_tag_detection



Step 3: Write the Python Script

Create a new file for the script in the scripts directory:

mkdir scripts
touch scripts/ar_tag_detector.py
chmod +x scripts/ar_tag_detector.py

Open ar_tag_detector.py and add the following code:

#!/usr/bin/env python

import rospy
import math
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose, PoseStamped

def ar_tag_callback(msg):
    if len(msg.markers) == 0:
        rospy.loginfo("No AR tags detected.")
        return

    for marker in msg.markers:
        # AR tag data
        tag_id = marker.id
        tag_pose = marker.pose.pose

        # Robot pose with respect to the tag
        robot_pose = PoseStamped()
        robot_pose.pose = tag_pose

        # Print tag data
        rospy.loginfo(f"Tag ID: {tag_id}")
        rospy.loginfo(f"Tag Pose: {tag_pose}")

        # Distance between robot and tag
        distance = math.sqrt(
            tag_pose.position.x**2 +
            tag_pose.position.y**2 +
            tag_pose.position.z**2
        )
        rospy.loginfo(f"Distance to Tag {tag_id}: {distance:.2f} meters")

def main():
    rospy.init_node('ar_tag_detector', anonymous=True)

    # Subscribe to the AR tag topic
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, ar_tag_callback)

    rospy.loginfo("AR Tag Detection Node Started")
    rospy.spin()

if __name__ == "__main__":
    main()

This script does the following:
	1.	Subscribes to the /ar_pose_marker topic, which ar_track_alvar uses to publish detected AR tag information.
	2.	Iterates over detected tags, printing their ID, pose, and calculating the distance between the robot and the tag.
	3.	Logs relevant information to the console.

Step 4: Configure CMakeLists.txt

In your package’s CMakeLists.txt, make sure the script is installed correctly by adding this line at the end of the file:

catkin_install_python(PROGRAMS scripts/ar_tag_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

Step 5: Build the Package

	1.	Navigate to your workspace:

cd ~/catkin_ws


	2.	Build the workspace:

catkin_make


	3.	Source the workspace:

source devel/setup.bash



Step 6: Run the AR Tag Detector

To run the AR tag detector, you’ll need both the ar_track_alvar node and your script running. Follow these steps:
	1.	Start a ROS core (if it’s not already running):

roscore


	2.	Run the AR tag tracking node:

roslaunch ar_track_alvar pr2_indiv.launch

	Adjust the launch file as necessary depending on your AR tag configuration.

	3.	Run your AR tag detector:

rosrun ar_tag_detection ar_tag_detector.py



You should now see the AR tag information in the console, including each tag’s ID, pose, and distance to the robot. This script will continuously log data for each detected AR tag as the robot moves around the environment.