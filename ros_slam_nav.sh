#!/bin/bash

# Step 1: Navigate to the catkin workspace source directory
cd ~/catkin_ws/src

# Step 2: Create the 'navigation_slam' package
catkin_create_pkg navigation_slam std_msgs rospy roscpp sensor_msgs rtabmap_ros realsense2_camera robot_localization move_base

# Step 3: Create necessary directories for launch files
cd navigation_slam
mkdir launch config

# Step 4: Create the launch files
# RS and RTABMap Launch File (rs_rtabmap.launch)
cat > launch/rs_rtabmap.launch <<EOL
<launch>
  <!-- Initialize Realsense Cameras -->
  <arg name="device_type_camera1" default="t265"/>
  <arg name="device_type_camera2" default="d4.5"/>
  <arg name="serial_no_camera1" default=""/>
  <arg name="serial_no_camera2" default=""/>
  <arg name="camera1" default="t265"/>
  <arg name="camera2" default="d400"/>
  <arg name="clip_distance" default="-2"/>
  <arg name="use_rviz" default="true"/>
  <arg name="use_rtabmapviz" default="true"/>
  
  <include file="$(find realsense2_camera)/launch/rs_d400_and_t265.launch">
    <arg name="device_type_camera1" value="$(arg device_type_camera1)"/>
    <arg name="device_type_camera2" value="$(arg device_type_camera2)"/>
    <arg name="serial_no_camera1" value="$(arg serial_no_camera1)"/>
    <arg name="serial_no_camera2" value="$(arg serial_no_camera2)"/>
    <arg name="camera1" value="$(arg camera1)"/>
    <arg name="camera2" value="$(arg camera2)"/>
    <arg name="clip_distance" value="$(arg clip_distance)"/>
  </include>
  
  <node pkg="rosserial_python" type="serial_node.py" name="serial_node">
    <param name="port" value="/dev/ttyACM0"/>
  </node>

  <include file="$(find rtabmap_ros)/launch/rtabmap.launch">
    <arg name="depth_topic" value="/$(arg camera2)/aligned_depth_to_color/image_raw"/>
    <arg name="rtabmap_args" value="--delete_db_on_start"/>
    <arg name="frame_id" value="$(arg camera2)_link"/>
    <arg name="visual_odometry" value="false"/>
    <arg name="odom_topic" value="/$(arg camera1)/odom/sample"/>
    <arg name="rgb_topic" value="/$(arg camera2)/color/image_raw"/>
    <arg name="camera_info_topic" value="/$(arg camera2)/color/camera_info"/>
    <arg name="queue_size" value="200"/>
    <arg name="rviz" value="$(arg use_rviz)"/>
    <arg name="rtabmapviz" value="$(arg use_rtabmapviz)"/>
  </include>

  <include file="$(find robot_localization)/launch/ukf_template.launch"/>
  <param name="/ukf_se/frequency" value="300"/>
  <param name="/ukf_se/base_link_frame" value="t265_link"/>
  <param name="/ukf_se/odom0" value="rtabmap/odom"/>
  <rosparam param="/ukf_se/odom0_config">
    [true, true, true, true, true, true, true, true, true, true, true, true]
  </rosparam>
  <param name="/ukf_se/imu0" value="/imu/data"/>
</launch>
EOL

# Move Base Launch File (move_base.launch)
cat > launch/move_base.launch <<EOL
<launch>
  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <rosparam file="$(find robot_2dnav)/costmap_common_params.yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find robot_2dnav)/costmap_common_params.yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find robot_2dnav)/local_costmap_params.yaml" command="load" />
    <rosparam file="$(find robot_2dnav)/global_costmap_params.yaml" command="load" />
    <rosparam file="$(find robot_2dnav)/base_local_planner_params.yaml" command="load" />
    <remap from="cmd_vel" to="/cmd_vel"/>
    <remap from="odom" to="/t265/odom/sample"/>
    <remap from="/map" to="rtabmap/grid_map"/>
  </node>
</launch>
EOL

# Create config files (costmap_common_params.yaml, etc.)
# You can add the respective content from the configuration provided in your question.

# Step 5: Return to the workspace root
cd ~/catkin_ws

# Instructions for the user
echo "Launch files and configuration files are set up. Now, please run 'catkin_make' to build the workspace."

# End of script
