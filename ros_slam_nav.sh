#!/bin/bash

# Step 1: Navigate to the catkin workspace source directory
cd ~/catkin_ws/src

# Step 2: Create the 'navigation_slam' package
catkin_create_pkg navigation_slam std_msgs rospy roscpp sensor_msgs rtabmap_ros realsense2_camera robot_localization move_base

# Step 3: Create necessary directories for launch and config files
cd navigation_slam
mkdir -p launch config

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
  
  <!-- Launch Realsense Cameras -->
  <include file="$(find realsense2_camera)/launch/rs_d400_and_t265.launch">
    <arg name="device_type_camera1" value="$(arg device_type_camera1)"/>
    <arg name="device_type_camera2" value="$(arg device_type_camera2)"/>
    <arg name="serial_no_camera1" value="$(arg serial_no_camera1)"/>
    <arg name="serial_no_camera2" value="$(arg serial_no_camera2)"/>
    <arg name="camera1" value="$(arg camera1)"/>
    <arg name="camera2" value="$(arg camera2)"/>
    <arg name="clip_distance" value="$(arg clip_distance)"/>
  </include>
  
  <!-- ROS Serial Node for Arduino Communication -->
  <node pkg="rosserial_python" type="serial_node.py" name="serial_node">
    <param name="port" value="/dev/ttyACM0"/>
  </node>

  <!-- RTABMap Configuration -->
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

  <!-- UKF Localization -->
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

# Step 5: Create configuration files (costmap_common_params.yaml, etc.)
# Costmap Common Parameters (costmap_common_params.yaml)
cat > config/costmap_common_params.yaml <<EOL
obstacle_range: .15
raytrace_range: 3.0
footprint: [[-0.14,  -0.14], [-0.14, 0.14], [0.14, 0.14], [0.14, -0.14]]
inflation_radius: 0.2
observation_sources: point_cloud_sensor
point_cloud_sensor: {sensor_frame: d400_link, data_type: PointCloud2, topic: /d400/depth/color/points, marking: true, clearing: true}
EOL

# Global Costmap Parameters (global_costmap_params.yaml)
cat > config/global_costmap_params.yaml <<EOL
global_costmap:
  global_frame: map
  robot_base_frame: t265_link
  update_frequency: 5.0
  static_map: true
  track_unknown_space: true
  rolling_window: false
  map_topic: rtabmap/proj_map
EOL

# Local Costmap Parameters (local_costmap_params.yaml)
cat > config/local_costmap_params.yaml <<EOL
local_costmap:
  global_frame: t265_odom_frame
  robot_base_frame: t265_link
  update_frequency: 5.0
  publish_frequency: 2.0
  static_map: false
  rolling_window: true
  width: 4.0
  height: 4.0
  resolution: 0.05
  origin_x: 0
  origin_y: 0
  plugins:
    - {name: obstacle_layer, type: "costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "costmap_2d::InflationLayer"}
EOL

# Base Planner Parameters (base_local_planner_params.yaml)
cat > config/base_local_planner_params.yaml <<EOL
TrajectoryPlannerROS:
  max_vel_x: 0.6
  min_vel_x: 0.25
  max_vel_theta: .5
  min_in_place_vel_theta: 0.4
  acc_lim_theta: 2.5
  acc_lim_x: 2.5
  acc_lim_y: 2.5
  holonomic_robot: false
  yaw_goal_tolerance: 0.25
  xy_goal_tolerance: 0.1
  latch_xy_goal_tolerance: true
  sim_time: 1.5
  sim_granularity: 0.025
  vx_samples: 12
  vtheta_samples: 20
  angular_sim_granularity: 0.05
  meter_scoring: true
  publish_cost_grid: true
  pdist_scale: 0.7
  gdist_scale: 0.8
  occdist_scale: 0.01
  publish_cost_grid_pc: false
EOL

# Step 6: Return to the workspace root
cd ~/catkin_ws

# Instructions for the user
echo "Launch files and configuration files are set up. Now, please run 'catkin_make' to build the workspace."

# End of script
