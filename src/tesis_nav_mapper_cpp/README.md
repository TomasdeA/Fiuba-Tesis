# Tesis Nav Mapper

## Description
The porpouse of this module is to directly map the depth measurement to a matrix (default in 5x10) and publish it in a topic.

## Running

Terminal 1
1. source /opt/ros/humble/setup.bash
2. ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true align_depth:=true
Note: This will run the realsense ros node that publishes the necesary topics.(camera/camera/depth/image_rect_raw)

Terminal 2
1. source /opt/ros/humble/setup.bash
2. source /workspaces/tesis_nav_assistant_ws/install/setup.bash
3. colcon build
4. ros2 run tesis_nav_mapper_cpp depth_to_matrix
