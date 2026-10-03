# PARC2026-Engineers-League
Pan-African Robotics Competition (PARC) Engineers League 2026 project development.

## Package Overview

- [`parc_robot_bringup`](./parc_robot_bringup/) : Contains config, world, scripts and launch files to bringup the CAYTU SITO-E robot.
- [`parc_robot_description`](./parc_robot_description/) : Contains the URDF description files for the CAYTU SITO-E and launch files for the robot state publisher and description.

- Launch command: 
    ros2 launch parc_robot_bringup task.launch.py
    ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
- Control with teleop: 
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap \
/cmd_vel:=/robot_base_controller/cmd_vel_unstamped
- Map saver: 
    ros2 run nav2_map_server map_saver_cli -f /home/jachin/parc_main/PARC2026-ENG-ARCX6/ros2_ws/src/maps/restaurant

Nav2 mapserver
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=restaurant_save.yaml -p use_sim_time:
=true

ros2 run nav2_util lifecycle_bringup  map_server

ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true


- New teleop control: 
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cmd_vel_key

- Nav2:
    ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  map:=/home/jachin/parc_main/PARC2026-ENG-ARCX6/ros2_ws/src/restaurant_savemap.yaml \
  params_file:=/home/jachin/parc_main/PARC2026-ENG-ARCX6/ros2_ws/src/parc_robot_bringup/config/nav2_params.yaml

## To run nav2
- Launch command: ros2 launch parc_robot_bringup task.launch.py
- Set map in Fixed Frame - Global Options
- Set Map thing and select map as Topic and Transient Local for Durability Policy
- Set Pose Estimate 

## References
- [Cafe world](https://app.gazebosim.org/OpenRobotics/fuel/models/Cafe)
- [Other Gazebo models](https://app.gazebosim.org/models)
- [realsense_gazebo_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)
- [realsense-ros](https://github.com/realsenseai/realsense-ros)
- [RobotCAD](https://github.com/drfenixion/freecad.robotcad)
