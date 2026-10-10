# ARCX6 : PARC Engineers League 

## Introduction

In this task, an autonomous service robot must navigate a crowded stadium using Nav2, detecting people and obstacles, planning safe paths, and adjusting its movement to reach a goal marked by a green circle. As robots become more common in public spaces, they offer benefits such as improved efficiency, safety, and reduced human workload. However, challenges including high costs, job displacement, reliability, and safety risks remain. Autonomous navigation is therefore essential for enabling robots to operate safely and effectively around people.

**Team Country:** Ghana

**Team Member Names:**

* Eyiram Gaze (Team Leader)
* Phillip Kuuku Quartey
* Jachin Kpogli
* Selina-Elsie Winpiini Azare
* Sarfo Mirriam
* Serwaa Abena Agyapong

## Dependencies

**Packages needed are:**

* `ros2_control`: Provides the controller manager, hardware interfaces, and robot control stack used by the robot bringup.
    * `$ sudo apt install ros-jazzy-ros2-control`

* `ros2_controllers`: Standard controllers used by the robot platform.
    * `$ sudo apt install ros-jazzy-ros2-controllers`

* `navigation2` and `nav2_bringup`: Nav2 stack for planning, localization, costmaps, recovery behaviors, and autonomous navigation.
    * `$ sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup`

* `robot_localization`: Fuses odometry and IMU data to estimate the robot pose reliably.
    * `$ sudo apt install ros-jazzy-robot-localization`

* `ros_gz_bridge`: Bridges ROS topics to Gazebo and sim messages used by the robot world.
    * `$ sudo apt install ros-jazzy-ros-gz-bridge`

* `ros_gz_image`: Gazebo image transport bridge for camera topics.
    * `$ sudo apt install ros-jazzy-ros-gz-image`

* `ros_gz_sim`: Gazebo simulation support for the robot and environment.
    * `$ sudo apt install ros-jazzy-ros-gz-sim`

* `gz_ros2_control`: Connects Gazebo to ROS 2 control.
    * `$ sudo apt install ros-jazzy-gz-ros2-control`

* `image_transport_plugins`: Image transport plugins for camera feeds used during bringup and visualization.
    * `$ sudo apt install ros-jazzy-image-transport-plugins`

* `joint_state_publisher` and `robot_state_publisher`: Publish robot state and joint transforms for the URDF model.
    * `$ sudo apt install ros-jazzy-joint-state-publisher ros-jazzy-robot-state-publisher`

* `urdf` and `xacro`: Parse and generate the robot description from URDF/Xacro files.
    * `$ sudo apt install ros-jazzy-urdf ros-jazzy-xacro`

* `rviz2`: Visualizes the robot, map, and costmaps during debugging and setup.
    * `$ sudo apt install ros-jazzy-rviz2`

* `opencv`: Open-source computer vision library used to detect the green goal marker in camera images.
    * `$ sudo apt install python3-opencv`

* `cv_bridge`: Converts ROS image messages to OpenCV images for processing.
    * `$ sudo apt install ros-jazzy-cv-bridge`

* `message_filters`: Synchronizes RGB image and point cloud streams for target localization.
    * `$ sudo apt install ros-jazzy-message-filters`

* `sensor_msgs`: Provides standard sensor message definitions used by image and point cloud topics.
    * `$ sudo apt install ros-jazzy-sensor-msgs`

* `sensor_msgs_py`: Reads 3D points from the organized point cloud.
    * `$ sudo apt install ros-jazzy-sensor-msgs-py`

* `tf2_ros` and `tf2_geometry_msgs`: Transform the detected target and robot frames into the map frame.
    * `$ sudo apt install ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs`

To install all our ROS 2 package dependencies, run these commands from the root of your ROS 2 workspace:

```bash
cd ~/ros2_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
```

## Task

The task given was to autonomously navigate a service robot in a stadium environment. The robot is supposed to percieve the surrounding with its onboard sensor systems, detect obstacles and crowds, and dynamically plan and follow a trajectory. Our first step was to generate an accurate map representation of the environment using the SLAM toolbox. We then used the Nav2 stack to generate costmaps of obstacles on the map and activated autonomous navigation on the robot to be able to navigate to a random goal that we set to it to. We went ahead to implement a node that used OpenCV through the depth camera to identify the green goal and another to confirm detection and send the coordinates as the goal to the navigator. The robot then uses these coordinates to plan a path and dynamically change the path-to-goal while making adjustements to obstacles change. We went ahead to tweak paramaters to optimize speed, obstacle avoidance and travel distance to goal to improve on the speed and performance of the robot.

### Important setup note

> Before running, set the minimum LiDAR range to 0.35. With the default value, the LiDAR detects the robot’s tyres as obstacles and autonomous movement will fail.

The LiDAR model is defined in the Xacro file at:

`parc_robot_description/urdf/rplidar_c1.xacro`

> Change min. value on line 46 to from 0.05 to 0.35

After modifying the robot description or launch configuration, rebuild the workspace and source the environment before running the project:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Commands required to run our solution <br>

1. ` ros2 launch parc_robot_bringup task.launch.py `
2. ` ros2 launch task_solution task_solution.launch.py `

In case visualization of map is required:
1. Set Global Fixed Frame in RViz to /map
2. Add Map and set topic to /map to visualize map
3. Add Map and set topic to `/global_costmap/costmap`, and the colour scheme to `costmap`


## Things to Note
1. Modify the minimum scan range for the lidar from 0.05 to 0.35 (line 46 of rplidar_c1.xacro in the description)
2. Pose setup for localization is done automatically in our code to match the origin pose seen in gazebo on spawn

## Challenges Faced

Describe any challenges your team may have faced in solving these tasks. Try to document them in bullets as much as you can.
* Challenge was unclear at the start as its task description was general. It was hard at the start to develop an approach and plan to follow. Clarity was later made and we were able to narrow down. 
* At the 
* The position of the lidar made the robot tyres act as obstacles and were not allowing the robot to move when Nav2 was implemented
* Visualization rays of lidar in gazebo were appearing outside the lidar of the robot. To fix this: after gazebo launches, and 'Visualize lidar' is activated with the topic set to `/scan`, run this command in a new terminal: </br> 
`gz model -m sitoe_robot -l`

* Difficutly in localization of robot in map due to accumulated drift of odom transform and jittery movement.
* Setting up Nav2 the entire pipeline for Nav2 came with multiple challenges amd errors.
* At first, we were not fully aware that a custom parameter file was necessary, and we discovered that relying on Nav2’s default settings was not ideal before we set up and validated our own configuration.
* Many of the configurations and setup steps caused errors or unexpected output, and finding and fixing each one took a lot of time.
* At one point, setting the initial pose did not work correctly. The robot’s heading was about 90° off from the direction we set, which caused a mismatch between RViz and Gazebo and made navigation unreliable.

## Package Overview

- [`parc_robot_bringup`](./parc_robot_bringup/) : Contains config, world, scripts and launch files to bringup the CAYTU SITO-E robot.
- [`parc_robot_description`](./parc_robot_description/) : Contains the URDF description files for the CAYTU SITO-E and launch files for the robot state publisher and description.
- [`task_solution`](./) : </br>
Our solution package. Contains the Nav2 parameters and map, the localization and navigation launch files, the `green_detector` node (OpenCV green detection and 3D localization), the `green_nav` node (search and navigate to the target), and `task_solution.launch.py`, which runs everything.

--------------------------------------------------