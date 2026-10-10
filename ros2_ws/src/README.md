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

**Packages needed are:** (These are ONLY examples for you to follow)

* `ros2_control`: ROS packages including controller interface, controller manager, hardware interface etc.
    * `$ sudo apt-get install ros-jazzy-ros2-control`

* `navigation2` and `nav2_bringup`: Nav2 stack (planner, controller, behaviors, AMCL, map server, collision monitor).
    * `$ sudo apt-get install ros-jazzy-navigation2 ros-jazzy-nav2-bringup`
   
* `opencv`: Open-source computer vision library used for green detection.
    * `$ sudo apt-get install python3-opencv`
      
* `cv_bridge`: Converts ROS image messages to OpenCV images.
    * `$ sudo apt-get install ros-jazzy-cv-bridge`
      
* `message_filters`: Time-synchronizes the RGB image and the point cloud.
    * `$ sudo apt-get install ros-jazzy-message-filters`
      
* `sensor_msgs_py`: Reads 3D points from the organized point cloud.
    * `$ sudo apt-get install ros-jazzy-sensor-msgs-py`
   
* `tf2_ros` and `tf2_geometry_msgs`: Transform the detected target into the map frame.
    * `$ sudo apt-get install ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs`

To install all our ROS 2 package dependencies, run this command from the root of your ros2 workspace
rosdep install --from-paths src --ignore-src -r -y from the root of your ROS 2 workspace after initializing rosdep with ` sudo rosdep init ` (Ensure it completes without errors and outputs a message confirming the creation of /etc/ros/rosdep/sources.list.d) and ` rosdep update `.

## Task

The task given was to autonomously navigate a service robot in a stadium environment. The robot is supposed to percieve the surrounding with its onboard sensor systems, detect obstacles and crowds, and dynamically plan and follow a trajectory. Our first step was to generate an accurate map representation of the environment using the SLAM toolbox. We then used the Nav2 stack to generate costmaps of obstacles on the map and activated autonomous navigation on the robot to be able to navigate to a random goal that we set to it to. We went ahead to implement a node that used OpenCV through the depth camera to identify the green goal and another to confirm detection and send the coordinates as the goal to the navigator. The robot then uses these coordinates to plan a path and dynamically change the path-to-goal while making adjustements to obstacles change. We went ahead to tweak paramaters to optimize speed, obstacle avoidance and travel distance to goal to improve on the speed and performance of the robot.

Write the command required to run your solution. Should be in this format: <br>
1. ` ros2 launch parc_robot_bringup task.launch.py `
2. ` ros2 launch task_solution task_solution.launch.py `

In case visualization of map is required:
1. Set Global Fixed Frame in RViz to /map
2. Add Map and set topic to /map to visualize map
3. Add Map and set topic to /localcostmap/

## Things to Note
1. Pose setup for localization is done automatically in our code to match the origin pose seen in gazebo on spawn

## Challenges Faced

Describe any challenges your team may have faced in solving these tasks. Try to document them in bullets as much as you can.
* The position of the lidar made the robot tyres act as obstacles and were not allowing the robot to move when Nav2 was implemented
* Difficutly in localization of robot in map due to accumulated drift of odom transform.
* 

## Package Overview

- [`parc_robot_bringup`](./parc_robot_bringup/) : Contains config, world, scripts and launch files to bringup the CAYTU SITO-E robot.
- [`parc_robot_description`](./parc_robot_description/) : Contains the URDF description files for the CAYTU SITO-E and launch files for the robot state publisher and description.
- [`arcx6_solution`](./arcx6_solution/) : </br>
Our solution package. Contains the Nav2 parameters and map, the localization and navigation launch files, the `green_detector` node (OpenCV green detection and 3D localization), the `green_nav` node (search and navigate to the target), and `task_solution.launch.py`, which runs everything.

--------------------------------------------------
