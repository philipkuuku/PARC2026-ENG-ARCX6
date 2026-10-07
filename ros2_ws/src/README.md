# TEAM 1: PARC Engineers League 

## Introduction

In this section, you should give an overview of the competition task, briefly describe your understanding of precision agriculture with the pros and cons of adoption by farmers (*This should be only 5-6 sentences*).

**Team Country:** Country

**Team Member Names:**

* First and Last Name (Team Leader)
* First and Last Name
* ...

## Dependencies

**Packages needed are:** (These are ONLY examples for you to follow)

* `ros2_control`: ROS packages including controller interface, controller manager, hardware interface etc.

    * `$ sudo apt-get install ros-jazzy-ros2-control`

* `opencv`: Open-source computer vision library.

    * `$ sudo apt-get install python3-opencv`

## Task

Include a brief description of your approach to the solution (*This should be only 5-7 sentences*).

Write the command required to run your solution. Should be in this format: <br>
` ros2 run <your-package-name> task_solution.py `


## Challenges Faced

Describe any challenges your team may have faced in solving these tasks. Try to document them in bullets as much as you can.

## Package Overview

- [`parc_robot_bringup`](./parc_robot_bringup/) : Contains config, world, scripts and launch files to bringup the CAYTU SITO-E robot.
- [`parc_robot_description`](./parc_robot_description/) : Contains the URDF description files for the CAYTU SITO-E and launch files for the robot state publisher and description.

## References
- [Cafe world](https://app.gazebosim.org/OpenRobotics/fuel/models/Cafe)
- [Other Gazebo models](https://app.gazebosim.org/models)
- [realsense_gazebo_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)
- [realsense-ros](https://github.com/realsenseai/realsense-ros)
- [RobotCAD](https://github.com/drfenixion/freecad.robotcad)

--------------------------------------------------