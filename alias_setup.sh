#!/bin/bash
# --- ROS2 shell setup ---

source /opt/ros/jazzy/setup.bash   # change 'humble' to your distro if different

# --- base navigation / sourcing ---
alias rd='cd ~/ros_ws'                          # go to workspace root
alias sr='source ~/ros_ws/install/setup.bash'   # source workspace overlay

# --- build aliases ---
alias cb='rd && colcon build && sr'                              # build whole workspace
alias cbs='rd && colcon build --symlink-install && sr'            # build whole workspace, symlinked
alias cbp='rd && colcon build --packages-select'                 # build one package (usage: cbp my_demo_pkg)
alias cbsp='rd && colcon build --symlink-install --packages-select'  # same, symlinked

# --- dependency / introspection aliases ---
alias ri='rd && rosdep install --from-paths src --ignore-src -y -r'  # install missing deps
alias rr='ros2 run'         # usage: rr my_demo_pkg talker
alias rtl='ros2 topic list'
alias rnl='ros2 node list'
alias rte='ros2 topic echo' # usage: rte /demo_topic

# To make 'rqt_graph' work
unset GTK_PATH