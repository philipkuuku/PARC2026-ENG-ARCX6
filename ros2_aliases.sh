#!/bin/bash
# --- ROS2 shell setup ---

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS2_ALIAS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${ROS2_WS:-${ROS2_ALIAS_DIR}/ros2_ws}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

# --- base navigation / sourcing ---
alias rd='cd "$ROS2_WS"'
alias sr='source "$ROS2_WS/install/setup.bash"'

# --- build aliases ---
alias cb='rd && colcon build && sr'
alias cbs='rd && colcon build --symlink-install && sr'
alias cbp='rd && colcon build --packages-select'
alias cbsp='rd && colcon build --symlink-install --packages-select'

# --- dependency / introspection aliases ---
alias ri='rd && rosdep install --from-paths src --ignore-src -y -r'
alias rr='ros2 run'
alias rtl='ros2 topic list'
alias rnl='ros2 node list'
alias rte='ros2 topic echo'

# To make 'rqt_graph' work
unset GTK_PATH