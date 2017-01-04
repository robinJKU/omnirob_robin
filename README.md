# omnirob_robin
Omnirob @ Institute of Robotics, JKU Linz

omnirob_robin needs ros-gazebo5/6/7 to run the simulation.
Gazebo7 has been tested, read last step for upgrade instructions.

omnirob_robin installation instructions:
Git clone the omnirob_robin stack: https://github.com/robinJKU/omnirob_robin
Git clone the ros_common_robin stack: https://github.com/robinJKU/ros_common_robin
To use slam_kart clone it from here: https://github.com/ros-perception/slam_karto
To install dependencies run (use your own workspace path):
rosdep install --from-paths ~/catkin_ws/src --ignore-src --rosdistro=indigo -y


Optional steps for simulation:
After a clean ros-indigo-desktop-full install do the following:
sudo apt-get remove gazebo2

add the repository:
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable \`lsb_release -cs\` main" > /etc/apt/sources.list.d/gazebo-stable.list'
sudo apt-get update

sudo apt-get install ros-indigo-gazebo7-ros-pkgs
