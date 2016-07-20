# omnirob_robin v1.0
Omnirob @Institute of Robotics, JKU Linz

omnirob_robin needs ros-gazebo5/6/7 to run the simulation.
Gazebo7 has been tested, ros-indigo-ros-control was compiled from source.

You may need to add the repository:
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
sudo apt-get update

After a clean ros-indigo-desktop-full install do the following:
sudo apt-get remove gazebo2
sudo apt-get install ros-indigo-gazebo7-ros-pkgs

Copy the gazebo_ros_control package from github to your workspace:
https://github.com/ros-simulation/gazebo_ros_pkgs/tree/indigo-devel



To install dependencies run:
rosdep install --from-paths WORKSPACE/src --ignore-src --rosdistro=indigo -y
