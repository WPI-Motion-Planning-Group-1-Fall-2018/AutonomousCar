# AutonomousCar
Autonomous vehicle motion planning implementation/simulation


modified https://github.com/osrf/car_demo with motion planning implementation



Required Dependencies:

gazebo 8 and ros kinetic

uninstall gazebo 7:
sudo apt-remove gazebo7* ros-kinetic-gazebo*
sudo apt-get install ros-kinetic-gazebo8*


Usage:

standard compile with catkin_make in catkin_ws

source devel/setup.bash

roslaunch prius_sim full_sim.launch
