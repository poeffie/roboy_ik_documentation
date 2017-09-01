# Basics
> MoveIt! is state of the art software for mobile manipulation, incorporating the latest advances in motion planning, manipulation, 3D perception, kinematics, control and navigation. It provides an easy-to-use platform for developing advanced robotics applications, evaluating new robot designs and building integrated robotics products for industrial, commercial, R&D and other domains.
> -- <cite><http://moveit.ros.org/>

For our project we are using ROS MoveIt! to calculate the inverse kinematics for Roboy. More specifically we need a joint trajectory, that describes how joint angles change over time. This joint trajectoy will be fed into CASPR / CASPROS to simulate / output motor commands. Therefore it is necessary to make use of MoveIt!'s motion planning functionality.

# Installation
In the current implementation of the code, Ubuntu 16.04.3 LTS (Xenial Xerus) was used, which only works with ROS Kinetic. In this case, installation is done after the following steps (provided that ROS Kinetic is already installed):

1. `sudo apt-get install ros-kinetic-moveit`
2. `source /opt/ros/kinetic/setup.bash`

For further information, go to the [official MoveIt! installation website](http://moveit.ros.org/install/)
