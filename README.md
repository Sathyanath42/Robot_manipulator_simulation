# Vention_task
A pick and place robot manipulator simulation

*****
SETUP
* System Details: ROS Melodic running on Ubuntu 18.04
* Robot Arm: Panda arm from Franka Emika [panda_arm_demo](https://github.com/ros-planning/panda_moveit_config)
* Libraries used include: [moveit_commander](http://wiki.ros.org/moveit_commander)
* Tutorial: [moveit](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/index.html#)

*****
RUNNING THE CODE
* Setup the catkin_ws and import the necessary packages : [Setting up ROS](https://github.com/Sathyanath42/ROS)
* Install the move-it libraries
~~~
sudo apt install ros-melodic-moveit
~~~
* Clone the panda config into the src folder
~~~
cd ~/ws_moveit/src
git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel
~~~
* Copy the python file into the scripts folder within the source folder
* Run it from there or call the node directly with rosrun
~~~
$roslaunch panda_moveit_config demo.launch
$rosrun <...folder_name...> movit_1.py

~~~
