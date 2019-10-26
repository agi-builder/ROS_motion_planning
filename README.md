# ROS motion planning
This is the repository for robotics course projects.

For more infomantion please go to https://xiangzhuo-ding.github.io/ROS_Projects.html

## Requirements:
1. Environment: Ubuntu 16.04
2. ROS Kinetic
3. xterm
4. ```sudo apt-get install ros-kinetic-moveit```
    ```sudo apt-get update```
    ```sudo apt-get install python-pip```
    ```sudo -H pip install pyassimp --upgrade```

## Set up the workspace:
1. Go to each project.
2. Run ```catkin_make```

## Let's run it!
1. Run ```source devel/setup.bash``` for each terminal
2. Run the simulator 
    ```roslaunch motion_planning kuka_lwr.launch```
    or
    ```roslaunch motion_planning ur5.launch```
3. Now you can start motion planning by running
    ```rosrun run mp.py```
