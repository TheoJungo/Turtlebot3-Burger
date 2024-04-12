# Turtlebot3-Burger
School project with a robot Tutlebot3-Burger. Simulation + Real Tests.
![image](https://github.com/gwendal6468/Turtlebot3-Burger/assets/19518431/f37b44f3-acb5-40fe-aa41-ee6908c0f2e7)

For this project we used ROS2 Humble and a turtlebot3-Burger.
The robot need to be able to do a follow me, then go back to current position. 
to navigate in the room we have done a scan of the floor. 

Here ise a picture of the scan of the floor using : $ ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
![image](https://github.com/gwendal6468/Turtlebot3-Burger/assets/19518431/d3ca2a50-ce45-4abf-bce7-8cfaed27f97d)

How to use: 
1- opend the robot in SSH using your phone and connect to the raspi (login: pi password: raspberry)

2- run the bringup command on the rpi using ssh: $ ros2 launch turtlebot3_manipulation_bringup gazebo.launch.py

3- run on your computer the map RVIZ2 using this command : $ ros2 launch turtlebot3_manipulation_navigation2 navigation2.launch.py map_yaml_file:=$HOME/map.yaml

4- Setup a 2D pose estimate on the map opened with RVIZ2 and wait the lidar to estimate the position of the robot (you can use keyboard movement to move the robot to have a better accuracy)

5- Run the "challenge_navigation.py"


