# Python Applications in Robotics Final Project

This project enabled one turtlebot waffle robot to autonomously navigate to different goals while a second turtlebot waffle robot followed it around. Both utilized LIDAR to prevent collisions. The final product was simulated in Gazebo and utilized Rviz for the camera feeds. ROS was used to allow for the autonomous operation of the robots.

My contribution to this project was the leader pathfinding, which can be found in /final_project_group3/src/leader.py

# Simulation Operation Instructions:
Copy `final_project_group3` into a catkin workspace. 
Run `multiple_robots.launch` `navigation.launch` and `launch.launch` in that order
