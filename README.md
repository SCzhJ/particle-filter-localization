# ROS particle filter localization and RRT

## To run RRT*
### 1. launch gazebo
roslaunch bot_sim gazebo_bot.launch
### 2. launch map server
roslaunch bot_sim read_map.launch
### 3. launch rviz
roslaunch bot_sim gazebo_bot_rviz.launch
### 4. launch RRT* animation
rosrun bot_sim RRT_star.py
