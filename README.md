# simple particle filter on ROS
1. launch gazebo:
roslaunch bot_sim gazebo_bot.launch

2. launch map server
roslaunch bot_sim read_map.launch

3. launch rviz
roslaunch bot_sim gazebo_bot_rviz.launch

4. run particle filter
rosrun bot_sim MCL.py

5. use keyboard to control motion
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
