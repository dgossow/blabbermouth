blabberface
===========

Adds a chatty, doodly face to your robot.

Requirements on Ubuntu: 
- ROS
- libpulse-dev
- festival-dev
- festvox-us1
- libqt4-dev

Launch the face app:

`roslaunch blabberface blabberface.launch`

Let it speak:

`rostopic pub /robot_face/text_out std_msgs/String "Hello world!"`
