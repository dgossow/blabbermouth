blabberface
===========

Adds a simple animated, talking head to your robot.

Requirements on Ubuntu: 
- ROS Fuerte (http://www.ros.org)
- libpulse-dev
- festival-dev
- festvox-us1
- libqt4-dev

How-To
------

Build

`cd blabberface`
`make`

Launch the face app:

`roslaunch blabberface blabberface.launch`

Let it speak:

`rostopic pub -1 /robot_face/text_out std_msgs/String 'Hello world!'`
