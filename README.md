# 4_edbot-ros-pkg
Edbot research robot. 

edbot is powered by ROS-Indigo running on top of 
ubilinux, a debian wheezy based linux disto, on a Intel Edison 
platform. Edbot's differential drive system is built from 2 dc 
gear motors driven by Dimension Engineering's Kanagroo Motion 
Controller plus a Sabertooth 2x12 Motor Driver.

This edbot ros package contains the following nodes:

edbot src/diffdrv.py

edbot robot base differential drive controller: 
subscribes to /cmd_vel. translates linear & angular command velocities 
from ros /cmd_vel topic into motion control command sequences needed by 
kangaroo motion controller. kangaroo mc is interfaces to edison via 
19.2KBs serial link.

edbot src/odometriclocalizer.py

edbot robot differential drive controller:
Based on robot parameters (wheel dia, track width, encCPR) & wheel 
encoder counts, this node computes  and publishes odometry and 
tf (between /odom world frame & /base_link frame)

