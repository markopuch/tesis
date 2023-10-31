# tesis
development of a beach clenaer autonomous mobile robot
you need to locate your files in the correct folder. 

you need to change the serial_node.py file to run multiple arduinos: 
https://answers.ros.org/question/12684/using-multiple-arduinos-running-multiple-nodes/

rospy.init_node("serial_node", anonymous=True) 
rospy.loginfo("ROS Serial Python Node {}".format(rospy.get_caller_id()))
    
on raspberry pi : /opt/ros/{DISTRO}/lib/rosserial_python/serial_node.py


use this comand to try in one arduino nano:
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0

----------------------------------------------
FOLLOW THIS TUTORIAL FOR BNO005
https://automaticaddison.com/visualize-imu-data-using-the-bno055-ros-and-jetson-nano/

---------------------------------------------
FOLLOW THIS TUTORIAL http://wiki.ros.org/nmea_navsat_driver
FOR KINETIC

git clone -b kinetic-devel ....

you have to copy the folder "launch", "scripts" and "src" from the master branch and paste it on the kinetic branch

then, you have to copy all the files in scripts and paste in src. After, delete the folder script (which is empty).
Finally, change the the first line in all the files you have copied to: 
#! /usr/bin/env python

Also, don't forget to update your launch file. 

asdasdsadsa
---------------------------
follow this tutorial http://wiki.ros.org/usb_cam

git clone https://github.com/ros-drivers/usb_cam.git
roslaunch usb_cam usb_cam.launch
rosrun image_view image_view image:=/usb_cam/image_raw
