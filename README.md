# tesis
development of a beach clenaer autonomous mobile robot
you need to locate your files in the correct folder. 

you need to change the serial_node.py file to run multiple arduinos: 
https://answers.ros.org/question/12684/using-multiple-arduinos-running-multiple-nodes/

rospy.init_node("serial_node", anonymous=True) 
rospy.loginfo("ROS Serial Python Node {}".format(rospy.get_caller_id()))
    



use this comand to try in one arduino nano:
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0

FOLLOW THIS TUTORIAL FOR BNO005
https://automaticaddison.com/visualize-imu-data-using-the-bno055-ros-and-jetson-nano/

FOLLOW THIS TUTORIAL http://wiki.ros.org/nmea_navsat_driver
FOR KINETIC
