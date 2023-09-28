# tesis
development of a beach clenaer autonomous mobile robot
you need to locate your files in the correct folder. 

you need to change the serial_node.py file to run multiple arduinos:
rospy.init_node("serial_node", anonymous=True) 
    rospy.loginfo("ROS Serial Python Node {}".format(rospy.get_caller_id()))
    

the arduino code "ros_speed2" is the file that works with ros.

use this comand:
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB3

