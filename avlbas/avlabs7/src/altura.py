#!/usr/bin/env python3
import rospy
from hector_uav_msgs.msg import Altimeter
from std_msgs.msg import Float64 

class MessagePrinter:
        def __init__(self):
            self.first_values = []
            self.current_value = 0.0
            self.first_value=0.0
            self.subscriber = rospy.Subscriber("altimeter", Altimeter, self.callback)
            self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
            self.publisher = rospy.Publisher('altitude',Float64,queue_size=10)

        def callback(self, data):
            if len(self.first_values) < 20:
                self.first_values.append(data.altitude)
            else:
                self.first_value = sum(self.first_values) / len(self.first_values)
                self.current_value = data.altitude - self.first_value

        def timer_callback(self, event):
            if self.first_value != 0.0:
                self.publisher.publish(self.current_value)
                rospy.loginfo("Altitude: %s", self.current_value)

def main():
    rospy.init_node('altitude_reader', anonymous=True)
    mp = MessagePrinter()
    rospy.spin()

if __name__ == '__main__':
    main()