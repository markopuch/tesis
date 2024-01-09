#!/usr/bin/env python3
import rospy
from hector_uav_msgs.msg import Altimeter
from std_msgs.msg import Float64 

class MessagePrinter:
    def __init__(self):

        self.first_values = []
        self.first_value=0.0
       
        self.filtered_value_preview=0.0
        self.current_value_preview=0.0
        self.filtered_value_preview_1=0.0
        self.current_value_preview_1=0.0
        self.current_value = 0.0
        self.filtered_value=-0.0

        self.first_value = None
        self.subscriber = rospy.Subscriber("altimeter", Altimeter, self.callback)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.publisher = rospy.Publisher('altitude_filter2',Float64,queue_size=10)

    def callback(self, data):
        if len(self.first_values) < 20:
                self.first_values.append(data.altitude)
            
        else:
            self.first_value = sum(self.first_values) / len(self.first_values)
            
            a0=1.95558189
            a1=-0.95654717
            b0=0.00024132
            b1=0.00048264
            b2=0.00024132
            self.current_value = data.altitude - self.first_value
            
            self.filtered_value = self.filtered_value_preview*a0 + self.filtered_value_preview_1*a1 + b0*self.current_value + b1*self.current_value_preview + b2*self.current_value_preview_1
            
            self.filtered_value_preview = self.filtered_value
            self.filtered_value_preview_1 = self.filtered_value_preview
            self.current_value_preview = self.current_value
            self.current_value_preview_1 = self.current_value_preview
            

    def timer_callback(self, event):
        if self.first_value is not None:
            self.publisher.publish(self.filtered_value)
            rospy.loginfo("Altitude: %s", self.filtered_value)

def main():
    rospy.init_node('altitude_reader', anonymous=True)
    mp = MessagePrinter()
    rospy.spin()

if __name__ == '__main__':
    main()