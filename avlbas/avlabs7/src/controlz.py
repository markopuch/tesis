#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Twist
from hector_uav_msgs.msg import Altimeter


class PIDController:
    def __init__(self,kp,ki,kd):
        self.pid = PID(kp,ki,kd)     
        self.subscriber = rospy.Subscriber('alt_escalada',Altimeter,self.callback)
        self.publisher = rospy.Publisher('cmd_vel',Twist,queue_size=10)
        self.setpoint = 2.0
        self.last_time = rospy.get_time()
    
    def callback(self,data):
        current_time = rospy.get_time()
        dt = current_time - self.last_time

        if dt <= 0.0:
            return
        
        control_action = self.pid.update(self.setpoint,data.altitude,dt)
        command = Twist()
        command.linear.z = control_action

        self.publisher.publish(command)
        self.last_time = current_time



class PID:
    def __init__(self,kp,ki,kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
    
    def update(self,setpoint,measured_value,dt):
        error = setpoint - measured_value
        self.integral += error*dt
        derivative = (error - self.prev_error)/dt

        output = self.kp*error + self.ki*self.integral + self.kd*derivative
        self.prev_error = error

        return output

if __name__ == '__main__':
    try:
        rospy.init_node('pid_alt_control')
        kp = 0.1
        ki = 0.01
        kd = 0.05
        pid_node = PIDController(kp,kd,ki)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        