#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;
//variables de ROs
float vx=0;
float w=0;
float input=0;

//ROS CALLBACK FUNCTION
void cmd_vel_cb( const geometry_msgs::Twist& twist){
  vx = twist.linear.x;
  w = twist.angular.z;
} 
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb );

geometry_msgs::Twist cmd_msg;
ros::Publisher cmd_pub("cmd_ar", &cmd_msg);

std_msgs::Float32 in_msg;
ros::Publisher input_pub("input_ar", &in_msg);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(cmd_pub); 
  nh.advertise(input_pub);
}

void loop(){

  input= 10*vx-5.985*w; //left//max 7.8 rad/s

  cmd_msg.linear.x=vx;
  cmd_msg.angular.z=w;
  in_msg.data = input;


  cmd_pub.publish( &cmd_msg );
  input_pub.publish( &in_msg );

  nh.spinOnce();
  delay(100);

}
