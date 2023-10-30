#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

ros::NodeHandle  nh;

#define encoderPinA 2
#define encoderPinB 3
#define motorDir 10
#define motorpwm 9
#define LOOPTIME 10


int counts = 8400;  // Encoder Pulse per revolution.
volatile long  encoderValue = 0;


unsigned long previousMillis = 0;
long interval = 100; 
float input;
float setpoint;

//PID VARIABLES
float output, cv1, error, error1, error2;
float Kp=2, Kd=0.00, Ki=50, Tm=0.1;


//variables de ROs
float vx=0;
float w=0;

//ROS CALLBACK FUNCTION
void cmd_vel_cb( const geometry_msgs::Twist& twist){
  vx = twist.linear.x;
  w = twist.angular.z;
} 


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb );

void setup() {
   nh.initNode();
   nh.subscribe(sub);

  pinMode (encoderPinA, INPUT_PULLUP);
  pinMode (encoderPinB, INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt (encoderPinA), readEncoderA, CHANGE);
  attachInterrupt (digitalPinToInterrupt (encoderPinB), readEncoderB, CHANGE);
  
  pinMode(motorDir, OUTPUT);
  pinMode(motorpwm, OUTPUT);

  TCCR1B = TCCR1B & 0b11111000 | 1;

}

void loop(){

  unsigned long currentMillis = millis();

  //MOTOR VELOCITY
  if((currentMillis - previousMillis)>= interval){
    previousMillis = currentMillis;

    input = 10* encoderValue * (2*PI/counts) ; //rad/seg
    encoderValue = 0;
  }

  //
  setpoint= 10*vx+5.985*w; //right//max 7.8 rad/s
  error = setpoint - input;


  //PID CONTROLLER
  output = cv1 + (Kp + Kd/Tm)*error + (-Kp + Ki*Tm - 2*Kd/Tm)*error1+ (Kd/Tm)*error2 ;
  cv1 = output;
  error2 = error1;
  error1 = error;

  //SATURATION
  if(output > 255.0){
    output = 255.0;
  }else if (output < -255.0) {
    output = -255.0;
  }

  //OUTPUT
  pwmOut(output);

  nh.spinOnce();
  delay(100);

}

void readEncoderA()
{
  if (digitalRead(encoderPinA) != digitalRead(encoderPinB))
  {
    encoderValue++;
  }
  else
  {
    encoderValue--;
  }
}

void readEncoderB()
{
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB))
  {
    encoderValue++;
  }
  else
  {
    encoderValue--;
  }
}

void forward () {
  digitalWrite(motorDir, LOW); 
  
}
void reverse () {
  digitalWrite(motorDir, HIGH); 
  
}


void pwmOut(float out) {
  if (out > 0) {  // if output > 0, move motor in forward direction
    analogWrite(motorpwm, out);
    forward();
  }
  else {  // if output < 0, move motor in reverse direction
    analogWrite(motorpwm, abs(out));
    reverse();
  }
}
