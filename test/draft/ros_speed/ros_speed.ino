 #include <ros.h>
 #include <geometry_msgs/Twist.h>
 #include <ros/time.h>


ros::NodeHandle  nh;

#define motorDir 6
#define motorpwm 5
#define encoderPinA 2
#define encoderPinB 3
#define LOOPTIME 10

//variables de ROs
float vx=0;
float w=0;

//  Encoder
volatile long  encoderValue = 0;
int counts = 8400;  // Encoder Pulse per revolution.


//Define Variables we'll be connecting to
double input = 0, output = 0, setpoint = 0;
double integral = 0, previous_error = 0;
double dt = 0.001;  // Sampling time in seconds
unsigned long previousTime=0;

//Specify the links and initial tuning parameters
double kp = 200 , ki = 20 , kd = 0.3;             // modify for optimal performance




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


  // Set initial values
  input = 0;
  output = 0;
//  setpoint = 4; //velocity rad/seg 
  //maximo sin carga 7.9 rad/seg or 76rpm. 

}

void loop() {
  unsigned long currentTime = millis();
  double elapsedTime = (double)(currentTime - previousTime) / 1000.0;  // Convert to seconds
  
  //PID CONTROLLER

  setpoint=10*vx+2.1*w; //kinematics
  
  if (elapsedTime >= dt) {
    
    previousTime = currentTime;
    
    input = (encoderValue / (double)counts) * 2 * PI * 60; //rad/seg del eje principal  
    
    encoderValue=0; //velocidad
  } 

  double error = setpoint - input;
  integral += error * elapsedTime;
  
  double derivative = (error - previous_error) / elapsedTime;


  output = kp * error + ki * integral + kd * derivative;
  previous_error = error;
  
  // Apply saturation limits
  if (output > 255) {
    output = 255;
  } else if (output < -255) {
    output = -255;
  }

  pwmOut(output);
  nh.spinOnce();
  delay(1);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// ************** encoders interrupts **************

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
  digitalWrite(motorDir, HIGH); 
  
}
void reverse () {
  digitalWrite(motorDir, LOW); 
  
}


void pwmOut(int out) {
  if (out > 0) {  // if output > 0, move motor in forward direction
    analogWrite(motorpwm, out);
    forward();
  }
  else {  // if output < 0, move motor in reverse direction
    analogWrite(motorpwm, abs(out));
    reverse();
  }
}
