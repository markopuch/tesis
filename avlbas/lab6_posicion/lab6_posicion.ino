#include <ros.h>
#include <std_msgs/Float32.h>
#include <ros/time.h>

ros::NodeHandle  nh;

#define encoderPinA 2
#define encoderPinB 3
#define motorDir1 6
#define motorDir2 4
#define motorpwm 5
#define LOOPTIME 10

int counts = 6533 ;  // Encoder Pulse per revolution.
volatile long  encoderValue = 0;


unsigned long previousMillis = 0;
long interval = 100; 
float input;
float setpoint;
int angle = 360; // Maximum degree of motion.

//PID VARIABLES
float output, cv1, error, error1, error2;
float Kp=1, Kd=0.3, Ki=0.06  , Tm=0.1;


//variables de ROs
float pos=0;


////ROS CALLBACK FUNCTION
void pos_cb( const std_msgs::Float32& msg){
  pos = msg.data;
} 



ros::Subscriber<std_msgs::Float32>  sub("pos", pos_cb );

void setup() {
   nh.initNode();
   nh.subscribe(sub);

//  Serial.begin(9600);
  pinMode (encoderPinA, INPUT_PULLUP);
  pinMode (encoderPinB, INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt (encoderPinA), readEncoderA, CHANGE);
  attachInterrupt (digitalPinToInterrupt (encoderPinB), readEncoderB, CHANGE);
  
  pinMode(motorDir1, OUTPUT);
  pinMode(motorDir2, OUTPUT);
  pinMode(motorpwm, OUTPUT);

  TCCR1B = TCCR1B & 0b11111000 | 1;
  
//  pos=360;
//  setpoint= pos;
//  input=0;
//  output=0;
}

void loop(){

  unsigned long currentMillis = millis();
  
   
  //MOTOR VELOCITY
  if((currentMillis - previousMillis)>= interval){
    

    input = (encoderValue / (double)(counts)) * angle; //angulo grados
    
    setpoint= pos;
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
    
    previousMillis = currentMillis;
    
  }

  //
//  Serial.print(setpoint);
//  Serial.print(",");
//  Serial.println(input);
//  Serial.print(",");
//  Serial.println(output);

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
  digitalWrite(motorDir1, HIGH);
  digitalWrite(motorDir2, LOW); 
  
}
void reverse () {
  digitalWrite(motorDir1, LOW); 
  digitalWrite(motorDir2, HIGH);
  
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
