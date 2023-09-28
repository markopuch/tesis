#define encoderPinA 2
#define encoderPinB 3
#define motorDir 6
#define motorpwm 5

float pv;
float setpoint;

volatile long  encoderValue = 0;
unsigned long previousMillis = 0;
long interval = 100; 


float output, cv1, error, error1, error2;
float Kp=2, Kd=0.00, Ki=50, Tm=0.1;
float vx= 0.05;
float w=0.0;


void control_pid()
{
  unsigned long currentMillis = millis();
  if((currentMillis - previousMillis)>= interval){
    previousMillis = currentMillis;
    //    pv = 10* encoderValue * (60.0/8400.0) ; //rpm 
    pv = 10* encoderValue * (2*PI/8400.0) ; //rad/seg
    encoderValue = 0;
  }
  //  setpoint = 60; //rpm setpoint 75 max
  
  error = setpoint - pv;


  output = cv1 + (Kp + Kd/Tm)*error + (-Kp + Ki*Tm - 2*Kd/Tm)*error1+ (Kd/Tm)*error2 ;
  cv1 = output;
  error2 = error1;
  error1 = error;

  if(output > 255.0){
    output = 255.0;
  }else if (output < -255.0) {
    output = -255.0;
  }
  
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
  digitalWrite(motorDir, HIGH); 
  
}
void reverse () {
  digitalWrite(motorDir, LOW); 
  
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
