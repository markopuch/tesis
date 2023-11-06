#include "pid.h"

void setup(){
  pinMode (encoderPinA, INPUT_PULLUP);
  pinMode (encoderPinB, INPUT_PULLUP);
  Serial.begin(115200);
  attachInterrupt (digitalPinToInterrupt (encoderPinA), readEncoderA, CHANGE);
  attachInterrupt (digitalPinToInterrupt (encoderPinB), readEncoderB, CHANGE);
  
  pinMode(motorDir, OUTPUT);
  pinMode(motorpwm, OUTPUT);
  
  TCCR1B = TCCR1B & 0b11111000 | 1;
}

void loop(){

  
  setpoint= 10*vx-5.985*w; //max 7.8 rad/s
  setpoint=-setpoint;
  control_pid();
  pwmOut(output);
  
  Serial.print(setpoint);
  Serial.print(",");
  Serial.print(output);
  Serial.print(","); 
  Serial.println(pv);

  delay(100);

}
