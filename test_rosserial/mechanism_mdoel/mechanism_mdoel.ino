#define motorDir1 6
#define motorDir2 4
#define motorpwm 5
#define encoderPinA 2
#define encoderPinB 3

//  Encoder
volatile long  encoderValue = 0;
unsigned long previousMillis = 0;

int counts = 9600;  // Encoder Pulse per revolution.

//Define Variables we'll be connecting to
float input;
long dt = 100;  // Sampling time in 

//Specify the links and initial tuning parameters
float output=0;


void setup() {
  Serial.begin(9600);

  pinMode (encoderPinA, INPUT_PULLUP);
  pinMode (encoderPinB, INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt (encoderPinA), readEncoderA, CHANGE);
  attachInterrupt (digitalPinToInterrupt (encoderPinB), readEncoderB, CHANGE);
  
  pinMode(motorDir1, OUTPUT);
  pinMode(motorDir2, OUTPUT);
  pinMode(motorpwm, OUTPUT);

  TCCR1B = TCCR1B & 0b11111000 | 1;

  
}

void loop() {
  unsigned long currentMillis = millis();
  float Time = millis()/1000.0;
  if((currentMillis - previousMillis)>= dt) {
    previousMillis = currentMillis;
    input = 10*encoderValue *( 60 / (double)counts) ; //rpm del eje principal  
    encoderValue=0; //velocidad
  } 

  
  pwmOut(output);
  //float volt=(output/256)*5;
  
  Serial.print(Time);//time
  Serial.print(",");
  Serial.print(output); // volt
  Serial.print(",");
  Serial.println(input); // speed
//  Serial.print(",");
//  Serial.println(volt); //pwm

  if(Time>5){
   output=255;
  }
//
//  if(Time>10){
//   output=70;
//  }
//
//  if(Time>30){
//   output=100;
//  }
//
//  if(Time>50){
//   output=255;
//  }
//
//  if(Time>70){
//   output=-50;
//  }
//
//
//  if(Time>80){
//   output=-65;
//  }
//
//
//  if(Time>90){
//   output=-155;
//  }
//
//
//  if(Time>120){
//   output=-255;
//  }

 
  delay(dt);
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
