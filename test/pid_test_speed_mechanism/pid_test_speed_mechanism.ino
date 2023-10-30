#define motorDir1 6
#define motorDir2 4
#define motorpwm 5
#define encoderPinA 2
#define encoderPinB 3


//  Encoder
volatile long  encoderValue = 0;
unsigned long previousMillis = 0;

int counts = 6533;  // Encoder Pulse per revolution.

//Define Variables we'll be connecting to
float input, setpoint;
long dt = 100;  // Sampling time in seconds

//Specify the links and initial tuning parameters
float output, cv1, error, error1, error2;
//float Kp=6, Kd=0.00, Ki=10, Tm=0.1;            // modify for optimal performance
float Kp=2, Kd=0.00, Ki=4, Tm=0.1;            // modify for optimal performance






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


  // Set initial values
  setpoint = 30; //RPM
  
  //velocity rad/seg max 2.6
  //maximo sin carga 7.9 rad/seg or 76rpm. 

}

void loop() {

  unsigned long currentMillis = millis();
  if((currentMillis - previousMillis)>= dt) {
    previousMillis = currentMillis;
    input = 10*encoderValue *( 60 / (double)counts) ; //rpm del eje principal  
    encoderValue=0; //velocidad
  } 

  //PID Control
  
  error = setpoint - input;
  output = cv1 + (Kp + Kd/Tm)*error + (-Kp + Ki*Tm - 2*Kd/Tm)*error1+ (Kd/Tm)*error2 ;
  cv1 = output;
  error2 = error1;
  error1 = error;

  if(output > 255.0){
    output = 255.0;
  }else if (output < -255.0) {
    output = -255.0;
  }

  
  pwmOut(output);
  
  Serial.print(setpoint);
  Serial.print(",");
  Serial.print(input);
  Serial.print(",");
  Serial.println(output);
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
