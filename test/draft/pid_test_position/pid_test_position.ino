
#define motorDir 6
#define motorpwm 5
#define encoderPinA 2
#define encoderPinB 3


//variables de ROs
float vx=0;
float w=0;

//  Encoder
volatile long  encoderValue = 0;
int counts = 8400;  // Encoder Pulse per revolution.
int angle = 360; // Maximum degree of motion.


//Define Variables we'll be connecting to
double input = 0, output = 0, setpoint = 0;
double integral = 0, previous_error = 0;
double dt = 0.001;  // Sampling time in seconds
unsigned long previousTime=0;

//Specify the links and initial tuning parameters
double kp = 440 , ki = 800 ; 
float kd = 0.5;             // modify for optimal performance





void setup() {
  Serial.begin(9600);

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
  setpoint = 360; //angulo grados

}

void loop() {
  unsigned long currentTime = millis();
  double elapsedTime = (double)(currentTime - previousTime) / 1000.0;  // Convert to seconds
  
  //PID CONTROLLER
  
  if (elapsedTime >= dt) {

    input = (encoderValue / (double)(counts)) * angle; //angulo grados
    
    
    double error = setpoint - input;
    integral += error * elapsedTime;
    
    double derivative = (error - previous_error) / elapsedTime;


    output = kp * error + ki * integral + kd * derivative;
    previous_error = error;
    
    // Apply saturation limits
    output = constrain(output, -255.0, 255.0);
    pwmOut(output);
    previousTime = currentTime;

  }
  
  Serial.print(encoderValue);
  Serial.print(",");
  Serial.print(setpoint);
  Serial.print(",");
  Serial.print(input);
  Serial.print(",");
  Serial.println(output);
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
