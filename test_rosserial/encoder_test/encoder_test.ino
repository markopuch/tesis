#define encoderPinA 2
#define encoderPinB 3
volatile long  encoderValue = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode (encoderPinA, INPUT_PULLUP);
  pinMode (encoderPinB, INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt (encoderPinA), readEncoderA, CHANGE);
  attachInterrupt (digitalPinToInterrupt (encoderPinB), readEncoderB, CHANGE);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(encoderValue);
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
