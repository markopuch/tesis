

void setup() {
  Serial.begin(9600);
  float yourFloat = 3.14159;  // Replace this with your float value

  // Round to two decimal places
  float roundedValue = round(yourFloat * 1000.0) / 1000.0;

//  input= 10*vx-5.985*w;
//  // Round to two decimal places
//  float input = round(input * 100.0) / 100.0;
//
//  Serial.println(input);
}

void loop() {
  // Your code here
}
