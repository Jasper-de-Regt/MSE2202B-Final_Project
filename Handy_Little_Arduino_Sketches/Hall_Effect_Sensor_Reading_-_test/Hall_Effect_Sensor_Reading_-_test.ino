void setup() {
  const int readPin = A0;
  //const int powerPin = 5;
  pinMode(readPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  const int readPin = A0;
  int sensorVal = analogRead(readPin);
  
  Serial.println(sensorVal);
  //Serial.println(sensorVal * (5.0/1023.0));

}
