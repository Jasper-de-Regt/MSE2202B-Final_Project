// type servo position 0 to 180 in serial monitor and it will command servo to move to that position
//usefull for determining servo travel endpoints, midpoints, etc

String readString;
#include <Servo.h>
Servo myservo;  // create servo object to control a servo

void setup() {
  Serial.begin(9600);               // pour a bowl of serial
  myservo.attach(2, 500, 2500);    // the pin (9) for the servo control, and range of motion (0-180 degrees)
  Serial.println("servo-test");     // beautiful message for serial monitor
}

void loop() {

  while (Serial.available()) {
    char c = Serial.read();   //gets one byte from serial buffer
    readString += c;          //makes the string readString
    delay(2);                 //slow down looping to allow buffer to fill with next character
  }

  if (readString.length() > 0) { //if something has been sent over serial
    Serial.println(readString);  //echos back value that you sent, so you know serial is working
    int n = readString.toInt();  //convert readString into a number

    // if number is >= 500 then the code assumes you are sending microseconds (1000-2000)
    if (n >= 500)
    {
      Serial.print("writing Microseconds: ");
      Serial.println(n);
      myservo.writeMicroseconds(n);
    }
    else  // else if the number ie less than 500, the code assumes you are trying to write an angle (0-180)
    {
      Serial.print("writing Angle: ");
      Serial.println(n);
      myservo.write(n);
    }

    readString = ""; //empty for next input
  }
}
