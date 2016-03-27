#include <Servo.h>

Servo serv0;
int delayBy = 10;


void setup() {
  Serial.begin(9600);

  serv0.attach(7);

  //Sets the initial servo position
  serv0.write(180);


  Serial.println(serv0.read());
}

void loop() {
  /*serv0.write(serv0.read() - 1);
  delay(50);
  Serial.println(serv0.read());*/

  moveToPosn(180, 0, 1);
  serv0.detach();

}

void moveToPosn(int initial, int final, int increment, bool hardStartTrue) {
  //If the position must start at the initial mark
  if (hardStartTrue == true) { 
    while (serv0.read() != initial) {
      serv0.write(initial);
    }
  }

  if (initial > final) {
    while (serv0.read() > final) {
      serv0.write(serv0.read() - increment);
      delay(delayBy);
    }
  }
  else if (initial < final) {
    while (serv0.read() < final) {
      serv0.write(serv0.read() + increment);
      delay(delayBy);
    }
  }

}
