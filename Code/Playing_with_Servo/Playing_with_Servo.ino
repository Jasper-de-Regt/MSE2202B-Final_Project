#include <Servo.h>

Servo serv0;
int delayBy = 10;
int servoPin1 = 7;


void setup() {
  Serial.begin(9600);

  serv0.attach(servoPin1);

  //Sets the initial servo position
  serv0.write(180);


  Serial.println(serv0.read());

  if (serv0.read() > 15) {
    while (serv0.read() > 15) {
      serv0.write(serv0.read() - 1);
    }
  }
  
  
}

void loop() {
  /*serv0.write(serv0.read() - 1);
  delay(50);
  Serial.println(serv0.read());*/

  moveToPosn(180, 30, 0);
  //moveToPosnByInc(0, 180, 3, 1);
  serv0.detach(); Serial.println("Detached");

}

void moveToPosnByInc(int initial, int final, int increment, bool hardStartTrue) {
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





void moveToPosn(int initial, int final, bool hardStartTrue) {


  if (serv0.attached() == false) {
    serv0.attach(servoPin1);
  }

  //If the position must start at the initial mark
  if (hardStartTrue == true) {
    Serial.println("Hard Start");
    while (serv0.read() != initial) {
      serv0.write(initial);
      delay(delayBy);
    }
  }

  while (serv0.read() != final) {
    Serial.println("Movement - MTP");
    while (serv0.read() > final) {
      serv0.write(serv0.read() - 1);
      delay(delayBy);
    }
    while (serv0.read() < final) {
      serv0.write(serv0.read() + 1);
      delay(delayBy);
    }
  }



}
