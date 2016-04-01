#include <Servo.h>
//VAR
int timeDiff = 20; //Set as global.

void setup() {
}
void loop() {
}
void moveToPosition(Servo serv0, int final_position) {

  long previous = millis();
  
  while (serv0.read() != final_position) {

    if ((millis() - previous) >= timeDiff) {
      if (serv0.read() > final_position) {
        serv0.write(serv0.read() - 1);
      }
      if (serv0.read() < final_position) {
        serv0.write(serv0.read() + 1);
      } 
      previous = millis();
    }
  }
}

