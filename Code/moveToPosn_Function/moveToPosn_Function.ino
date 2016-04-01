/*
//VAR
int timeDiff = 20; //Set as global.

void moveToPosn(Servo serv0, int finalPosn) {

  long previous = millis();

  while (serv0.read() != finalPosn) {

    if ((millis() - previous) >= timeDiff) {
      if (serv0.read() > final) {
        serv0.write(serv0.read() - 1);
      }
      if (serv0.read() < final) {
        serv0.write(serv0.read() + 1);
      } 
      previous = millis();
    }
  }
}
*/
