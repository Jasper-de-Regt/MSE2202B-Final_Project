#ifndef robot_functions
#define robot_functions

/* Function List + How to Call

moveToPosn(servo, servo pin, initial posn, final posn, bool to start at initial);

*/



void moveToPosn(Servo serv0, int servo_pin, int initial, int final, bool hardStartTrue) {

  int current = millis(); //current time
  int previous = current;

  //Checks to make sure the servo is attached
  if (serv0.attached() == false) {
  //  serv0.attach(servo_pin);
  }

  //If the position must start at the initial mark
  if (hardStartTrue == true) {
    Serial.println("Hard Start");
    while (serv0.read() != initial) {
      if ((current - previous) >= timeDiff) {
        serv0.write(initial);
        previous = current;
      }

    }
  }

  while (serv0.read() != final) {

    if ((current - previous) >= timeDiff) {
      if (serv0.read() > final) {
        serv0.write(serv0.read() - 1);
      }
      if (serv0.read() < final) {
        serv0.write(serv0.read() + 1);
      }
      previous = current;
    }


    current = millis();
  }
  
}



#endif
