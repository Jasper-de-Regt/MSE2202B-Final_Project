#include <Servo.h>

Servo tt_servo, e_servo, w_servo, m_servo;
//Turntable (tt) xy plane. Elbow (e) xyz space, Wrist (w) xyz, Magnet (m) xyz space
int delayBy = 10; //Works as low as 2 (for speed).

//Arbitrary servo Pins - can be changed manually
const int tt_servoPin = 7;
const int e_servoPin = 8;
const int m_servoPin = 9;
const int w_servoPin = 10;


void setup() {
  Serial.begin(9600);

  //Attach the servo pins
  tt_servo.attach(tt_servoPin);
  e_servo.attach(e_servoPin);
  m_servo.attach(m_servoPin);
  w_servo.attach(m_servoPin);

  //Sets the initial servo position to 0
  /* Note: All of these servos would need to be calibrated on running.
  This can occur either by testing or using sensor data.
  -Testing is probably easier to do, given the time constraints.
  */
  tt_servo.write(0);
  e_servo.write(0);
  m_servo.write(0);
  w_servo.write(0);


  //Serial.println(serv0.read());

  /*
    //Setting the initial servo position to some value at 15 degrees or less
    if (serv0.read() > 15) {
      while (serv0.read() > 15) {
        serv0.write(serv0.read() - 1);
      }
    }
  */

}

void loop() {
  /*serv0.write(serv0.read() - 1);
  delay(50);
  Serial.println(serv0.read());*/

  //Arbitrary testing commands
  moveToPosn(m_servo, 0, 110, 0);
  /* Passes the servo object, initial position,
  final position, bool of whether to enforce the initial position */
  delay(500); //REMOVE later
  moveToPosn(110, 0, 0);

  //moveToPosnByInc(0, 180, 3, 1);

  /*
  Detaching the servo to avoid overheating or erractic behaviour
  Running the same code or even power through the small servo
  for too long will
  */
  //serv0.detach(); Serial.println("Detached");

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





void moveToPosn(Servo serv0, int initial, int final, bool hardStartTrue) {

  //Checks to make sure the servo is attached
  /*if (serv0.attached() == false) {
  //  serv0.attach(servoPin1);
  }*/

  //If the position must start at the initial mark
  if (hardStartTrue == true) {
    Serial.println("Hard Start");
    while (serv0.read() != initial) {
      serv0.write(initial);
      delay(delayBy);
    }
  }

  /*
  //Need to rewrite using time differences, not delays.
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
      Serial.print("Status: "); Serial.println(serv0.read());

    }
  */

  int current = millis(); //current time
  int previous = current;

  while (serv0.read() != final) {
    if (
    
    current = millis();
  }


}
