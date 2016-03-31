#ifndef robot_functions
#define robot_functions

/* Function List + How to Call

  void moveToPosn(servo, servo pin, initial posn, final posn, bool to start at initial);
  void moveFurtherFromWall(int driveSpeed, char wallSide); //wallSide should be either R or L (capital or lowercase)
  void skidsteerNinetyRight(int driveSpeed); // driveSpeed will be a constant as defined in the main code
  void skidsteerNinetyLeft(int driveSpeed) // Same note as above
  void driveStraightAheadEncoders(int driveSpeed, int encoderTicks); //driveSpeed is a constant, encoder ticks corresponds to the distance you want to travel. **1000 encoder ticks makes for about 15.5" or 39.4cm

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


void moveFurtherFromWall(int driveSpeed, char wallSide) {
  if ((wallSide == 'R') || (wallSide == 'r')) { // if wall is on right
    skidsteerNinetyLeft(driveSpeed);            // turn 90 left
    driveStraightAheadEncoders(1600, 203);      // drive head ~8cm
    skidsteerNinetyLeft(driveSpeed);            // turn 90 left again
  }
  if ((wallSide == 'L') || (wallSide == 'l')) { // if wall is on right
    skidsteerNinetyLeft(driveSpeed);            // turn 90 left
    driveStraightAheadEncoders(1600, 203);      // drive head ~8cm
    skidsteerNinetyLeft(driveSpeed);            // turn 90 left again
  }
}


// call this to do a 90 degree pivot to the right
// for example, this will pivot right 90 degrees at speed 1600. skidsteerNinetyRight(1600);
void skidsteerNinetyRight(int driveSpeed) {
  encoder_rightMotor.zero();      // 0 both encoders
  encoder_leftMotor.zero();
  while ((encoder_rightMotor.getRawPosition() > -439) || (encoder_leftMotor.getRawPosition() < 439)) {      // turn to 90 degree encoder value, 900 encoder ticks makes for a 180
    servo_rightMotor.writeMicroseconds(3000 - driveSpeed);
    servo_leftMotor.writeMicroseconds(driveSpeed);
  }
  stopDrive();
}
// call this to do a 90 degree pivot to the left
// for example, this will pivot left 90 degrees at speed 1600. skidsteerNinetyLeft(1600);
void skidsteerNinetyLeft(int driveSpeed) {
  encoder_rightMotor.zero();      // 0 both encoders
  encoder_leftMotor.zero();
  while ((encoder_rightMotor.getRawPosition() < 439) || (encoder_leftMotor.getRawPosition() > -439)) {     // turn to 90 degree encoder value, 900 encoder ticks makes for a 180
    servo_rightMotor.writeMicroseconds(driveSpeed);
    servo_leftMotor.writeMicroseconds(3000 - driveSpeed);
  }
  stopDrive();
}



// call this to drive "straight" ahead to a new encoder value
// for example this will drive straight ahead at speed 1600 until both motors have incremented 1000 encdoer ticks. driveStraightAheadEncoders(1600, 1000);
// 1000 encoder ticks makes for about 15.5" or 39.4cm
void driveStraightAheadEncoders(int driveSpeed, int encoderTicks) {
  encoder_rightMotor.zero();      // 0 both encoders
  encoder_leftMotor.zero();
  while ((encoder_rightMotor.getRawPosition() < encoderTicks) || (encoder_leftMotor.getRawPosition() < encoderTicks)) {      // drive ahead to encoder value
    servo_rightMotor.writeMicroseconds(driveSpeed);
    servo_leftMotor.writeMicroseconds(driveSpeed);
  }
  stopDrive();
}

//moves turntable to desired position
void turnTurntableEncodersPosition(int encoderPosition) {
  if ((encoder_turntable_motor.getRawPosition() - encoderPosition) < 0) {
    while ((encoder_turntable_motor.getRawPosition() - encoderPosition) < 0) {
      servo_turntable_motor.writeMicroseconds(1600);
    }
  }
  else {
    while ((encoder_turntable_motor.getRawPosition() - encoderPosition) > 0) {
      servo_turntable_motor.writeMicroseconds(1400);
    }
  }
  stopTurntable();
}

//moves arm to desired position
void armEncoderPosition(int encoderPosition) {
  if ((encoder_arm_motor.getRawPosition() - encoderPosition) < 0) {
    while ((encoder_arm_motor.getRawPosition() - encoderPosition) < 0) {
      servo_arm_motor.writeMicroseconds(1600);
    }
  }
  else {
    while ((encoder_arm_motor.getRawPosition() - encoderPosition) > 0) {
      servo_arm_motor.writeMicroseconds(1400);
    }
  }
  stopArm();
}

int hallEffectMeasurement(){
  return analogRead(ci_hall_effect); //Range: 0-1024
}

// scans for fluctuating magnetic field to see if there is a magnetic tesseract, return true if true
void tesseractScanSweep(int maxPosition) {
  for(int i=encoder_turntable_motor.getRawPosition(); i<maxPosition; i+10){
  turnTurntableEncodersPosition(i);
  if(/*hallEffectMeasurement*/){       //checks to see if there is an abnormality in the hall effects analog read, if yes then break the loop
    break;
    }
  
  }
}

// call this function to follow a wall. Example followWall(R, 15, 1600) will follow a wall on the right side, maintaining a distance of 15cm, at a speed of 1600
void followWall(int driveSpeed, char wallSide, int desiredDistance) {

  int speedModifier = (driveSpeed - 1500) / 3;    // how much to modify the speed for turns
  if ((driveSpeed - speedModifier * 1.5) < 1500) {    // ensure the motor wouldnt run in reverse
    speedModifier = (driveSpeed - 1500) / 1.5;
  }

  int frontLeftSensorData = frontLeftPingSensor.ping_cm();
  int backLeftSensorData = backLeftPingSensor.ping_cm();

  // if the wall is on the right side
  if ((wallSide == 'R') || (wallSide == 'r')) {
    int frontRightSensorData = frontRightPingSensor.ping_cm();                      //populate int with sensor data (in centimeters)
    int backRightSensorData = backRightPingSensor.ping_cm();
    ////////////////////////////////////////////////////////////////////////////////
    if (((frontRightSensorData + backRightSensorData) / 2) == desiredDistance) {    // if the correct distance away from wall
      if (frontRightSensorData == backRightSensorData) {                            // if correct distance from wall, and driving parallel, drive straight
        driveStraight(driveSpeed);
      }
      if (frontRightSensorData < backRightSensorData) {                             // if correct distance from wall, and driving towards wall, turn away from wall a little
        turnLeft(driveSpeed, speedModifier);
      }
      if (frontRightSensorData > backRightSensorData) {                             // if correct distance from wall, and driving away from wall, turn towards wall a little
        turnRight(driveSpeed, speedModifier);
      }
    }
    ////////////////////////////////////////////////////////////////////////////////
    if (((frontRightSensorData + backRightSensorData) / 2) < desiredDistance) {     // if too close to the wall
      if (frontRightSensorData == backRightSensorData) {                            // if too close to the wall, and driving parallel to wall, turn away from wall a little
        turnLeft(driveSpeed, speedModifier);
      }
      if (frontRightSensorData < backRightSensorData) {                             // if too close to the wall, and driving towards the wall, turn away from wall a bunch
        turnLeftSharp(driveSpeed, speedModifier);
      }
      if (frontRightSensorData > backRightSensorData) {                             // if too close to the wall, and driving away from wall, drive straight
        driveStraight(driveSpeed);
      }
    }
    ////////////////////////////////////////////////////////////////////////////////
    if (((frontRightSensorData + backRightSensorData) / 2) > desiredDistance) {     // if too far away from the wall
      if (frontRightSensorData == backRightSensorData) {                            // if too far away from the wall, and parallel to the wall, turn towards wall slightly
        turnRight(driveSpeed, speedModifier);
      }
      if (frontRightSensorData < backRightSensorData) {                             // if too far away from the wall, and angled towards the wall, drive straight
        driveStraight(driveSpeed);
      }
      if (frontRightSensorData > backRightSensorData) {                              // if too far from the wall, and heading away from the wall, turn towards the wall a bunch
        turnRightSharp(driveSpeed, speedModifier);
      }
    }
  }                                                             // end of   if (wallside=='r')
  //*************************************************************************************************************************************************************************************//
  // if the wall is on the left side
  if ((wallSide == 'L') || (wallSide == 'l')) {
    int frontLeftSensorData = frontLeftPingSensor.ping_cm();                      //populate int with sensor data (in centimeters)
    int backLeftSensorData = backLeftPingSensor.ping_cm();
    ////////////////////////////////////////////////////////////////////////////////
    if (((frontLeftSensorData + backLeftSensorData) / 2) == desiredDistance) {    // if the correct distance away from wall
      if (frontLeftSensorData == backLeftSensorData) {                            // if correct distance from wall, and driving parallel, drive straight
        driveStraight(driveSpeed);
      }
      if (frontLeftSensorData < backLeftSensorData) {                             // if correct distance from wall, and driving towards wall, turn away from wall a little
        turnRight(driveSpeed, speedModifier);
      }
      if (frontLeftSensorData > backLeftSensorData) {                             // if correct distance from wall, and driving away from wall, turn towards wall a little
        turnLeft(driveSpeed, speedModifier);
      }
    }
    ////////////////////////////////////////////////////////////////////////////////
    if (((frontLeftSensorData + backLeftSensorData) / 2) < desiredDistance) {     // if too close to the wall
      if (frontLeftSensorData == backLeftSensorData) {                            // if too close to the wall, and driving parallel to wall, turn away from wall a little
        turnRight(driveSpeed, speedModifier);
      }
      if (frontLeftSensorData < backLeftSensorData) {                             // if too close to the wall, and driving towards the wall, turn away from wall a bunch
        turnRightSharp(driveSpeed, speedModifier);
      }
      if (frontLeftSensorData > backLeftSensorData) {                             // if too close to the wall, and driving away from wall, drive straight
        driveStraight(driveSpeed);
      }
    }
    ////////////////////////////////////////////////////////////////////////////////
    if (((frontLeftSensorData + backLeftSensorData) / 2) > desiredDistance) {     // if too far away from the wall
      if (frontLeftSensorData == backLeftSensorData) {                            // if too far away from the wall, and parallel to the wall, turn towards wall slightly
        turnLeft(driveSpeed, speedModifier);
      }
      if (frontLeftSensorData < backLeftSensorData) {                             // if too far away from the wall, and angled towards the wall, drive straight
        driveStraight(driveSpeed);
      }
      if (frontLeftSensorData > backLeftSensorData) {                              // if too far from the wall, and heading away from the wall, turn towards the wall a bunch
        turnLeftSharp(driveSpeed, speedModifier);
      }
    }
  }                                                             // end of   if (wallside=='l')
}



// some mini functions, mainly used in followWall()
void stopDrive() {
  servo_rightMotor.writeMicroseconds(1500);
  servo_leftMotor.writeMicroseconds(1500);
}
void stopTurntable() {
  servo_turntable_motor.writeMicroseconds(1500);
}
void stopArm() {
  servo_arm_motor.writeMicroseconds(1500);
}
void driveStraight(int driveSpeed) {
  servo_rightMotor.writeMicroseconds(driveSpeed);
  servo_leftMotor.writeMicroseconds(driveSpeed);
}
void turnRight(int driveSpeed, int speedModifier) {
  servo_rightMotor.writeMicroseconds(driveSpeed - speedModifier);
  servo_leftMotor.writeMicroseconds(driveSpeed + speedModifier);
}
void turnRightSharp(int driveSpeed, int speedModifier) {
  servo_rightMotor.writeMicroseconds(driveSpeed - speedModifier * 1.5);
  servo_leftMotor.writeMicroseconds(driveSpeed + speedModifier * 1.5);
}
void turnLeft(int driveSpeed, int speedModifier) {
  servo_rightMotor.writeMicroseconds(driveSpeed + speedModifier);
  servo_leftMotor.writeMicroseconds(driveSpeed - speedModifier);
}
void turnLeftSharp(int driveSpeed, int speedModifier) {
  servo_rightMotor.writeMicroseconds(driveSpeed + speedModifier * 1.5);
  servo_leftMotor.writeMicroseconds(driveSpeed - speedModifier * 1.5);
}




#endif
