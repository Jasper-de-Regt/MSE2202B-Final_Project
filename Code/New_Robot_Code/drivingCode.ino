// driving functions
// void followWall(int ci_drive_speed, char wallSide, int desiredDistance); // call to follow a wall. Example followWall(1600, R, 15) will follow a wall on the right side, maintaining a distance of 15cm, at a speed of 1600
// void moveFurtherFromWall(int ci_drive_speed, char wallSide); //wallSide should be either R or L (capital or lowercase)
//void stopDrive(); //Stops the motors
//void driveStraight(int ci_drive_speed); //Sets both motors at the same speed
//void driveStraightAheadEncoders(int ci_drive_speed, int encoderTicks); //ci_drive_speed is a constant, encoder ticks corresponds to the distance you want to travel. **1000 encoder ticks makes for about 15.5" or 39.4cm
// Turning functions
//void turnRight(int ci_drive_speed, int speedModifier); //ci_drive_speed is const. speedModifier is added/subtracted from left/right respectively.
//void turnRightSharp(int ci_drive_speed, int speedModifier); //similar to turnRight(), but speedModifier is multiplied by 1.5
//void turnLeft(int ci_drive_speed, int speedModifier); //ci_drive_speed is const. speedModifier is added/subtracted from right/left respectively.
//void turnLeftSharp(int ci_drive_speed, int speedModifier); //similar to turnLeft(), but speedModifier is multiplied by 1.5
//void skidsteerNinetyRight(int ci_drive_speed); // ci_drive_speed will be a constant as defined in the main code
//void skidsteerNinetyLeft(int ci_drive_speed) // Same note as above



// some mini functions, mainly used in followWall()
// stops both drive motors....
void stopDrive() {
  right_motor.writeMicroseconds(1500);
  left_motor.writeMicroseconds(1500);
}

// does a wide turn right
void turnRight(int ci_drive_speed, int speedModifier) {
  right_motor.writeMicroseconds(ci_drive_speed - speedModifier);
  left_motor.writeMicroseconds(ci_drive_speed + speedModifier);
}
// does a "tighter than turnRight" turn right
void turnRightSharp(int ci_drive_speed, int speedModifier) {
  right_motor.writeMicroseconds(ci_drive_speed - speedModifier * 1.5);
  left_motor.writeMicroseconds(ci_drive_speed + speedModifier * 1.5);
}
// does a wide turn left
void turnLeft(int ci_drive_speed, int speedModifier) {
  right_motor.writeMicroseconds(ci_drive_speed + speedModifier);
  left_motor.writeMicroseconds(ci_drive_speed - speedModifier);
}
// does a "tighter than turnLeft" turn left
void turnLeftSharp(int ci_drive_speed, int speedModifier) {
  right_motor.writeMicroseconds(ci_drive_speed + speedModifier * 1.5);
  left_motor.writeMicroseconds(ci_drive_speed - speedModifier * 1.5);
}//****************end of mini functions****************end of mini functions****************

// sets both motors to the same speed, needs some control code to drive straight
void driveStraight(const int ci_drive_speed) {

  // if it has been awhile since this function was called, update leftSpeed with the passed speed value
  if ((millis() - lastDriveStraightUpdateTime) > 40) {
    leftSpeedDriveStraight = ci_drive_speed;
  }

  // the left motor speed is updated every 20mS in this loop
  if ((millis() - lastDriveStraightUpdateTime) > 20) {
    int error = encoder_rightMotor.getRawPosition() - encoder_leftMotor.getRawPosition();   // error is the difference in .getRawPositions()
    if (error < 0) {        // if the left motor went too far, slow it down
      leftSpeedDriveStraight -= 15;
    }
    else if (error > 0) {       // else if the left motor didnt go far enough, speed it up
      leftSpeedDriveStraight += 15;
    }

    leftSpeedDriveStraight = constrain(leftSpeedDriveStraight, 1500, 2000);   // constrain leftSpeedDriveStraight to values possible to send to servo
    left_motor.writeMicroseconds(leftSpeedDriveStraight);        // set leftSpeedDriveStraight
    right_motor.writeMicroseconds(ci_drive_speed);   // right motor runs constantly at the passed speed

    encoder_leftMotor.zero();    // zero encoders to prevent overflow errors
    encoder_rightMotor.zero();
    lastDriveStraightUpdateTime = millis();          // update last time the speeds were updated
  }
}






// call this to drive "straight" reverse to a new encoder value
// for example this will drive straight back at speed 1400 until both motors have incremented -1000 encoder ticks:
// driveStraightReverseEncoders(1400, -1000);
// 1000 encoder ticks makes for about 15.5" or 39.4cm
//void driveStraightReverseEncoders(int ci_drive_speed, int encoderTicks) {

//}//****************end of driveStraightreverseEncoders****************end of driveStraightreverseEncoders****************



// call this to drive "straight" ahead to a new encoder value
// for example this will drive straight ahead at speed 1600 until both motors have incremented 1000 encoder ticks:
// driveStraightAheadEncoders(1600, 1000);
// 1000 encoder ticks makes for about 15.5" or 39.4cm
void driveStraightAheadEncoders(int ci_drive_speed, int desiredPosition) {

  // if it has been awhile since this function was called, update leftSpeed with the passed speed value and reset encoderTracker
  if ((millis() - lastDriveStraightUpdateTime) > 40) {
    leftSpeedDriveStraight = ci_drive_speed;
    encoderTracker = 0;
  }

  // while the encoder ticks have not surpassed the desiredposition, the function runs
  while (encoderTracker < desiredPosition) {

    // the left motor speed is updated every 20mS in this loop
    if ((millis() - lastDriveStraightUpdateTime) > 20) {
      int error = encoder_rightMotor.getRawPosition() - encoder_leftMotor.getRawPosition();   // error is the difference in .getRawPositions()
      if (error < 0) {        // if the left motor went too far, slow it down
        leftSpeedDriveStraight -= 15;
      }
      else if (error > 0) {       // else if the left motor didnt go far enough, speed it up
        leftSpeedDriveStraight += 15;
      }

      leftSpeedDriveStraight = constrain(leftSpeedDriveStraight, 1500, 2000);   // constrain leftSpeedDriveStraight to values possible to send to servo
      left_motor.writeMicroseconds(leftSpeedDriveStraight);        // set leftSpeedDriveStraight
      right_motor.writeMicroseconds(ci_drive_speed);    // the right motor constantly runs at the passed speed

      encoderTracker += encoder_rightMotor.getRawPosition();  // tracks how far the encoder has moved

      encoder_leftMotor.zero();    // zero encoders to prevent overflow errors
      encoder_rightMotor.zero();
      lastDriveStraightUpdateTime = millis();          // update last time the speeds were updated
    }
  }
  stopDrive();            // stop motors when call has finished
  encoderTracker = 0;     // logically this is redundant as its reset at the start of the call
}//****************end of driveStraightAHeadEncoders****************end of driveStraightAHeadEncoders****************


// call this to do a 90 degree turn in place to the left
// for example, this will pivot left 90 degrees at speed 1600:
// skidsteerNinetyLeft(1600);
void skidsteerNinetyLeft(int ci_drive_speed) {
  encoder_rightMotor.zero();      // 0 both encoders
  encoder_leftMotor.zero();
  while ((encoder_rightMotor.getRawPosition() < 439) || (encoder_leftMotor.getRawPosition() > -439)) {     // turn to 90 degree encoder value, 900 encoder ticks makes for a 180
    right_motor.writeMicroseconds(ci_drive_speed);
    left_motor.writeMicroseconds(3000 - ci_drive_speed);
  }
  stopDrive();
}//************************************************************************************************************************************************


// call this to do a 90 degree turen in palce to the right
// for example, this will pivot right 90 degrees at speed 1600:
// skidsteerNinetyRight(1600);
void skidsteerNinetyRight(int ci_drive_speed) {
  encoder_rightMotor.zero();      // 0 both encoders
  encoder_leftMotor.zero();
  while ((encoder_rightMotor.getRawPosition() > -439) || (encoder_leftMotor.getRawPosition() < 439)) {      // turn to 90 degree encoder value, 900 encoder ticks makes for a 180
    right_motor.writeMicroseconds(3000 - ci_drive_speed);
    left_motor.writeMicroseconds(ci_drive_speed);
  }
  stopDrive();
}//****************end of skidsteerNinetyRight****************end of skidsteerNinetyRight****************

// call this function to move further from a wall
// start condition should be parallel to a wall
// this will spin the robot in place 90 degrees so it faces away from the wall
// then the robot will drive straight forward (away from wall)
// then the robot will spin in place 90 degrees so it faces opposite start direction (now further from the wall)
// return. At this point the robot is parallel to the wall, further away than before, and facing the opposite direction
// wallSide is a char indicating on which side of the robot the wall is when making the function call.
void moveFurtherFromWall(int ci_drive_speed, char wallSide) {
  if ((wallSide == 'R') || (wallSide == 'r')) { // if wall is on right
    skidsteerNinetyLeft(ci_drive_speed);            // turn 90 left
    driveStraightAheadEncoders(1600, 203);      // drive head ~8cm
    skidsteerNinetyLeft(ci_drive_speed);            // turn 90 left again
  }
  if ((wallSide == 'L') || (wallSide == 'l')) { // if wall is on right
    skidsteerNinetyRight(ci_drive_speed);            // turn 90 left
    driveStraightAheadEncoders(1600, 203);      // drive head ~8cm
    skidsteerNinetyRight(ci_drive_speed);            // turn 90 left again
  }
}//****************end of moveFurtherFromWall****************end of moveFurtherFromWall****************


// call this function to follow a wall.
// for example, to drive at speed 1600 alongside a wall located to the right at a distance of 15 cm, call:
// followWall(1600, 'r', 15);
void followWall(int ci_drive_speed, char wallSide, int desiredDistance) {
  int speedModifier = (ci_drive_speed - 1500) / 3;    // how much to modify the speed for turns
  if ((ci_drive_speed - speedModifier * 1.5) < 1500) {    // ensure the motor wouldnt run in reverse
    speedModifier = (ci_drive_speed - 1500) / 1.5;
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
        driveStraight(ci_drive_speed);
      }
      if (frontRightSensorData < backRightSensorData) {                             // if correct distance from wall, and driving towards wall, turn away from wall a little
        turnLeft(ci_drive_speed, speedModifier);
      }
      if (frontRightSensorData > backRightSensorData) {                             // if correct distance from wall, and driving away from wall, turn towards wall a little
        turnRight(ci_drive_speed, speedModifier);
      }
    }
    ////////////////////////////////////////////////////////////////////////////////
    if (((frontRightSensorData + backRightSensorData) / 2) < desiredDistance) {     // if too close to the wall
      if (frontRightSensorData == backRightSensorData) {                            // if too close to the wall, and driving parallel to wall, turn away from wall a little
        turnLeft(ci_drive_speed, speedModifier);
      }
      if (frontRightSensorData < backRightSensorData) {                             // if too close to the wall, and driving towards the wall, turn away from wall a bunch
        turnLeftSharp(ci_drive_speed, speedModifier);
      }
      if (frontRightSensorData > backRightSensorData) {                             // if too close to the wall, and driving away from wall, drive straight
        driveStraight(ci_drive_speed);
      }
    }
    ////////////////////////////////////////////////////////////////////////////////
    if (((frontRightSensorData + backRightSensorData) / 2) > desiredDistance) {     // if too far away from the wall
      if (frontRightSensorData == backRightSensorData) {                            // if too far away from the wall, and parallel to the wall, turn towards wall slightly
        turnRight(ci_drive_speed, speedModifier);
      }
      if (frontRightSensorData < backRightSensorData) {                             // if too far away from the wall, and angled towards the wall, drive straight
        driveStraight(ci_drive_speed);
      }
      if (frontRightSensorData > backRightSensorData) {                              // if too far from the wall, and heading away from the wall, turn towards the wall a bunch
        turnRightSharp(ci_drive_speed, speedModifier);
      }
    }
  }                                    // end of   if (wallside=='r')
  //*************************************************************************************************************************************************************************************/
  // if the wall is on the left side
  if ((wallSide == 'L') || (wallSide == 'l')) {
    int frontLeftSensorData = frontLeftPingSensor.ping_cm();                      //populate int with sensor data (in centimeters)
    int backLeftSensorData = backLeftPingSensor.ping_cm();
    ////////////////////////////////////////////////////////////////////////////////
    if (((frontLeftSensorData + backLeftSensorData) / 2) == desiredDistance) {    // if the correct distance away from wall
      if (frontLeftSensorData == backLeftSensorData) {                            // if correct distance from wall, and driving parallel, drive straight
        driveStraight(ci_drive_speed);
      }
      if (frontLeftSensorData < backLeftSensorData) {                             // if correct distance from wall, and driving towards wall, turn away from wall a little
        turnRight(ci_drive_speed, speedModifier);
      }
      if (frontLeftSensorData > backLeftSensorData) {                             // if correct distance from wall, and driving away from wall, turn towards wall a little
        turnLeft(ci_drive_speed, speedModifier);
      }
    }
    ////////////////////////////////////////////////////////////////////////////////
    if (((frontLeftSensorData + backLeftSensorData) / 2) < desiredDistance) {     // if too close to the wall
      if (frontLeftSensorData == backLeftSensorData) {                            // if too close to the wall, and driving parallel to wall, turn away from wall a little
        turnRight(ci_drive_speed, speedModifier);
      }
      if (frontLeftSensorData < backLeftSensorData) {                             // if too close to the wall, and driving towards the wall, turn away from wall a bunch
        turnRightSharp(ci_drive_speed, speedModifier);
      }
      if (frontLeftSensorData > backLeftSensorData) {                             // if too close to the wall, and driving away from wall, drive straight
        driveStraight(ci_drive_speed);
      }
    }
    ////////////////////////////////////////////////////////////////////////////////
    if (((frontLeftSensorData + backLeftSensorData) / 2) > desiredDistance) {     // if too far away from the wall
      if (frontLeftSensorData == backLeftSensorData) {                            // if too far away from the wall, and parallel to the wall, turn towards wall slightly
        turnLeft(ci_drive_speed, speedModifier);
      }
      if (frontLeftSensorData < backLeftSensorData) {                             // if too far away from the wall, and angled towards the wall, drive straight
        driveStraight(ci_drive_speed);
      }
      if (frontLeftSensorData > backLeftSensorData) {                              // if too far from the wall, and heading away from the wall, turn towards the wall a bunch
        turnLeftSharp(ci_drive_speed, speedModifier);
      }
    }
  }                              // end of   if (wallside=='l')
}//****************end of followWall****************end of followWall****************

