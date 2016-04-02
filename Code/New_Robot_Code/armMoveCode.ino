//****************************************************************************//
//****************************************************************************//
//******************************** NOTES *************************************//
//*** List of functions in this header file:
//***
//*** void sweepServo(Servo servo, int desiredPosition);   // pass the desired position as an angle
//*** void moveTurntable(int desiredPosition)              // pass the desiredPosition as an encoder value
//*** void moveArm(int desiredPosition)                    // pass the desiredPosition as an encoder value
//*** bool tesseractArmScan();                             // scans with arm for tesseract, picks tesseract up or pushes it aside






// this function will move the arm to left at a scanning height with the magnet retracted
// it will then sweep the arm to the right while scanning with the hall effect sensor
// the greatest hall effect angle is stored
// if a magnetic field was detected, the arm will turn to that position, extend the magnet, raise/center the arm, and return true
// if a magnetic field was not detected, the arm will drop, sweep left in an attempt to knock away the bad tesseract, and return false
bool tesseractArmScan() {
  // retract magnet
  servo_magnet.write(ci_magnet_retract);
  // move arm up
  moveArm(ci_arm_carry_height);
  // wrist out?
  sweepServo(servo_wrist, ci_wrist_parallel);
  // move turntable to far left position
  moveTurntable(ci_turntable_left);
  // take a hall effect reading out in the air to determine a sensor threshold
  // this wasn't working as a global constant int, so this "calibrates" the value every fn call
  const int ci_hall_effect_scanning_threshold = analogRead(ci_hall_effect_pin)+10;  //nothing~515, near~530
  // move arm to scanning height
  moveArm(ci_arm_scanning_height);
  // move wrist to 90 degrees down
  sweepServo(servo_wrist, ci_wrist_scan);


  // move turntable from left to right while polling the hall effect sensor
  // store the greatest hall effect value and remmeber at which encoder angle it occurs
  // not too picky about overshooting the far right angle slightly due to arm momentum, inconsequential
  int greatestHallReading = 0;
  int greatestHallEncoderAngle = 0;
  while (encoder_turntable.getRawPosition() < ci_turntable_right) {
    turntable_motor.writeMicroseconds(1670);
    if (analogRead(ci_hall_effect_pin) > greatestHallReading) {
      greatestHallReading = analogRead(ci_hall_effect_pin);
      greatestHallEncoderAngle = encoder_turntable.getRawPosition();
      Serial.println();
      Serial.print("new greatestHallValue: ");
      Serial.print(greatestHallReading);
      Serial.print("    new greatestHallEncoderAngle: ");
      Serial.print(greatestHallEncoderAngle);
    }
    Serial.println();
    Serial.print("inside scanning while loop, encoder value: ");
    Serial.print(encoder_turntable.getRawPosition());
  }
  turntable_motor.writeMicroseconds(1500);
  Serial.println();
  Serial.print("outside while loop   greatest hall reading:");
  Serial.print(greatestHallReading);
  Serial.print("   greatestHallEncoderAngle: ");
  Serial.print(greatestHallEncoderAngle);

  // if a magnetic tesseract was found, pick it up
  if (greatestHallReading > ci_hall_effect_scanning_threshold) {
    // move to best encoder value - offset
    // the offset is due to the hall effect sensor being beside the arm magnet
    moveTurntable(greatestHallEncoderAngle - 65);
    // extend magnet
    servo_magnet.write(ci_magnet_extend);
    // raise arm
    moveArm(ci_arm_carry_height);
    // move wrist to carry
    sweepServo(servo_wrist, ci_wrist_carry);
    // center turntable
    moveTurntable(ci_turntable_center);
    return true;
  }

  // if a non-magnetic tesseract was found, toss it to the side?
  else {
    // move wrist to parallal
    sweepServo(servo_wrist, ci_wrist_parallel);
    // drop arm
    moveArm(ci_arm_push_away_height);
    // move wrist to push away
    sweepServo(servo_wrist, ci_wrist_push_away);
    // sweep to far left
    moveTurntable(ci_turntable_left);
    // raise arm
    moveArm(ci_arm_carry_height);
    //center turntable
    moveTurntable(ci_turntable_center);
    return false;
  }
}


// this function sweeps (slowly moves) a servo to a new position
// this is a blocking function, the function will not return until finished sweeping to the desiredPosition
// keep in mind a servo has no position fedback to a microcontroller, there is no way to see if the servo is stalled
void sweepServo(Servo servo, int desiredPosition) {
  if (servo.read() < desiredPosition) {   // if the servo angle needs to increase
    for (int position = servo.read(); position < desiredPosition; position++) {
      servo.write(position);
      delay(15);
    }
  }
  else {                                  // else if the servo angle needs to decrease
    for (int position = servo.read(); position > desiredPosition; position--) {
      servo.write(position);
      delay(15);
    }
  }
}//****************end of sweepServo fn****************end of sweepServo fn****************





// this function moves the turntable to a new encoder value.
// this is a blocking function, it won't return until the turntable has been in the correct position for awhile
// this is basically a basic P controller
void moveTurntable(int desiredPosition) {
  // encoder values increase as the turntable moves to the right
  int speedDelta = 110;
  int tolerance = 10;         // deadband tolerance, what +/- encoder value is close enough to be considered good enough?
  bool stayInFunction = true; // this is a blocking function, stay in this function until this bool is false

  while (stayInFunction == true) {
    // clip speeds to max speeds
    if (moveSpeed > (1500 + speedDelta)) {
      moveSpeed = (1500 + speedDelta);
    }
    else if (moveSpeed < (1500 - speedDelta)) {
      moveSpeed = (1500 - speedDelta);
    }

    // if the turntable has been in the correct position for awhile (10 calls)
    // stop motor, set stayInFunction to false
    if (counter > 10) {
      moveSpeed = 1500;
      turntable_motor.writeMicroseconds(moveSpeed);
      stayInFunction = false;
    }
    // if its in the correct position, don't move and increment counter
    else if ((encoder_turntable.getRawPosition() < (desiredPosition + tolerance)) && (encoder_turntable.getRawPosition() > (desiredPosition - tolerance))) {
      turntable_motor.writeMicroseconds(1500);
      counter++;
      delay(3);
      stayInFunction = true;
    }
    // if it needs to move to the right
    else if (encoder_turntable.getRawPosition() < desiredPosition) {
      moveSpeed++;
      turntable_motor.writeMicroseconds(moveSpeed);
      counter = 0;
      stayInFunction = true;
    }
    //else if it needs to move to the left
    else if (encoder_turntable.getRawPosition() > desiredPosition) {
      moveSpeed--;
      turntable_motor.writeMicroseconds(moveSpeed);
      counter = 0;
      stayInFunction = true;
    }
  }
  counter = 0;
}//****************end moveTurntable fn****************end moveTurntable fn****************





// this function moves the arm to a new encoder value.
// this is a blocking function, it won't return until the arm has been in the correct position for awhile
// this is basically a basic P controller
void moveArm(int desiredPosition) {
  // encoder values increase as the arm moves up
  int tolerance = 10;         // deadband tolerance, what +/- encoder value is close enough to be considered good enough?
  bool stayInFunction = true; // this is a blocking function, stay in this function until this bool is false

  while (stayInFunction == true) {
    // clip speeds to max speeds
    if (moveSpeed > 1700) {
      moveSpeed = 1700;
    }
    else if ((moveSpeed < 1370) && (encoder_arm.getRawPosition() > 400)) {
      moveSpeed = 1370;
    }
    else if ((moveSpeed < 1450) && (encoder_arm.getRawPosition() < 200)) {
      moveSpeed = 1450;
    }

    // if the arm has been in the correct position for awhile (10 calls)
    // stop motor, set stayInFunction to false
    if (counter > 10) {
      moveSpeed = 1500;
      arm_motor.writeMicroseconds(moveSpeed);
      stayInFunction = false;
    }
    // if its in the correct position, don't move and increment counter
    else if ((encoder_arm.getRawPosition() < (desiredPosition + tolerance)) && (encoder_arm.getRawPosition() > (desiredPosition - tolerance))) {
      arm_motor.writeMicroseconds(1500);
      counter++;
      delay(3);
      stayInFunction = true;
    }
    // if it needs to move to the right
    else if (encoder_arm.getRawPosition() < desiredPosition) {
      moveSpeed++;
      arm_motor.writeMicroseconds(moveSpeed);
      counter = 0;
      stayInFunction = true;
    }
    //else if it needs to move to the left
    else if (encoder_arm.getRawPosition() > desiredPosition) {
      moveSpeed--;
      arm_motor.writeMicroseconds(moveSpeed);
      counter = 0;
      stayInFunction = true;
    }
  }
  counter = 0;
}//****************end moveArm fn****************end moveArm fn****************
