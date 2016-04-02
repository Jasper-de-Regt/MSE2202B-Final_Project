

// this function moves the turntable to a new encoder value.
// this is a blocking function, it won't return until the turntable has been in the correct position for awhile
// this is basically a basic P controller
void moveTurntable(int desiredPosition) {
  // encoder values increase as the turntable moves to the right
  int tolerance = 10;         // deadband tolerance, what +/- encoder value is close enough?
  bool stayInFunction = true; // this is a blocking function, stay in this function until this bool is false

  while (stayInFunction == true) {
    // clip speeds to max speeds
    if (moveSpeed > 1650) {
      moveSpeed = 1600;
    }
    else if (moveSpeed < 1450) {
      moveSpeed = 1400;
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
}







// this function moves the arm to a new encoder value.
// this is a blocking function, it won't return until the arm has been in the correct position for awhile
// this is basically a basic P controller
void moveArm(int desiredPosition) {
  // encoder values increase as the arm moves up
  int tolerance = 10;         // deadband tolerance, what +/- encoder value is close enough?
  bool stayInFunction = true; // this is a blocking function, stay in this function until this bool is false

  Serial.println();
  Serial.print("arm encoder: ");
  Serial.print(encoder_arm.getRawPosition());

  while (stayInFunction == true) {
    // clip speeds to max speeds
    if (moveSpeed > 1700) {
      moveSpeed = 1600;
    }
    else if ((moveSpeed < 1370)&&(encoder_arm.getRawPosition()>400)){
      moveSpeed=1370;
    }
    else if ((moveSpeed<1450)&&(encoder_arm.getRawPosition()<200)){
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
}


