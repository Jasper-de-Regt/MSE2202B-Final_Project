// prototype functions
void stopTurntable() {
  turntable_motor.writeMicroseconds(1500);
}
void stopArm() {
  arm_motor.writeMicroseconds(1500);
}

//moves turntable to desired position
void turnTurntableEncodersPosition(int encoderPosition) {
  if ((encoder_turntable.getRawPosition() - encoderPosition) < 0) {
    while ((encoder_turntable.getRawPosition() - encoderPosition) < 0) {
      turntable_motor.writeMicroseconds(1600);
    }
  }
  else {
    while ((encoder_turntable.getRawPosition() - encoderPosition) > 0) {
      turntable_motor.writeMicroseconds(1400);
    }
  }
  stopTurntable();
}

//moves arm to desired position



void armEncoderPosition(int encoderPosition) {
  if ((encoder_arm.getRawPosition() - encoderPosition) < 0) {
    while ((encoder_arm.getRawPosition() - encoderPosition) < 0) {
      arm_motor.writeMicroseconds(1600);
    }
  }
  else {
    while ((encoder_arm.getRawPosition() - encoderPosition) > 0) {
      arm_motor.writeMicroseconds(1400);
    }
  }
  stopArm();
}

// scans for fluctuating magnetic field to see if there is a magnetic tesseract, return true if true
void tesseractScanSweep(int maxPosition) {
  int tesseractLocation;
  int encoderMaxHallRead = 0;
  for (int i = encoder_turntable.getRawPosition(); i < maxPosition; i + 30) { // not sure if this is the best way to scan
    if (analogRead(ci_hall_effect_pin) > encoderMaxHallRead) {
      tesseractLocation = encoder_turntable.getRawPosition();
    }
    turnTurntableEncodersPosition(i);
  }
  turnTurntableEncodersPosition(tesseractLocation);
  //armEncoderPosition(ci_arm_diagonal_position);
}




void servoMoveToPosition(Servo serv0, int final_Position) {
  long previous = millis();
  while (serv0.read() != final_Position) {
    //    if ((millis() - previous) >= timeDifference) {
    if (serv0.read() > final_Position) {
      serv0.write(serv0.read() - 1);
    }
    if (serv0.read() < final_Position) {
      serv0.write(serv0.read() + 1);
    }
    previous = millis();
  }
}

