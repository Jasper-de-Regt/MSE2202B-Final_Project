
void stopTurntable() {
  turntable_motor.writeMicroseconds(1500);
}
void stopArm() {
  arm_motor.writeMicroseconds(1500);
}

//moves turntable to desired position
void turnTurntableEncodersPosition(int encoderPosition) {
  if ((encoder_turntable_motor.getRawPosition() - encoderPosition) < 0) {
    while ((encoder_turntable_motor.getRawPosition() - encoderPosition) < 0) {
      turntable_motor.writeMicroseconds(1600);
    }
  }
  else {
    while ((encoder_turntable_motor.getRawPosition() - encoderPosition) > 0) {
      turntable_motor.writeMicroseconds(1400);
    }
  }
  stopTurntable();
}

//moves arm to desired position



void armEncoderPosition(int encoderPosition) {
  if ((encoder_arm_motor.getRawPosition() - encoderPosition) < 0) {
    while ((encoder_arm_motor.getRawPosition() - encoderPosition) < 0) {
      arm_motor.writeMicroseconds(1600);
    }
  }
  else {
    while ((encoder_arm_motor.getRawPosition() - encoderPosition) > 0) {
      arm_motor.writeMicroseconds(1400);
    }
  }
  stopArm();
}

/*
// scans for fluctuating magnetic field to see if there is a magnetic tesseract, return true if true

void tesseractScanSweep() {
  int tesseractLocation;
  int encoderMaxHallRead = 0;
  wristSweep(ci_wrist_scanning);
  //armEncoderPosition(ci_arm_horizontal_position);
  //still needs the arm height function
  for (int i = encoder_turntable_motor.getRawPosition(); i < ci_turntable_right_position; i + 30) { // not sure if this is the best way to scan
    if (analogRead(ci_hall_effect_pin) > encoderMaxHallRead) {
      tesseractLocation = encoder_turntable_motor.getRawPosition();
    }
    turnTurntableEncodersPosition(i);
  }
  turnTurntableEncodersPosition(tesseractLocation);
  magnetSweep(ci_magnet_down_position);
  magnetSweep(ci_magnet_up_position);
  //armEncoderPosition(ci_arm_diagonal_position)
  //turntableEncoderPosition(ci_turntable_middle_position);
}
*///had to remove due to play in gears without PID


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


void magnetSweep(int final_Position) {
  //high is up, low is down
  if (servo_magnet.read() > final_Position) {
    for (int i = servo_magnet.read(); i > final_Position; i-- ) {
      servo_magnet.write(i);
      delay(15);
    }
  }
  else {
    for (int i = servo_magnet.read(); i < final_Position; i++) {
      servo_magnet.write(i);
      delay(15);
    }
  }
}

void wristSweep(int final_Position) {
  //
  if (servo_wrist.read() > final_Position) {
    for (int i = servo_wrist.read(); i > final_Position; i-- ) {
      servo_wrist.write(i);
      delay(15);
    }
  }
  else {
    for (int i = servo_wrist.read(); i < final_Position; i++) {
      servo_wrist.write(i);
      delay(15);
    }
  }
}

