//*** void printSensorReadings();
//*** void printEncoderValues();
//*** void printPingSensorReadings();

// prints ping sensor readings in centimeters...
void printPingSensorReadings() {
  Serial.println();
  Serial.print("All readings [cm]    front: ");
  Serial.print(frontPingSensor.ping_cm());
  Serial.print("   frontLeft: ");
  Serial.print(frontLeftPingSensor.ping_cm());
  Serial.print("   backLeft: ");
  Serial.print(backLeftPingSensor.ping_cm());
  Serial.print("   frontRight: ");
  Serial.print(frontRightPingSensor.ping_cm());
  Serial.print("   backRight: ");
  Serial.print(backRightPingSensor.ping_cm());
}



// prints all 4 encoder values
void printEncoderValues() {
  Serial.println();
  Serial.print("leftMotor: ");
  Serial.print(encoder_leftMotor.getRawPosition());
  Serial.print("   rightMotor: ");
  Serial.print(encoder_rightMotor.getRawPosition());
  Serial.print("   arm: ");
  Serial.print(encoder_arm.getRawPosition());
  Serial.print("   turntable: ");
  Serial.print(encoder_turntable.getRawPosition());
}



// prints the hall effect, line tracker, and IR crown sensor data
void printSensorReadings() {
  Serial.println();
  Serial.print("ci_hall_effect_pin: ");
  Serial.print(analogRead(ci_hall_effect_pin));
  Serial.print("   ci_arm_linetracker_pin: ");
  Serial.print(analogRead(ci_arm_linetracker_pin));
  Serial.print("   ci_IR_crown_pin: ");
  if (digitalRead(ci_IR_crown_pin)) {
    Serial.print("HIGH, no tesseract detected");
  }
  else if (!digitalRead(ci_IR_crown_pin)) {
    Serial.print("LOW, tesseract detected!!");
  }
}

