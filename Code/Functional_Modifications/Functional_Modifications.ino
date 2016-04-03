void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

//turning functions

// does a wide turn right
void turnRight(int ci_drive_speed, int speedModifier) {
  //Assumes ci_drive_speed is > 1500
  
  if ((ci_drive_speed - speedModifier) < 1500) {
    right_motor.writeMicroseconds(1500);
  }
  else {
    right_motor.writeMicroseconds(ci_drive_speed - speedModifier);
  }
  
  left_motor.writeMicroseconds(ci_drive_speed + speedModifier);
}


// does a "tighter than turnRight" turn right
void turnRightOnPoint(int ci_drive_speed, int speedModifier) {
  //Assumes ci_drive_speed is > 1500
  //Get right motor to drive backwards at the same speed as the left motor drives forward
  right_motor.writeMicroseconds(1500 - ci_drive_speed - speedModifier * 1.5);
  left_motor.writeMicroseconds(ci_drive_speed + speedModifier * 1.5);
}
