// prototype navigation code

#ifndef navigation
#define navigation

/*
VARS I NEED
bool bt_holding_tesseract = false;
const int ci_IR_crown
unsigned int ui_num_turns
char ch_tracking_direction
const float cf_robot_diameter

Functions I need
driveStraightAheadEncoders()
followWall()
skidSteerNinetyRight()
skidSteerNinetyLeft()
stopDrive()
tesseractArmScan()



*/


while ((digitalRead(ci_IR_crown) != 0) && (!bt_holding_tesseract)) {
if (ui_num_turns == 0) { //If the robot has yet to turn
    driveStraightAheadEncoders(ci_drive_speed, 100); //Encoder tickers corresponds to ~1.55 cm. 
    //Won't get interrupted, but 
    //May get interrupted by a sensor getting tripped or some other condition.
  }
else if (ui_num_turns > 0) {
    if ((ui_num_turns % 2) == 1) {
      ch_tracking_direction = 'R';
    }
    else if ((ui_num_turns % 2) == 0) {
      ch_tracking_direction = 'L';
    }

    // Follow the wall from the tracking direction that corresponds to the number of turns
    followWall(ci_drive_speed, ch_tracking_direction , ((cf_robot_diameter / 3) + cf_robot_diameter * (ui_num_turns - 1)) ); //Sets the bot to follow the wall on the right hand side

    //If proximity to wall is <= 8 cm AND the wrist bar is parallel to the ground

    //The ultrasonic sensor isn't reliable below 7 cm
    if ((ui_front_distance_reading <= 8) && (servo_wrist_motor.read() >= ci_wrist_parallel)) {
      //Do a sweep
      // GET FUNCTION FROM MIKE
      //
      //
      //
      if ((ch_tracking_direction == 'L') || (ch_tracking_direction == 'l')) {
        skidsteerNinetyLeft(ci_drive_speed);
        skidsteerNinetyLeft(ci_drive_speed);
        ui_num_turns++;
      }
      else if ((ch_tracking_direction == 'R') || (ch_tracking_direction == 'r')) {
        skidsteerNinetyRight(ci_drive_speed);
        skidsteerNinetyRight(ci_drive_speed);
        ui_num_turns++;
      }
    }
}
}
if (digitalRead(ci_IR_crown) == 1) {
  stopDrive();
  driveStraightAheadEncoders(1500-(ci_drive_speed-1500), 130); //Drives back 2 in
  stopDrive();
  //tesseractScanSweep(ci_turntable_right);
  //tesseractScanSweep(ci_turntable_left);
  tesseractArmScan();

  //May need to add function -> turntableEncoders with different arm height to sweep bad tesseracts
}

if (bt_holding_tesseract) {
/*
Make sure the arm is up.
*/

  if (ui_num_turns  == 0){
    //Should turn 180 degrees on the spot
    skidsteerNinetyRight(ci_drive_speed);
    skidsteerNinetyRight(ci_drive_speed);
  }
  else if (ui_num_turns > 0) {
    //Turn back toward the wall
    if ((ch_tracking_direction == 'l') || (ch_tracking_direction == 'L')) {
      skidsteerNinetyLeft(ci_drive_speed);
    }
    else if ((ch_tracking_direction == 'r') || (ch_tracking_direction == 'R')) {
      skidsteerNinetyRight(ci_drive_speed);
    }

    //If away from the wall, drive toward the wall
    while (frontSensorData.ping_cm() >= 8) {
      driveStraightAheadEncoders(ci_drive_speed, 50); //Drive forward a half inch
    }

    //Turn left to face the origin
    skidsteerNinetyLeft(ci_drive_speed);

    //Drive toward the wall.
    while (frontSensorData.ping_cm() >= 8) {
      driveStraightAheadEncoders(ci_drive_speed, 50); //Drive forward a half inch
    }

    //When you get to the origin (roughly speaking), stop driving.
    stopDrive();

    //Turn 180
    skidsteerNinetyRight(ci_drive_speed);
    skidsteerNinetyRight(ci_drive_speed);

  }
}













#endif
