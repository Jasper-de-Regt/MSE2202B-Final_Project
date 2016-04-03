#include <Servo.h>
#include <EEPROM.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include <NewPing.h>

//  Naming conventions:
//  camelcase
//  - E.g. itLooksLikeThis

//significant words in the title should be separated by underscores "_"
//- E.g. motor_pin_one

//CAPITALIZE ACRONYMS
//- E.g. IR instead of infrared.
//    LED instead of light_emitting_diode or lightEmittingDiode


//****************************************************************
//************************GLOBAL VARIABLES************************
int moveSpeed = 1500;
int counter = 0;
// servo angle constants
const int ci_magnet_retract = 120;        //angle of servo_magnet with magnet in the "off" position
const int ci_magnet_extend = 10;          // angle of the servo_magnet with magnet in the "on" or "pickup" position
const int ci_wrist_scan = 20;             // angle of the servo_wrist in the max down position
const int ci_wrist_parallel = 98;         // angle of the servo_wrist when parallel with arm
const int ci_wrist_carry = 0;             // 0 is the correct value
const int ci_wrist_push_away = 35;        // wrist position to push away bad tesseracts
// encoder value constants
const int ci_turntable_left = 380;          // encoder ticks of the turntable at far left position
const int ci_turntable_right = 1280;        // encoder ticks of the turntable at far right position
const int ci_turntable_center = 800;        // encoder ticks of the turntable at center position (straight forward)
const int ci_arm_scanning_height = 60;       // encoder ticks with the arm at a height ideal for tesseract scanning/pickup
const int ci_arm_carry_height = 220;          // encoder ticks with the arm at height ideal for driving around and not blocking the front ping sensor
const int ci_arm_push_away_height = -46;      // encoder ticks with the arm dropped just above the ground, ready to push away bad tesseracts


//******************************************************************
//************************PORT PIN CONSTANTS************************
// servo and motor pin constants
const int ci_magnet_servo_pin = 4;      // 157 retracted, 18 extended
const int ci_wrist_servo_pin = 5;
const int ci_right_motor_pin = 8;
const int ci_left_motor_pin = 9;
const int ci_arm_motor_pin = 10;
const int ci_turntable_motor_pin = 11;
// sensor pin constants, ping is synomous with ultrasonic
const int ci_front_ping_pin = 13;
const int ci_front_right_ping_pin = 12;
const int ci_front_left_ping_pin = 2;
const int ci_back_right_ping_pin = 6;
const int ci_back_left_ping_pin = 3;
const int ci_arm_linetracker_pin = A2;
const int ci_hall_effect_pin = A3;
const int ci_IR_crown_pin = 7;          //High when no tesseract, low when tesseract
// I2C pin constants. Don't connect these pins to anything else
const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow


//*******************************************************************
//************************OBJECT DECLARATIONS************************
// declare all servo objects
Servo left_motor;
Servo right_motor;
Servo arm_motor;
Servo turntable_motor;
Servo servo_wrist;
Servo servo_magnet;
// declare all encoder objects
I2CEncoder encoder_leftMotor;
I2CEncoder encoder_rightMotor;
I2CEncoder encoder_arm;
I2CEncoder encoder_turntable;
// setup NewPing objects (ultrasonic distance sensors);
NewPing frontPingSensor(ci_front_ping_pin, ci_front_ping_pin, 200);
NewPing frontRightPingSensor(ci_front_right_ping_pin, ci_front_right_ping_pin, 200);
NewPing frontLeftPingSensor(ci_front_left_ping_pin, ci_front_left_ping_pin, 200);
NewPing backRightPingSensor(ci_back_right_ping_pin, ci_back_right_ping_pin, 200);
NewPing backLeftPingSensor(ci_back_left_ping_pin, ci_back_left_ping_pin, 200);


//*********************************************************************************************************//
//*********************************************************************************************************//
//*********************************************************************************************************//
//*********************************************************************************************************//



void setup() {
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);  // pour a bowl of serial, a special type of soup


  //*******************************************************************
  //************************PIN SETUPS*********************************
  pinMode(ci_left_motor_pin, OUTPUT);
  left_motor.attach(ci_left_motor_pin);
  pinMode(ci_right_motor_pin, OUTPUT);
  right_motor.attach(ci_right_motor_pin);
  pinMode(ci_arm_motor_pin, OUTPUT);
  arm_motor.attach(ci_arm_motor_pin);
  pinMode(ci_turntable_motor_pin, OUTPUT);
  turntable_motor.attach(ci_turntable_motor_pin);
  pinMode(ci_wrist_servo_pin, OUTPUT);
  servo_wrist.attach(ci_wrist_servo_pin);
  pinMode(ci_magnet_servo_pin, OUTPUT);
  servo_magnet.attach(ci_magnet_servo_pin);
  // sensor setups
  pinMode(ci_IR_crown_pin, INPUT);
  pinMode(ci_arm_linetracker_pin, INPUT);
  pinMode(ci_hall_effect_pin, INPUT);


  //*********************************************************************
  //************************ENCODER SETUP*********************************
  // setup encoders. Must be initiliazed in the order that they are chained together,
  // starting with the encoder directly attached to the arduino
  delay(3000);      // give I2C circuits a chance to boot up before attempting to make connections, fixes yellow encoder light issue
  encoder_leftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_leftMotor.setReversed(false);   // adjust for positive count when moving forward
  encoder_rightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_rightMotor.setReversed(true);   // adjust for positive count when moving forward
  encoder_arm.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_arm.setReversed(false);         // adjust for positive count when moving upwards
  encoder_turntable.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_turntable.setReversed(false);   // adjust for positive count when moving to the right
  encoder_turntable.zero();       // Robot arm must be positioned on the standoffs during powerup to be properly zeroed
  encoder_arm.zero();             // pretty sure encoders start out at 0 anyway and these calls are redundant
  Serial.println("setup has completed");
}//****************end setup****************end setup****************end setup****************end setup****************end setup****************



bool runOnce = true;


const int ci_left_gate_distance = 4;//values range from 3-5
const int ci_mode2_backup_after_pickup = 170; //needs value
const int ci_mode2_backup = 90; //needs value
const int ci_mode2_drive_placement = 242;//needs value
const int ci_mode2_movement_backward = 774; //needs value
const int ci_mode2_movement_forward = 774; //needs value
const int ci_restart_backup_distance = 355;//needs value
const int ci_mode2_wall_distance = 6;//values range from 5-8 mostly 6 and 7
const int ci_mode2_scan_distance = 7; //ranges 7-8

const int ci_arm_mode2_scaninng_height = 620; //needs val6ue
const int ci_arm_vertical_position = 420;//needs value
//const int ci_arm_horizontal_position = 0;//replace with drive through
const int ci_arm_mode2_tesseract_dropoff = 250; //needs value
const int ci_arm_mode2_tesseract_dropoff_bump = 300; //needs value
const int ci_arm_mode2_pickup_height = 675; //needs value
const int ci_arm_drive_through = 787;

const int ci_wrist_mode2_scanning_height = 180; //zero is the correct value
const int ci_wrist_vertical_position = 20;//needs value
const int ci_wrist_mode2_tesseract_dropoff = 127; //needs value

const int ci_turntable_mode2_placement1 = 800; //needs value
const int ci_turntable_mode2_placement2 = 1200; //needs value
const int ci_turntable_mode2_placement3 = 360; //needs value


void loop(){
  followWallReverse(1350,'R',7);
}



void followWallReverse(int ci_drive_speed, char wallSide, int desiredDistance) {
  int speedModifier = (ci_drive_speed - 1500) / 3;  // how much to modify the speed for turns
 speedModifier=-speedModifier;
 
 /*if ((ci_drive_speed - speedModifier * 1.5) < 1500) {    // ensure the motor wouldnt run in reverse
    speedModifier = (ci_drive_speed - 1500) / 1.5;
  }*/

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
      if (frontRightSensorData > backRightSensorData) {                             // if correct distance from wall, and driving towards wall, turn away from wall a little
       turnRight(ci_drive_speed, speedModifier);
      }
      if (frontRightSensorData < backRightSensorData) {                             // if correct distance from wall, and driving away from wall, turn towards wall a little
        turnLeft(ci_drive_speed, speedModifier); 
      }
    }
    ////////////////////////////////////////////////////////////////////////////////
    if (((frontRightSensorData + backRightSensorData) / 2) < desiredDistance) {     // if too close to the wall
      if (frontRightSensorData == backRightSensorData) {                            // if too close to the wall, and driving parallel to wall, turn away from wall a little
        turnRight(ci_drive_speed, speedModifier);
      }
      if (frontRightSensorData > backRightSensorData) {                             // if too close to the wall, and driving towards the wall, turn away from wall a bunch
         turnRightSharp(ci_drive_speed, speedModifier);
      }
      if (frontRightSensorData < backRightSensorData) {                             // if too close to the wall, and driving away from wall, drive straight
       driveStraight(ci_drive_speed);
      }
    }
    ////////////////////////////////////////////////////////////////////////////////
    if (((frontRightSensorData + backRightSensorData) / 2) > desiredDistance) {     // if too far away from the wall
      if (frontRightSensorData == backRightSensorData) {                            // if too far away from the wall, and parallel to the wall, turn towards wall slightly
        turnLeft(ci_drive_speed, speedModifier);
      }
     if (frontRightSensorData > backRightSensorData) {                             // if too far away from the wall, and angled towards the wall, drive straight
        driveStraight(ci_drive_speed); 
      }
     if (frontRightSensorData < backRightSensorData) {                              // if too far from the wall, and heading away from the wall, turn towards the wall a bunch
       turnLeftSharp(ci_drive_speed, speedModifier);
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
        turnLeft(ci_drive_speed, speedModifier);
      }
      if (frontLeftSensorData > backLeftSensorData) {                             // if correct distance from wall, and driving away from wall, turn towards wall a little
        turnRight(ci_drive_speed, speedModifier);
      }
    }
    ////////////////////////////////////////////////////////////////////////////////
    if (((frontLeftSensorData + backLeftSensorData) / 2) < desiredDistance) {     // if too close to the wall
      if (frontLeftSensorData == backLeftSensorData) {                            // if too close to the wall, and driving parallel to wall, turn away from wall a little
        turnLeft(ci_drive_speed, speedModifier);
      }
      if (frontLeftSensorData < backLeftSensorData) {                             // if too close to the wall, and driving towards the wall, turn away from wall a bunch
        turnLeftSharp(ci_drive_speed, speedModifier);
      }
      if (frontLeftSensorData > backLeftSensorData) {                             // if too close to the wall, and driving away from wall, drive straight
        driveStraight(ci_drive_speed);
      }
    }
    ////////////////////////////////////////////////////////////////////////////////
    if (((frontLeftSensorData + backLeftSensorData) / 2) > desiredDistance) {     // if too far away from the wall
      if (frontLeftSensorData == backLeftSensorData) {                            // if too far away from the wall, and parallel to the wall, turn towards wall slightly
        turnRight(ci_drive_speed, speedModifier);
      }
      if (frontLeftSensorData < backLeftSensorData) {                             // if too far away from the wall, and angled towards the wall, drive straight
        driveStraight(ci_drive_speed);
      }
      if (frontLeftSensorData > backLeftSensorData) {                              // if too far from the wall, and heading away from the wall, turn towards the wall a bunch
        turnRightSharp(ci_drive_speed, speedModifier);
      }
    }
  }                              // end of   if (wallside=='l')
}//****************end of followWall****************end of followWall****************

void stopDrive() {
  right_motor.writeMicroseconds(1500);
  left_motor.writeMicroseconds(1500);
}
// sets both motors to the same speed, needs some control code to drive straight
void driveStraight(int ci_drive_speed) {
  right_motor.writeMicroseconds(ci_drive_speed);
  left_motor.writeMicroseconds(ci_drive_speed);
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



// call this to drive "straight" ahead to a new encoder value
// for example this will drive straight ahead at speed 1600 until both motors have incremented 1000 encoder ticks:
// driveStraightAheadEncoders(1600, 1000);
// 1000 encoder ticks makes for about 15.5" or 39.4cm
void driveStraightAheadEncoders(int ci_drive_speed, int encoderTicks) {
  encoder_rightMotor.zero();      // 0 both encoders
  encoder_leftMotor.zero();
  while ((encoder_rightMotor.getRawPosition() < encoderTicks) || (encoder_leftMotor.getRawPosition() < encoderTicks)) {      // drive ahead to encoder value
    right_motor.writeMicroseconds(ci_drive_speed);
    left_motor.writeMicroseconds(ci_drive_speed);
  }
  stopDrive();
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
}//****************end of skidsteerNinetyLeft****************end of skidsteerNinetyLeft****************


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
  const int ci_hall_effect_scanning_threshold_max = analogRead(ci_hall_effect_pin) + 7; //nothing~515, value goes up or down depending on field orientation
  const int ci_hall_effect_scanning_threshold_min = analogRead(ci_hall_effect_pin) - 7;
  Serial.println();
  Serial.print("ci_hall_effect_scanning_threshold_max: ");
  Serial.print(ci_hall_effect_scanning_threshold_max);
  Serial.print("    ci_hall_effect_scanning_threshold_min: ");
  Serial.print(ci_hall_effect_scanning_threshold_min);

  // move arm to scanning height
  moveArm(ci_arm_scanning_height);
  // move wrist to 90 degrees down
  sweepServo(servo_wrist, ci_wrist_scan);


  // move turntable from left to right while polling the hall effect sensor
  // store the greatest hall effect value and remmeber at which encoder angle it occurs
  // not too picky about overshooting the far right angle slightly due to arm momentum, inconsequential
  int tesseractReading = 0;
  int tesseractAngle = 0;
  bool tesseractDetected = false;

  while (encoder_turntable.getRawPosition() < ci_turntable_right) {
    turntable_motor.writeMicroseconds(1670);

    // if a tesseract is detected during the sweep
    int reading = analogRead(ci_hall_effect_pin);
    if ((reading < ci_hall_effect_scanning_threshold_min) || (reading > ci_hall_effect_scanning_threshold_max)) {
      tesseractDetected = true;
      tesseractReading = analogRead(ci_hall_effect_pin);
      tesseractAngle = encoder_turntable.getRawPosition();
      Serial.println();
      Serial.print("new tesseractReading: ");
      Serial.print(tesseractReading);
      Serial.print("    new tesseractAngle: ");
      Serial.print(tesseractAngle);
    }

    Serial.println();
    Serial.print("inside scanning while loop, encoder value: ");
    Serial.print(encoder_turntable.getRawPosition());
    Serial.print("     ci_hall_effect_reading: ");
    Serial.print(reading);
  }
  turntable_motor.writeMicroseconds(1500);    //stop turntable
  Serial.println();
  Serial.print("outside scanning while loop   tesseractReading:");
  Serial.print(tesseractReading);
  Serial.print("   tesseractAngle: ");
  Serial.print(tesseractAngle);

  // if a magnetic tesseract was found, pick it up
  if (tesseractDetected) {

    moveTurntable(tesseractAngle - 65); // move to best encoder value - offset, offset is due to the hall effect sensor being beside the arm magnet
    servo_magnet.write(ci_magnet_extend);         // extend magnet
    moveArm(ci_arm_carry_height);                 // raise arm
    sweepServo(servo_wrist, ci_wrist_carry);      // move wrist to carry
    moveTurntable(ci_turntable_center);           // center turntable
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
