#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include <NewPing.h>
/*
  Naming conventions:

  camelcase
  - E.g. itLooksLikeThis

  significant words in the title should be separated by underscores "_"
  - E.g. motor_pin_one

  CAPITALIZE ACRONYMS
  - E.g. IR instead of infrared.
       LED instead of light_emitting_diode or lightEmittingDiode

*/

Servo left_motor;
Servo right_motor;
Servo arm_motor;
Servo turntable_motor;
Servo servo_wrist_motor;
Servo servo_magnet_motor;

I2CEncoder encoder_leftMotor;
I2CEncoder encoder_rightMotor;
I2CEncoder encoder_arm_motor;
I2CEncoder encoder_turntable_motor;

//port pin constants
//digital pins

const int frontRightPingPin = 1; // pins still need to be decided
const int backRightPingPin = 2; //
const int frontLeftPingPin = 3; //
const int backLeftPingPin = 6; //
const int ci_little_magnet_servo = 4;
const int ci_big_wrist_servo = 5;
const int ci_IR_crown = 7;  //High when no tesseract, low when tesseract
const int ci_right_motor = 8;
const int ci_left_motor = 9;
const int ci_arm_motor = 10;
const int ci_turntable_motor = 11 ;

//analog pins
const int ci_arm_linetracker = A2;
const int ci_hall_effect = A3;
const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow

//Position constants-------Set to 0 if unknown at the moment
//Turntable positions
const int ci_turntable_default_position = 0;     //  Experiment to determine appropriate value
const int ci_turntable_left_position = 400;
const int ci_turntable_middle_position = 980;
const int ci_turntable_right_position = 1540;

//Arm positions
const int ci_arm_vertical_position = 400;
const int ci_arm_diagonal_position = 200;
const int ci_arm_horizontal_position = 0;
const int ci_arm_modetwo_dropoff = 0;

//Wrist positions
const int ci_wrist_position_vertical = 0;    //  " Wrist bar is perpendicular to the arm.
const int ci_wrist_position_diagonal = 0;    //45 degree angle
const int ci_wrist_position_horizontal = 0;  //  " Wrist bar is parallel to the arm.
const int ci_wrist_modetwo_dropoff = 0;      //  "

//Magnet servo positions
const int ci_magnet_up_position = 0; // Cant pickup tesseracts
const int ci_magnet_down_position = 0; //Will pickup tesseracts

boolean bt_IRcrown_detection;                 //  " logic is backwards ie. false is positive and true is negative
//bt_IRcrown_detection=digitalRead(ci_IR_crown); //I think

//for driving
const float cd_robot_diameter = 23.42;          //  Radius of the device ~ 23.42 mm

char ch_tracking_direction = 'R';                //  Character value is either 'R', 'L', 'l' or 'r'

unsigned int ui_num_turns = 0;

const int ci_drive_speed = 1600;

int timeDifference = 20; //Set as global.

// setup NewPing objects
NewPing frontRightPingSensor(frontRightPingPin, frontRightPingPin, 200);
NewPing backRightPingSensor(backRightPingPin, backRightPingPin, 200);
NewPing frontLeftPingSensor(frontLeftPingPin, frontLeftPingPin, 200);
NewPing backLeftPingSensor(backLeftPingPin, backLeftPingPin, 200);

void setup() {
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);



  // Pin Setup
  //************************************************************************

  pinMode(ci_left_motor, OUTPUT);
  left_motor.attach(ci_left_motor);

  pinMode(ci_right_motor, OUTPUT);
  right_motor.attach(ci_right_motor);

  pinMode(ci_arm_motor, OUTPUT);
  arm_motor.attach(ci_arm_motor);

  pinMode(ci_turntable_motor, OUTPUT);
  turntable_motor.attach(ci_turntable_motor);

  pinMode(ci_big_wrist_servo, OUTPUT);
  servo_wrist_motor.attach(ci_big_wrist_servo);


  //************************************************************************

  // setup encoders. Must be initiliazed in the order that they are chained together,
  // starting with the encoder directly attached to the arduino
  encoder_leftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_leftMotor.setReversed(false);  // adjust for positive count when moving forward

  encoder_rightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_rightMotor.setReversed(true);  // adjust for positive count when moving forward

  encoder_arm_motor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_arm_motor.setReversed(true);  // adjust for positive count when turning clockwise

  encoder_turntable_motor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_turntable_motor.setReversed(false);  // adjust for positive count when moving forward

  encoder_turntable_motor.zero();       //Robot arm must be positioned on the standoffs to be properly zeroed
  encoder_arm_motor.zero();
  delay(1000);
}










void loop() {

}










//FUNCTIONS

/* Function List + How to Call

  Arm Servo functions
  void moveToPosn(servo, servo pin, initial posn, final posn, bool to start at initial);

  Driving functions
  void stopDrive(); //Stops the motors
  void driveStraight(int ci_drive_speed); //Sets both motors at the same speed
  void driveStraightAheadEncoders(int ci_drive_speed, int encoderTicks); //ci_drive_speed is a constant, encoder ticks corresponds to the distance you want to travel. **1000 encoder ticks makes for about 15.5" or 39.4cm

  Wall-Following functions
  void followWall(int ci_drive_speed, char wallSide, int desiredDistance); // call to follow a wall. Example followWall(1600, R, 15) will follow a wall on the right side, maintaining a distance of 15cm, at a speed of 1600
  void moveFurtherFromWall(int ci_drive_speed, char wallSide); //wallSide should be either R or L (capital or lowercase)

  Turning functions
  void turnRight(int ci_drive_speed, int speedModifier); //ci_drive_speed is const. speedModifier is added/subtracted from left/right respectively.
  void turnRightSharp(int ci_drive_speed, int speedModifier); //similar to turnRight(), but speedModifier is multiplied by 1.5
  void turnLeft(int ci_drive_speed, int speedModifier); //ci_drive_speed is const. speedModifier is added/subtracted from right/left respectively.
  void turnLeftSharp(int ci_drive_speed, int speedModifier); //similar to turnLeft(), but speedModifier is multiplied by 1.5
  void skidsteerNinetyRight(int ci_drive_speed); // ci_drive_speed will be a constant as defined in the main code
  void skidsteerNinetyLeft(int ci_drive_speed) // Same note as above

*/

// some mini functions, mainly used in followWall()

void stopDrive() {
  right_motor.writeMicroseconds(1500);
  left_motor.writeMicroseconds(1500);
}
void stopTurntable() {
  turntable_motor.writeMicroseconds(1500);
}
void stopArm() {
  arm_motor.writeMicroseconds(1500);
}
void driveStraight(int ci_drive_speed) {
  right_motor.writeMicroseconds(ci_drive_speed);
  left_motor.writeMicroseconds(ci_drive_speed);
}
void turnRight(int ci_drive_speed, int speedModifier) {
  right_motor.writeMicroseconds(ci_drive_speed - speedModifier);
  left_motor.writeMicroseconds(ci_drive_speed + speedModifier);
}
void turnRightSharp(int ci_drive_speed, int speedModifier) {
  right_motor.writeMicroseconds(ci_drive_speed - speedModifier * 1.5);
  left_motor.writeMicroseconds(ci_drive_speed + speedModifier * 1.5);
}
void turnLeft(int ci_drive_speed, int speedModifier) {
  right_motor.writeMicroseconds(ci_drive_speed + speedModifier);
  left_motor.writeMicroseconds(ci_drive_speed - speedModifier);
}
void turnLeftSharp(int ci_drive_speed, int speedModifier) {
  right_motor.writeMicroseconds(ci_drive_speed + speedModifier * 1.5);
  left_motor.writeMicroseconds(ci_drive_speed - speedModifier * 1.5);
}

// call this to drive "straight" ahead to a new encoder value
// for example this will drive straight ahead at speed 1600 until both motors have incremented 1000 encdoer ticks. driveStraightAheadEncoders(1600, 1000);
// 1000 encoder ticks makes for about 15.5" or 39.4cm



void driveStraightAheadEncoders(int ci_drive_speed, int encoderTicks) {
  encoder_rightMotor.zero();      // 0 both encoders
  encoder_leftMotor.zero();
  while ((encoder_rightMotor.getRawPosition() < encoderTicks) || (encoder_leftMotor.getRawPosition() < encoderTicks)) {      // drive ahead to encoder value
    right_motor.writeMicroseconds(ci_drive_speed);
    left_motor.writeMicroseconds(ci_drive_speed);
  }
  stopDrive();
}

// call this to do a 90 degree pivot to the left
// for example, this will pivot left 90 degrees at speed 1600. skidsteerNinetyLeft(1600);



void skidsteerNinetyLeft(int ci_drive_speed) {
  encoder_rightMotor.zero();      // 0 both encoders
  encoder_leftMotor.zero();
  while ((encoder_rightMotor.getRawPosition() < 439) || (encoder_leftMotor.getRawPosition() > -439)) {     // turn to 90 degree encoder value, 900 encoder ticks makes for a 180
    right_motor.writeMicroseconds(ci_drive_speed);
    left_motor.writeMicroseconds(3000 - ci_drive_speed);
  }
  stopDrive();
}
// call this to do a 90 degree pivot to the right
// for example, this will pivot right 90 degrees at speed 1600. skidsteerNinetyRight(1600);



void skidsteerNinetyRight(int ci_drive_speed) {
  encoder_rightMotor.zero();      // 0 both encoders
  encoder_leftMotor.zero();
  while ((encoder_rightMotor.getRawPosition() > -439) || (encoder_leftMotor.getRawPosition() < 439)) {      // turn to 90 degree encoder value, 900 encoder ticks makes for a 180
    right_motor.writeMicroseconds(3000 - ci_drive_speed);
    left_motor.writeMicroseconds(ci_drive_speed);
  }
  stopDrive();
}


void moveFurtherFromWall(int ci_drive_speed, char wallSide) {
  if ((wallSide == 'R') || (wallSide == 'r')) { // if wall is on right
    skidsteerNinetyLeft(ci_drive_speed);            // turn 90 left
    driveStraightAheadEncoders(1600, 203);      // drive head ~8cm
    skidsteerNinetyLeft(ci_drive_speed);            // turn 90 left again
  }
  if ((wallSide == 'L') || (wallSide == 'l')) { // if wall is on right
    skidsteerNinetyLeft(ci_drive_speed);            // turn 90 left
    driveStraightAheadEncoders(1600, 203);      // drive head ~8cm
    skidsteerNinetyLeft(ci_drive_speed);            // turn 90 left again
  }
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

// scans for fluctuating magnetic field to see if there is a magnetic tesseract, return true if true



void tesseractScanSweep(int maxPosition) {
  for (int i = encoder_turntable_motor.getRawPosition(); i < maxPosition; i + 30) { // not sure if this is the best way to scan
    turnTurntableEncodersPosition(i);
    if (analogRead(ci_hall_effect)) {     //checks to see if there is an abnormality in the hall effects analog read, if yes then break the loop
      break;
    }
    armEncoderPosition(ci_arm_diagonal_position);
  }
}

/*void badTesseractSweep(){
  for (int i = encoder_turntable_motor.getRawPosition(); i < maxPosition; i + 30) { // not sure if this is the best way to scan
    turnTurntableEncodersPosition(i);
    if (analogRead(ci_hall_effect)) {     //checks to see if there is an abnormality in the hall effects analog read, if yes then break the loop
      break;
    }
    armEncoderPosition(ci_arm_diagonal_position);
  }
  }
*/


void servoMoveToPosition(Servo serv0, int final_Position) {
  long previous = millis();
  while (serv0.read() != final_Position) {
    if ((millis() - previous) >= timeDifference) {
      if (serv0.read() > final_Position) {
        serv0.write(serv0.read() - 1);
      }
      if (serv0.read() < final_Position) {
        serv0.write(serv0.read() + 1);
      }
      previous = millis();
    }
  }
}


// call this function to follow a wall. Example followWall(R, 15, 1600) will follow a wall on the right side, maintaining a distance of 15cm, at a speed of 1600
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
  }                                                             // end of   if (wallside=='r')
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
  }                                                             // end of   if (wallside=='l')
}

