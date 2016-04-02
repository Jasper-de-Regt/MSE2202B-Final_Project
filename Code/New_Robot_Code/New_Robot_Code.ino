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
const int ci_hall_effect_scanning_threshold = 0;  // populate with a 10 bit value that would indicate a tesseract is near the arm hall effect sensor
// servo angle constants
const int ci_magnet_retract = 157;      //angle of servo_magnet with magnet in the "off" position
const int ci_magnet_extend = 18;        // angle of the servo_magnet with magnet in the "on" or "pickup" position
const int ci_wrist_down = 0;            // angle of the servo_wrist in the max down position
const int ci_wrist_straight_out = 0;    // angle of the servo_wrist when parallel with arm
// encoder value constants
const int ci_turntable_left = 0;        // encoder ticks of the turntable at far left position
const int ci_turntable_right = 0;       // encoder ticks of the turntable at far right position
const int ci_turntable_center = 0;      // encoder ticks of the turntable at center position (straight forward)
const int ci_arm_scanning_height = 0;   // encoder ticks with the arm at a height idesl for tesseract scanning/pickup
const int ci_arm_carry_height = 0;      // encoder ticks with the arm at height ideal for driving around and not blocking the front ping sensor
const int ci_arm_push_away_height = 0;  // encoder ticks with the arm dropped just above the ground, ready to push away bad tesseracts


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
<<<<<<< HEAD
I2CEncoder encoder_arm_motor;
I2CEncoder encoder_turntable_motor;

//port pin constants
//digital pins

const int frontPingPin = 13;
const int frontRightPingPin = 12;
const int backRightPingPin = 6;
const int frontLeftPingPin = 2;
const int backLeftPingPin = 3;
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
const int ci_wrist_position_vertical = 200;    //  " Wrist bar is perpendicular to the arm.
const int ci_wrist_position_Ddiagonal = 50;    //45 degree angle //D is for Downward
const int ci_wrist_position_Udiagonal = 150;    //45 degree angle //U is for Upward
const int ci_wrist_position_horizontal = 100;  //  " Wrist bar is parallel to the arm.
const int ci_wrist_modetwo_dropoff = 0;      //  "
const int ci_wrist_scanning = 25;

//Magnet servo positions
const int ci_magnet_up_position = 157; // Cant pickup tesseracts
const int ci_magnet_down_position = 18; //Will pickup tesseracts

const int ci_hall_effect_scanning_threshold = 20; //this is a minimum
=======
I2CEncoder encoder_arm;
I2CEncoder encoder_turntable;
// setup NewPing objects (ultrasonic distance sensors);
NewPing frontPingSensor(ci_front_ping_pin, ci_front_ping_pin, 200);
NewPing frontRightPingSensor(ci_front_right_ping_pin, ci_front_right_ping_pin, 200);
NewPing frontLeftPingSensor(ci_front_left_ping_pin, ci_front_left_ping_pin, 200);
NewPing backRightPingSensor(ci_back_right_ping_pin, ci_back_right_ping_pin, 200);
NewPing backLeftPingSensor(ci_back_left_ping_pin, ci_back_left_ping_pin, 200);

>>>>>>> refs/remotes/origin/JasperNewBranch

//*********************************************************************************************************//
//*********************************************************************************************************//
//*********************************************************************************************************//
//*********************************************************************************************************//


<<<<<<< HEAD
char ch_tracking_direction = 'R';                //  Character value is either 'R', 'L', 'l' or 'r'

unsigned int ui_num_turns = 0;

unsigned int moveSpeed=1500;
int counter=0;

const int ci_drive_speed = 1600;

int timeDifference = 20; //Set as global.

// setup NewPing objects
NewPing frontPingSensor(frontPingPin, frontPingPin, 200);
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

  pinMode(ci_little_magnet_servo, OUTPUT);
  servo_magnet_motor.attach(ci_little_magnet_servo);

  pinMode(ci_IR_crown, INPUT);
  pinMode(ci_arm_linetracker, INPUT);
  pinMode(ci_hall_effect, INPUT);

  //************************************************************************

  delay(3000);
=======

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
>>>>>>> refs/remotes/origin/JasperNewBranch
  // setup encoders. Must be initiliazed in the order that they are chained together,
  // starting with the encoder directly attached to the arduino
  delay(3000);      // give I2C circuits a chance to boot up before attempting to make connections, fixes yellow encoder light issue
  encoder_leftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_leftMotor.setReversed(false);   // adjust for positive count when moving forward
  encoder_rightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
<<<<<<< HEAD
  encoder_rightMotor.setReversed(true);  // adjust for positive count when moving forward

  encoder_arm_motor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_arm_motor.setReversed(true);  // adjust for positive count when turning clockwise

  encoder_turntable_motor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_turntable_motor.setReversed(false);  // adjust for positive count when moving forward
=======
  encoder_rightMotor.setReversed(true);   // adjust for positive count when moving forward
  encoder_arm.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_arm.setReversed(false);         // adjust for positive count when moving upwards
  encoder_turntable.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_turntable.setReversed(false);   // adjust for positive count when moving to the right
  encoder_turntable.zero();       // Robot arm must be positioned on the standoffs during powerup to be properly zeroed
  encoder_arm.zero();             // pretty sure encoders start out at 0 anyway and these calls are redundant
}//****************end setup****************end setup****************end setup****************end setup****************end setup****************
>>>>>>> refs/remotes/origin/JasperNewBranch


}



void loop() {
<<<<<<< HEAD
  Serial.print("Left: ");
  Serial.println(encoder_leftMotor.getRawPosition());
  Serial.print("Right: ");
  Serial.println(encoder_rightMotor.getRawPosition());
  /*
    Serial.print("Arm: ");
    Serial.println(encoder_arm_motor.getRawPosition());
    Serial.print("Turntable: ");
    Serial.println(encoder_turntable_motor.getRawPosition());
  */
}










//FUNCTIONS

/* Function List + How to Call
=======
  //arm_motor.write(130);
  /*
    Serial.println(encoder_turntable.getRawPosition());
    moveTurntable(800);
    Serial.println("BACK IN MAINNNNNNNNN");
    delay(2000);
    Serial.println(encoder_turntable.getRawPosition());
    moveTurntable(100);
    delay(2000);

    moveArm(0);
    delay(2000);
    moveArm(400);
    delay(2000);
  */
>>>>>>> refs/remotes/origin/JasperNewBranch

  printEncoderValues();


<<<<<<< HEAD
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

void magnetSweep(int final_Position) {
  //high is up, low is down
  if (servo_magnet_motor.read() > final_Position) {
    for (int i = servo_magnet_motor.read(); i > final_Position; i-- ) {
      servo_magnet_motor.write(i);
      delay(15);
    }
  }
  else {
    for (int i = servo_magnet_motor.read(); i < final_Position; i++) {
      servo_magnet_motor.write(i);
      delay(15);
    }
  }
}

void wristSweep(int final_Position) {
  //
  if (servo_wrist_motor.read() > final_Position) {
    for (int i = servo_wrist_motor.read(); i > final_Position; i-- ) {
      servo_wrist_motor.write(i);
      delay(15);
    }
  }
  else {
    for (int i = servo_wrist_motor.read(); i < final_Position; i++) {
      servo_wrist_motor.write(i);
      delay(15);
    }
  }
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
=======
>>>>>>> refs/remotes/origin/JasperNewBranch
}



<<<<<<< HEAD

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



void armEncoderPosition(int encoderPosition) { // on hold untold jasper adds P tuning
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



void tesseractScanSweep() {
  int tesseractLocation;
  int encoderMaxHallRead = 0;
  wristSweep(ci_wrist_scanning);
  //armEncoderPosition(ci_arm_horizontal_position);
  //still needs the arm height function
  for (int i = encoder_turntable_motor.getRawPosition(); i < ci_turntable_right_position; i + 30) { // not sure if this is the best way to scan
    if (analogRead(ci_hall_effect) > encoderMaxHallRead) {
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
  servo_magnet_motor.write(ci_magnet_up_position);
  // move arm up
  moveArm(ci_arm_diagonal_position);
  // wrist out?
  sweepServo(servo_wrist_motor, 180);
  // move turntable to far left position
  moveTurntable(ci_turntable_left_position);
  // move arm to scanning height----horizontal is scanning height+- 5/10 encoder ticks
  moveArm(ci_arm_horizontal_position);
  // move wrist to 90 degrees down
  sweepServo(servo_wrist_motor, ci_wrist_scanning);


  // move turntable from left to right while polling the hall effect sensor
  // store the greatest hall effect value and remmeber at which encoder angle it occurs
  // not too picky about overshooting the far right angle slightly due to arm momentum, inconsequential
  int greatestHallReading = 0;
  int greatestHallEncoderAngle = 0;
  while (encoder_rightMotor.getRawPosition() < ci_turntable_right_position) {
    turntable_motor.writeMicroseconds(1600);
    if (analogRead(ci_hall_effect) > greatestHallReading) {
      greatestHallReading = analogRead(ci_hall_effect);
      greatestHallEncoderAngle = encoder_rightMotor.getRawPosition();
    }
  }
  turntable_motor.writeMicroseconds(1500);

  // if a magnetic tesseract was found, pick it up
  if (greatestHallReading > ci_hall_effect_scanning_threshold) {
    // move to best encoder value - offset
    // the offset is due to the hall effect sensor being beside the arm magnet
    moveTurntable(greatestHallEncoderAngle - 0);
    // extend magnet
    servo_magnet_motor.write(ci_magnet_down_position);
    // raise arm
    moveArm(ci_arm_diagonal_position);
    // center turntable
    moveTurntable(ci_turntable_middle_position);
    return true;
  }

  // if a non-magnetic tesseract was found, toss it to the side?
  else {
    // drop arm
    moveArm(ci_arm_horizontal_position);
    // sweep to far left
    moveTurntable(ci_turntable_left_position);
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
  int tolerance = 10;         // deadband tolerance, what +/- encoder value is close enough to be considered good enough?
  bool stayInFunction = true; // this is a blocking function, stay in this function until this bool is false

  while (stayInFunction == true) {
    // clip speeds to max speeds
    if (moveSpeed > 1600) {
      moveSpeed = 1600;
    }
    else if (moveSpeed < 1400) {
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
    else if ((encoder_turntable_motor.getRawPosition() < (desiredPosition + tolerance)) && (encoder_turntable_motor.getRawPosition() > (desiredPosition - tolerance))) {
      turntable_motor.writeMicroseconds(1500);
      counter++;
      delay(3);
      stayInFunction = true;
    }
    // if it needs to move to the right
    else if (encoder_turntable_motor.getRawPosition() < desiredPosition) {
      moveSpeed++;
      turntable_motor.writeMicroseconds(moveSpeed);
      counter = 0;
      stayInFunction = true;
    }
    //else if it needs to move to the left
    else if (encoder_turntable_motor.getRawPosition() > desiredPosition) {
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
      moveSpeed = 1600;
    }
    else if ((moveSpeed < 1370) && (encoder_arm_motor.getRawPosition() > 400)) {
      moveSpeed = 1370;
    }
    else if ((moveSpeed < 1450) && (encoder_arm_motor.getRawPosition() < 200)) {
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
    else if ((encoder_arm_motor.getRawPosition() < (desiredPosition + tolerance)) && (encoder_arm_motor.getRawPosition() > (desiredPosition - tolerance))) {
      arm_motor.writeMicroseconds(1500);
      counter++;
      delay(3);
      stayInFunction = true;
    }
    // if it needs to move to the right
    else if (encoder_arm_motor.getRawPosition() < desiredPosition) {
      moveSpeed++;
      arm_motor.writeMicroseconds(moveSpeed);
      counter = 0;
      stayInFunction = true;
    }
    //else if it needs to move to the left
    else if (encoder_arm_motor.getRawPosition() > desiredPosition) {
      moveSpeed--;
      arm_motor.writeMicroseconds(moveSpeed);
      counter = 0;
      stayInFunction = true;
    }
  }
  counter = 0;
}//****************end moveArm fn****************end moveArm fn****************
=======
>>>>>>> refs/remotes/origin/JasperNewBranch
