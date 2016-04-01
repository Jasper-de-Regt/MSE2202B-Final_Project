#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include "robot_functions.h" //this is our header file

/*
Naming conventions:

*camelcase for function
- E.g. itLooksLikeThis

*significant words in the title should be separated by underscores "_" **for variables
- E.g. motor_pin_one

*CAPITALIZE ACRONYMS
- E.g. IR instead of infrared.
       LED instead of light_emitting_diode (variable) or lightEmittingDiode (function)

*/


Servo servo_turntable_motor;
Servo servo_arm_motor;
Servo servo_wrist_motor;
Servo servo_magnet_motor;

I2CEncoder encoder_rightMotor;
I2CEncoder encoder_leftMotor;
I2CEncoder encoder_turntable_motor;
I2CEncoder encoder_arm_motor;

#define DEBUG_ARM_turntable_ENCODERS

//port pin constants

const int ci_turntable_motor = 10;
const int ci_arm_motor = 11;
const int ci_hall_effect = 6;
const int ci_right_motor = 8;
const int ci_left_motor = 9;
const int ci_wrist_servo = 5;
const int ci_IR_crown = 7;  //High when no tesseract, low when tesseract
const int ci_magnet_servo = 4;
const int ci_arm_linetracker = 3;
const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow

//Position constants
const int ci_turntable_default_position = 0;     //  Experiment to determine appropriate value
const int ci_turntable_left_position = 400;      //  "
const int ci_turntable_middle_position = 980;    //  "
const int ci_turntable_right_position = 1540;    //  "
const int ci_arm_vertical_position = 0;          //  "
const int ci_arm_half_position = 200;            //  "
const int ci_arm_horizontal_position = 400;      //  "
const int ci_arm_wall_line_scan = 0;             //  "
const int ci_wrist_wall_line_scan = ;            //  "
const int ci_arm_modetwo_dropoff = ;             //  "
const int ci_wrist_modetwo_dropoff = ;           //  "
const int ci_arm_wall_tesseract_scan = ;         //  "
const int ci_wrist_wall_tesseract_scan = ;       //  "
const int ci_wrist_position_perpendicular = ;    //  " Wrist bar is perpendicular to the arm.
const int ci_wrist_position_parallel = ;         //  " Wrist bar is parallel to the arm.
const double cd_robot_diameter = 23.42;            //  Radius of the device ~ 23.42 mm

boolean bt_IRcrown_detection = ;                 //  "

long l_turntable_motor_position;
long l_arm_motor_position;

unsigned int ui_num_turns = 0;

const int ci_drive_speed = 1600;

void setup() {
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);

  // set up arm motor and turntable motor
  pinMode(ci_turntable_motor, OUTPUT);
  servo_turntable_motor.attach(ci_turntable_motor);
  pinMode(ci_arm_motor, OUTPUT);
  servo_arm_motor.attach(ci_arm_motor);

  // setup encoders. Must be initiliazed in the order that they are chained together,
  // starting with the encoder directly attached to the arduino
  encoder_leftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_leftMotor.setReversed(false);  // adjust for positive count when moving forward

  encoder_rightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_rightMotor.setReversed(true);  // adjust for positive count when moving forward

  encoder_turntable_motor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_turntable_motor.setReversed(false);  // adjust for positive count when moving forward

  encoder_arm_motor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_arm_motor.setReversed(true);  // adjust for positive count when turning clockwise

  encoder_turntable_motor.zero();       //Robot arm must be positioned on the standoffs to be properly zeroed
  encoder_arm_motor.zero();
}

void loop() {
#ifdef DEBUG_ARM_turntable_ENCODERS
  l_turntable_motor_position = encoder_turntable_motor.getRawPosition();
  l_arm_motor_position = encoder_arm_motor.getRawPosition();

  Serial.print("Encoders turntable: ");
  Serial.print(l_turntable_motor_position );
  Serial.print(", Arm: ");
  Serial.println(l_arm_motor_position );
#endif

  <<< <<< < HEAD
  /***************************************************************************************************************************/
  /*                                               MODE 1                                                                    */
  /***************************************************************************************************************************/


  /*
  Logic

  Calibrated (sensors, servos)? If no, calibrate. If yes, proceed.

  Drive
    If the bot hasn't turned 180 yet, drive straight just using encoders
    Else If the bot has turned more than 0 180 degree turns, follow wall -> set distance as (1/3) diameter + (num turns - 1)*diameter
      If numTurns % 2 == 1 (odd)
        use the right ultrasonic sensors
      Else If numTurns % 2 == 0 (even)
        use the left ultrasonic sensors

  If a wall has been detected in front of the device
    Turn 90 (alternating right and left) -> keep a counter
    Then turn another 90 degrees (same direction)
    Resume driving

  If the IR sensor has been tripped,
    stop driving
    Drive backward 2" (130 encoder ticks)
    stop driving
    Move the arm to the left side
    Lower the wrist
    Sweep the arm from side to side while reading from the Hall Effect sensor

    If a Hall Effect sensor detects a large enough change, record the encoder position of the turntable servo motor
      move the turntable servo motor back to that position

    If after 1 sweep, the Hall Effect sensor doesn't detect a change
      Move the turntable servo to the left position
      Lower the arm servo (a.k.a. "elbow") so that the end effector is low to the ground
      Move the turntable servo to the right position (to sweep the bad tesseract out of the way)
    ELSE If the Hall Effect sensor detects a large enough change
      Lower the magnet servo to lower the magnet -> picking up the tesseract
      Raise the wrist servo so the end effector support bar is parallel to the ground
      Drive back to the origin //Need to fully define logic how to do that
      Rotate the arm to the side of the device (logic determines which side to move to)
      Drive forward slowly (toward the corner of the arena)
      Scan with the IR sensor on the arm

      If the analog reading is defined in the range of an empty space (after reading 1 tape value) *calibrated values
        stop driving
        Raise the arm (elbow) ~30 degrees
        Lower the wrist so it hangs vertically
        Raise the magnet servo to raise the magnet -> releases servo
        Raise the wrist so the IR sensor is parallel with the ground
        Drive back to the origin (reverse drive?)

  //NEW CYCLE



  */


  if (ui_num_turns == 0) { //If the robot has yet to turn
    driveStraightAheadEncoders(ci_drive_speed, 7736); //Encoder tickers corresponds to 10 ft. May get interrupted by a sensor getting tripped or some other condition.
  }
  else if (ui_num_turns > 0) {
    if ((ui_num_turns % 2) == 1) {
    followWall(ci_drive_speed, 'R' , ((cd_robot_diameter / 3) + cd_robot_diameter* (ui_num_turns - 1)) ); //Sets the bot to follow the wall on the right hand side 
    }
    else if ((ui_num_turns % 2) == 0) {
      followWall(ci_drive_speed, 'L' , ((cd_robot_diameter / 3) + cd_robot_diameter* (ui_num_turns - 1)) ); //Sets the bot to follow the wall on the left hand side
    }
      
    //If proximity to wall is <= 7 cm AND the wrist bar is parallel to the ground
    //The ultrasonic sensor isn't reliable below 7 cm
    if ((ui_front_distance_reading <= 7) && (servo_wrist_motor.read() >= ci_wrist_parallel)) {
      skidsteerNinetyRight(ci_drive_speed);
      skidsteerNinetyRight(ci_drive_speed);
    }
  }

  //assuming the driving code is above

  if (bt_IRcrown_detection == true) { // if tesseract detected
    stopDrive();
    driveStraightAheadEncoders(1400, 130); // back up 2in
    armEncoderPosition(ci_arm_half_position); // raise arm to an angle of 45 degrees
    turnTurntableEncodersPosition(ci_turntable_left_position); // move turntable arm to the leftmost extremity

  }

  else continue;
}



