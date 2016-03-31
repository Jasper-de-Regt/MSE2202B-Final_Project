#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include "robot_functions.h" //this is our header file

/*
Naming conventions:

*camelcase
- E.g. itLooksLikeThis

*significant words in the title should be separated by underscores "_"
- E.g. motor_pin_one

*CAPITALIZE ACRONYMS
- E.g. IR instead of infrared.
       LED instead of light_emitting_diode or lightEmittingDiode

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
const int ci_IR_crown=7;    //High when no tesseract, low when tesseract
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

boolean bt_IRcrown_detection = ;                 //  "

long l_turntable_motor_position;
long l_arm_motor_position;

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

//assuming the driving code is above

if(bt_IRcrown_detection==true){  // if tesseract detected
  stopDrive();
driveStraightAheadEncoders(1400,130); // back up 2in
armEncoderPosition(ci_arm_half_position); // raise arm to an angle of 45 degrees
<<<<<<< HEAD
turnTurntableEncodersPosition(ci_turntable_left_position); // move turntable arm to the leftmost extremity
=======
turnTurntableEncodersPosition(ci_turntable_left_position); // move turntable arm to the leftmost extremity 
>>>>>>> refs/remotes/origin/Michael-Henderson-branch

}

else continue;
}



