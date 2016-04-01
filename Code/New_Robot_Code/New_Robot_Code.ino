#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

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

Servo servo_left_motor;
Servo servo_right_motor;
Servo servo_turntable_motor;
Servo servo_arm_motor;
Servo servo_wrist_motor;
Servo servo_magnet_motor;

I2CEncoder encoder_leftMotor;
I2CEncoder encoder_rightMotor;
I2CEncoder encoder_turntable_motor;
I2CEncoder encoder_arm_motor;

//port pin constants
//digital pins
const int ci_little_magnet_servo = 4;
const int ci_big_wrist_servo = 5;
const int ci_IR_crown = 7;  //High when no tesseract, low when tesseract
const int ci_right_motor = 8;
const int ci_left_motor = 9;
const int ci_arm_motor = 10;
const int ci_turntable_motor = 11 ;
const in

//ff =13;
//fr=12;
//br=6;
//fl=2;
//bl=3;



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
const double cd_robot_diameter = 23.42;          //  Radius of the device ~ 23.42 mm
char ch_tracking_direction = 'R';                //  Character value is either 'R', 'L', 'l' or 'r'
unsigned int ui_num_turns = 0;
const int ci_drive_speed = 1600;

void setup() {
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);



  // Pin Setup LRTA
  //************************************************************************

  pinMode(ci_left_motor, OUTPUT);
  servo_left_motor.attach(ci_left_motor);

  pinMode(ci_right_motor, OUTPUT);
  servo_right_motor.attach(ci_right_motor);

  pinMode(ci_turntable_motor, OUTPUT);
  servo_turntable_motor.attach(ci_turntable_motor);

  pinMode(ci_arm_motor, OUTPUT);
  servo_arm_motor.attach(ci_arm_motor);

  //************************************************************************

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
  // put your main code here, to run repeatedly:

}
