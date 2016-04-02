#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
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


Servo left_motor;
Servo right_motor;
Servo arm_motor;
Servo turntable_motor;
Servo servo_wrist;
Servo servo_magnet;


I2CEncoder encoder_leftMotor;
I2CEncoder encoder_rightMotor;
I2CEncoder encoder_arm;
I2CEncoder encoder_turntable;



// servo and motor pins
const int ci_magnet_servo = 4;   // 157 retracted, 18 extended
const int ci_wrist_servo = 5;
const int ci_IR_crown = 7;  //High when no tesseract, low when tesseract
const int ci_right_motor = 8;
const int ci_left_motor = 9;
const int ci_arm_motor = 10;
const int ci_turntable_motor = 11;

// sensor pins
const int frontPingPin = 13;
const int frontRightPingPin = 12;
const int backRightPingPin = 6;
const int frontLeftPingPin = 2;
const int backLeftPingPin = 3;
const int ci_arm_linetracker = A2;
const int ci_hall_effect = A3;

// I2C pins
const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow

//*********************************************************************************************************//
//*********************************************************************************************************//
//*********************************************************************************************************//
//*********************************************************************************************************//

// global ints
int moveSpeed = 1500;
int counter = 0;


//*********************************************************************************************************//
//*********************************************************************************************************//
//*********************************************************************************************************//
//*********************************************************************************************************//

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

  pinMode(ci_wrist_servo, OUTPUT);
  servo_wrist.attach(ci_wrist_servo);

  pinMode(ci_magnet_servo, OUTPUT);
  servo_magnet.attach(ci_magnet_servo);

  pinMode(ci_IR_crown, INPUT);
  pinMode(ci_arm_linetracker, INPUT);
  pinMode(ci_hall_effect, INPUT);

  //************************************************************************

  // setup encoders. Must be initiliazed in the order that they are chained together,
  // starting with the encoder directly attached to the arduino
  delay(3000);      // give I2C circuits a chance to boot up
  encoder_leftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_leftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_rightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_rightMotor.setReversed(true);  // adjust for positive count when moving forward
  encoder_arm.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_arm.setReversed(false);  // adjust for positive count when turning clockwise
  encoder_turntable.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_turntable.setReversed(false);  // adjust for positive count when moving forward
  encoder_turntable.zero();       //Robot arm must be positioned on the standoffs to be properly zeroed
  encoder_arm.zero();
}//**********************end setup**********************end setup**********************end setup**********************end setup**********************//





void loop() {
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

Serial.println();
Serial.print(encoder_arm.getRawPosition());
Serial.print(encoder_turntable.getRawPosition());


}



