#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_TurntableMotor;
Servo servo_ArmMotor;

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;
I2CEncoder encoder_TurntableMotor;
I2CEncoder encoder_ArmMotor;

//port pin constants
const int ci_Right_Motor = ?;
const int ci_Left_Motor = ?;
const int ci_TurnTable_Motor = ?;
const int ci_Arm_Motor = ?;

const int ci_Turntable_Left_Position = ?;
const int ci_Turntable_Middle_Position = ?;
const int ci_Turntable_Right_Position = ?;
const int ci_Arm_Retracted_Position = ?;
const int ci_Arm_Extended_Position = ?;

void setup() {
  // put your setup code here, to run once:
  // set up drive motors
  pinMode(ci_RightMotor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);

  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);

  // set up arm motors
  pinMode(ci_TurnTable_Motor, OUTPUT);
  servo_TurntableMotor.attach(ci_TurnTable_Motor);

  pinMode(ci_Arm_Motor, OUTPUT);
  servo_ArmMotor.attach(ci_Arm_Motor);

  encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward
  encoder_TurntableMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_TurntableMotor.setReversed(false);  // adjust for positive count when moving forward 
  encoder_ArmMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_ArmMotor.setReversed(true);  // adjust for positive count when turning clockwise
}

void loop() {
  // put your main code here, to run repeatedly:

}
