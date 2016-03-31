#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_ArmMotor;    
Servo servo_GripMotor;

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;
I2CEncoder encoder_TurntableMotor;
I2CEncoder encoder_ArmMotor;


const int ci_TurnTable_Motor=?;
const int ci_Arm_Motor=?;

const int ci_Turntable_Left_Position=?;
const int ci_Turntable_Middle_Position=?;
const int ci_Turntable_Right_Position=?;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
