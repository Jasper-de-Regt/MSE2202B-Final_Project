#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>


Servo servo_TurntableMotor;
Servo servo_ArmMotor;


I2CEncoder encoder_TurntableMotor;
I2CEncoder encoder_ArmMotor;

#define DEBUG_ARM_TURNTABLE_ENCODERS

//port pin constants

const int ci_TurnTable_Motor = 2;
const int ci_Arm_Motor = 3;
const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow

//Position constants
const int ci_Turntable_Left_Position = 400;     // Experiment to determine appropriate value
const int ci_Turntable_Middle_Position = 980;    //  "
const int ci_Turntable_Right_Position = 1540;    //  "
const int ci_Arm_Vertical_Position = 0;         //  "
const int ci_Arm_Extended_Position = 400;        //  "

long l_Turntable_Motor_Position;
long l_Arm_Motor_Position;

void setup() {
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);

  // set up arm motor and turntable motor
  pinMode(ci_TurnTable_Motor, OUTPUT);
  servo_TurntableMotor.attach(ci_TurnTable_Motor);
  pinMode(ci_Arm_Motor, OUTPUT);
  servo_ArmMotor.attach(ci_Arm_Motor);

  encoder_TurntableMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_TurntableMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_ArmMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_ArmMotor.setReversed(true);  // adjust for positive count when turning clockwise

  encoder_TurntableMotor.zero();
  encoder_ArmMotor.zero();
}

void loop() {
#ifdef DEBUG_ARM_TURNTABLE_ENCODERS
  l_Turntable_Motor_Position = encoder_TurntableMotor.getRawPosition();
  l_Arm_Motor_Position = encoder_ArmMotor.getRawPosition();

  Serial.print("Encoders Turntable: ");
  Serial.print(l_Turntable_Motor_Position );
  Serial.print(", Arm: ");
  Serial.println(l_Arm_Motor_Position );
#endif
}
