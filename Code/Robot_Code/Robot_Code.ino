#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>


Servo servo_turntable_motor;
Servo servo_elbow_motor;
Servo servo_wrist_motor;
Servo servo_magnet_motor;


I2CEncoder encoder_turntable_motor;
I2CEncoder encoder_arm_motor;

#define DEBUG_ARM_turntable_ENCODERS

//port pin constants

const int ci_turntable_motor = 2;
const int ci_arm_motor = 3;
const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow

//Position constants
const int ci_turntable_left_position = 400;     // Experiment to determine appropriate value
const int ci_turntable_middle_position = 980;    //  "
const int ci_turntable_right_position = 1540;    //  "
const int ci_arm_vertical_position = 0;         //  "
const int ci_arm_extended_position = 400;        //  "

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

  encoder_turntable_motor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_turntable_motor.setReversed(false);  // adjust for positive count when moving forward
  encoder_arm_motor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_arm_motor.setReversed(true);  // adjust for positive count when turning clockwise

  encoder_turntable_motor.zero();
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
}


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}


void moveToPosn(Servo serv0, int initial, int final, bool hardStartTrue) {

  int current = millis(); //current time
  int previous = current;

  //Checks to make sure the servo is attached
  /*if (serv0.attached() == false) {
  //  serv0.attach(servoPin1);
  }*/

  //If the position must start at the initial mark
  if (hardStartTrue == true) {
    Serial.println("Hard Start");
    while (serv0.read() != initial) {
      if ((current - previous) >= timeDiff) {
        serv0.write(initial);
        previous = current;
      }

    }
  }

  while (serv0.read() != final) {


    if ((current - previous) >= timeDiff) {
      if (serv0.read() > final) {
        serv0.write(serv0.read() - 1);
      }
      if (serv0.read() < final) {
        serv0.write(serv0.read() + 1);
      }
      previous = current;
    }


    current = millis();
  }


}
