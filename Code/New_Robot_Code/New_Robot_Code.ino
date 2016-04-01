#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>


//port pin constants
//digital pins
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

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
