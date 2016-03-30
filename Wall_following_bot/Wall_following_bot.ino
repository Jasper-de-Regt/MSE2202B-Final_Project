#include <NewPing.h>
#include <Servo.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_rightMotor;
Servo servo_leftMotor;

// port pin constants
const int rightServoPin = 8;
const int leftServoPin = 9;
const int frontRightPingPin = 7;
const int backRightPingPin = 4;

int myspeed = 1600;
int setpoint = 100;

NewPing frontRightPingSensor(frontRightPingPin, frontRightPingPin, 200);
NewPing backRightPingSensor(backRightPingPin, backRightPingPin, 200);







//*********************************************************************************************************************************************************************************************************************************************************************//
//*********************************************************************************************************************************************************************************************************************************************************************//

void setup() {
  // set up drive motors
  pinMode(rightServoPin, OUTPUT);
  pinMode(leftServoPin, OUTPUT);
  servo_rightMotor.attach(rightServoPin);
  servo_leftMotor.attach(leftServoPin);

  Serial.begin(9600);

}

//*********************************************************************************************************************************************************************************************************************************************************************//
//*********************************************************************************************************************************************************************************************************************************************************************//

void loop() {




  int speedModifier = (myspeed-1500)/3;
  int  frontRightSensorData = frontRightPingSensor.ping_cm();
  int backRightSensorData = backRightPingSensor.ping_cm();

  // if the correct distance away
  if ((frontRightSensorData + backRightSensorData) / 2 == setpoint) {
    // if straight
    if (frontRightSensorData == backRightSensorData) {
      //drive straight
      servo_rightMotor.writeMicroseconds(myspeed);
      servo_leftMotor.writeMicroseconds(myspeed);
    }
    //if angled towards wall
    if (frontRightSensorData < backRightSensorData) {
      //turn away a little
      servo_rightMotor.writeMicroseconds(myspeed + speedModifier);
      servo_leftMotor.writeMicroseconds(myspeed - speedModifier);
    }
    //if angled away
    if (frontRightSensorData > backRightSensorData) {
      // turn towards
      servo_rightMotor.writeMicroseconds(myspeed - speedModifier);
      servo_leftMotor.writeMicroseconds(myspeed + speedModifier);
    }
  }

  //if too close
  if ((frontRightSensorData + backRightSensorData) / 2 < setpoint) {
    // if parallel
    if (frontRightSensorData == backRightSensorData) {
      //parallel and too close, turn away slightly
      servo_rightMotor.writeMicroseconds(myspeed + speedModifier);
      servo_leftMotor.writeMicroseconds(myspeed - speedModifier);
    }
    //if angled towards
    if (frontRightSensorData < backRightSensorData) {
      //you are too close and heading closer, turn away more
      servo_rightMotor.writeMicroseconds(myspeed + speedModifier);
      servo_leftMotor.writeMicroseconds(myspeed - speedModifier);
    }
    // if angled away
    if (frontRightSensorData > backRightSensorData) {
      // thats good because your too close, drive straight
      servo_rightMotor.writeMicroseconds(myspeed);
      servo_leftMotor.writeMicroseconds(myspeed);
    }
  }


  //if too far
  if ((frontRightSensorData + backRightSensorData) / 2 > setpoint) {
    // if parallel
    if (frontRightSensorData == backRightSensorData) {
      //parallel and too far, turn closer slightly
      servo_rightMotor.writeMicroseconds(myspeed - speedModifier);
      servo_leftMotor.writeMicroseconds(myspeed + speedModifier);
    }
    //if angled towards
    if (frontRightSensorData < backRightSensorData) {
      //you are too far and heading closer, go straight
      servo_rightMotor.writeMicroseconds(myspeed);
      servo_leftMotor.writeMicroseconds(myspeed);
    }
    // if angled away
    if (frontRightSensorData > backRightSensorData) {
      // too far and heading further away, turn in a bunch
      servo_rightMotor.writeMicroseconds(myspeed - speedModifier);
      servo_leftMotor.writeMicroseconds(myspeed + speedModifier);
    }
  }






}
