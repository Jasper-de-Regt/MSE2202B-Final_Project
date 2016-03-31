#include <NewPing.h>
#include <Servo.h>
#include <Wire.h>
#include <I2CEncoder.h>


// port pin constants
const int rightServoPin = 8;
const int leftServoPin = 9;

// ultrasonic pins
const int frontRightPingPin = 7;
const int backRightPingPin = 4;
const int frontLeftPingPin = 11;
const int backLeftPingPin = 12;

// I2C pins
const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow


// setup drive motors as servo objects
Servo servo_rightMotor;
Servo servo_leftMotor;

// setup encoder objects
I2CEncoder encoder_rightMotor;
I2CEncoder encoder_leftMotor;

// setup NewPing objects
NewPing frontRightPingSensor(frontRightPingPin, frontRightPingPin, 200);
NewPing backRightPingSensor(backRightPingPin, backRightPingPin, 200);
NewPing frontLeftPingSensor(frontLeftPingPin, frontLeftPingPin, 200);
NewPing backLeftPingSensor(backLeftPingPin, backLeftPingPin, 200);


//test variable
int once = 0;





//*********************************************************************************************************************************************************************************************************************************************************************//
//*********************************************************************************************************************************************************************************************************************************************************************//

void setup() {
  // set up drive motors
  Wire.begin();   //Wire needed for I2CEncoder library, kind of like a Serial.begin(9600);
  Serial.begin(9600);

  // setup wheel motors
  pinMode(rightServoPin, OUTPUT);
  pinMode(leftServoPin, OUTPUT);
  servo_rightMotor.attach(rightServoPin);
  servo_leftMotor.attach(leftServoPin);


  // setup encoders. Must be initiliazed in the order that they are chained together,
  // starting with the encoder directly attached to the arduino
  encoder_leftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_leftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_rightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_rightMotor.setReversed(true);  // adjust for positive count when moving forward


}

//*********************************************************************************************************************************************************************************************************************************************************************//
//*********************************************************************************************************************************************************************************************************************************************************************//
//*********************************************************************************************************************************************************************************************************************************************************************//
//*********************************************************************************************************************************************************************************************************************************************************************//
void loop() {

  //followWall('r', 20, 1600);



  if (once == 0) {
    followWall(1600, 'R', 15);
    if (encoder_rightMotor.getRawPosition() > 2000) {
    once = 1;
  }
  }

  

  if (once == 1) {
    moveFurtherFromWall(1600, 'R');
    driveStraightAheadEncoders(1600,1000);

    once++;
  }
if (once==2){
      followWall(1600, 'L', 23);
}


}


//*********************************************************************************************************************************************************************************************************************************************************************//
//*********************************************************************************************************************************************************************************************************************************************************************//
//*********************************************************************************************************************************************************************************************************************************************************************//
//*********************************************************************************************************************************************************************************************************************************************************************//






void moveFurtherFromWall(int driveSpeed, char wallSide) {
  if ((wallSide == 'R') || (wallSide == 'r')) { // if wall is on right
    skidsteerNinetyLeft(driveSpeed);            // turn 90 left
    driveStraightAheadEncoders(1600, 203);      // drive head ~8cm
    skidsteerNinetyLeft(driveSpeed);            // turn 90 left again
  }
  if ((wallSide == 'L') || (wallSide == 'l')) { // if wall is on right
    skidsteerNinetyLeft(driveSpeed);            // turn 90 left
    driveStraightAheadEncoders(1600, 203);      // drive head ~8cm
    skidsteerNinetyLeft(driveSpeed);            // turn 90 left again
  }
}


// call this to do a 90 degree pivot to the right
// for example, this will pivot right 90 degrees at speed 1600. skidsteerNinetyRight(1600);
void skidsteerNinetyRight(int driveSpeed) {
  encoder_rightMotor.zero();      // 0 both encoders
  encoder_leftMotor.zero();
  while ((encoder_rightMotor.getRawPosition() > -439) || (encoder_leftMotor.getRawPosition() < 439)) {      // turn to 90 degree encoder value, 900 encoder ticks makes for a 180
    servo_rightMotor.writeMicroseconds(3000 - driveSpeed);
    servo_leftMotor.writeMicroseconds(driveSpeed);
  }
  stopDrive();
}
// call this to do a 90 degree pivot to the left
// for example, this will pivot left 90 degrees at speed 1600. skidsteerNinetyLeft(1600);
void skidsteerNinetyLeft(int driveSpeed) {
  encoder_rightMotor.zero();      // 0 both encoders
  encoder_leftMotor.zero();
  while ((encoder_rightMotor.getRawPosition() < 439) || (encoder_leftMotor.getRawPosition() > -439)) {     // turn to 90 degree encoder value, 900 encoder ticks makes for a 180
    servo_rightMotor.writeMicroseconds(driveSpeed);
    servo_leftMotor.writeMicroseconds(3000 - driveSpeed);
  }
  stopDrive();
}

// call this to drive "straight" ahead to a new encoder value
// for example this will drive straight ahead at speed 1600 until both motors have incremented 1000 encdoer ticks. driveStraightAheadEncoders(1600, 1000);
// 1000 encoder ticks makes for about 15.5" or 39.4cm
void driveStraightAheadEncoders(int driveSpeed, int encoderTicks) {
  encoder_rightMotor.zero();      // 0 both encoders
  encoder_leftMotor.zero();
  while ((encoder_rightMotor.getRawPosition() < encoderTicks) || (encoder_leftMotor.getRawPosition() < encoderTicks)) {      // drive ahead to encoder value
    servo_rightMotor.writeMicroseconds(driveSpeed);
    servo_leftMotor.writeMicroseconds(driveSpeed);
  }
  stopDrive();
}




















// call this function to follow a wall. Example followWall(R, 15, 1600) will follow a wall on the right side, maintaining a distance of 15cm, at a speed of 1600
void followWall(int driveSpeed, char wallSide, int desiredDistance) {

  int speedModifier = (driveSpeed - 1500) / 3;    // how much to modify the speed for turns
  if ((driveSpeed - speedModifier * 1.5) < 1500) {    // ensure the motor wouldnt run in reverse
    speedModifier = (driveSpeed - 1500) / 1.5;
  }

  int frontLeftSensorData = frontLeftPingSensor.ping_cm();
  int backLeftSensorData = backLeftPingSensor.ping_cm();

  // if the wall is on the right side
  if ((wallSide == 'R') || (wallSide == 'r')) {
    int frontRightSensorData = frontRightPingSensor.ping_cm();                      //populate int with sensor data (in centimeters)
    int backRightSensorData = backRightPingSensor.ping_cm();
    ////////////////////////////////////////////////////////////////////////////////
    if (((frontRightSensorData + backRightSensorData) / 2) == desiredDistance) {    // if the correct distance away from wall
      if (frontRightSensorData == backRightSensorData) {                            // if correct distance from wall, and driving parallel, drive straight
        driveStraight(driveSpeed);
      }
      if (frontRightSensorData < backRightSensorData) {                             // if correct distance from wall, and driving towards wall, turn away from wall a little
        turnLeft(driveSpeed, speedModifier);
      }
      if (frontRightSensorData > backRightSensorData) {                             // if correct distance from wall, and driving away from wall, turn towards wall a little
        turnRight(driveSpeed, speedModifier);
      }
    }
    ////////////////////////////////////////////////////////////////////////////////
    if (((frontRightSensorData + backRightSensorData) / 2) < desiredDistance) {     // if too close to the wall
      if (frontRightSensorData == backRightSensorData) {                            // if too close to the wall, and driving parallel to wall, turn away from wall a little
        turnLeft(driveSpeed, speedModifier);
      }
      if (frontRightSensorData < backRightSensorData) {                             // if too close to the wall, and driving towards the wall, turn away from wall a bunch
        turnLeftSharp(driveSpeed, speedModifier);
      }
      if (frontRightSensorData > backRightSensorData) {                             // if too close to the wall, and driving away from wall, drive straight
        driveStraight(driveSpeed);
      }
    }
    ////////////////////////////////////////////////////////////////////////////////
    if (((frontRightSensorData + backRightSensorData) / 2) > desiredDistance) {     // if too far away from the wall
      if (frontRightSensorData == backRightSensorData) {                            // if too far away from the wall, and parallel to the wall, turn towards wall slightly
        turnRight(driveSpeed, speedModifier);
      }
      if (frontRightSensorData < backRightSensorData) {                             // if too far away from the wall, and angled towards the wall, drive straight
        driveStraight(driveSpeed);
      }
      if (frontRightSensorData > backRightSensorData) {                              // if too far from the wall, and heading away from the wall, turn towards the wall a bunch
        turnRightSharp(driveSpeed, speedModifier);
      }
    }
  }                                                             // end of   if (wallside=='r')
  //*************************************************************************************************************************************************************************************//
  // if the wall is on the left side
  if ((wallSide == 'L') || (wallSide == 'l')) {
    int frontLeftSensorData = frontLeftPingSensor.ping_cm();                      //populate int with sensor data (in centimeters)
    int backLeftSensorData = backLeftPingSensor.ping_cm();
    ////////////////////////////////////////////////////////////////////////////////
    if (((frontLeftSensorData + backLeftSensorData) / 2) == desiredDistance) {    // if the correct distance away from wall
      if (frontLeftSensorData == backLeftSensorData) {                            // if correct distance from wall, and driving parallel, drive straight
        driveStraight(driveSpeed);
      }
      if (frontLeftSensorData < backLeftSensorData) {                             // if correct distance from wall, and driving towards wall, turn away from wall a little
        turnRight(driveSpeed, speedModifier);
      }
      if (frontLeftSensorData > backLeftSensorData) {                             // if correct distance from wall, and driving away from wall, turn towards wall a little
        turnLeft(driveSpeed, speedModifier);
      }
    }
    ////////////////////////////////////////////////////////////////////////////////
    if (((frontLeftSensorData + backLeftSensorData) / 2) < desiredDistance) {     // if too close to the wall
      if (frontLeftSensorData == backLeftSensorData) {                            // if too close to the wall, and driving parallel to wall, turn away from wall a little
        turnRight(driveSpeed, speedModifier);
      }
      if (frontLeftSensorData < backLeftSensorData) {                             // if too close to the wall, and driving towards the wall, turn away from wall a bunch
        turnRightSharp(driveSpeed, speedModifier);
      }
      if (frontLeftSensorData > backLeftSensorData) {                             // if too close to the wall, and driving away from wall, drive straight
        driveStraight(driveSpeed);
      }
    }
    ////////////////////////////////////////////////////////////////////////////////
    if (((frontLeftSensorData + backLeftSensorData) / 2) > desiredDistance) {     // if too far away from the wall
      if (frontLeftSensorData == backLeftSensorData) {                            // if too far away from the wall, and parallel to the wall, turn towards wall slightly
        turnLeft(driveSpeed, speedModifier);
      }
      if (frontLeftSensorData < backLeftSensorData) {                             // if too far away from the wall, and angled towards the wall, drive straight
        driveStraight(driveSpeed);
      }
      if (frontLeftSensorData > backLeftSensorData) {                              // if too far from the wall, and heading away from the wall, turn towards the wall a bunch
        turnLeftSharp(driveSpeed, speedModifier);
      }
    }
  }                                                             // end of   if (wallside=='l')
}



// some mini functions, mainly used in followWall()
void stopDrive() {
  servo_rightMotor.writeMicroseconds(1500);
  servo_leftMotor.writeMicroseconds(1500);
}
void driveStraight(int driveSpeed) {
  servo_rightMotor.writeMicroseconds(driveSpeed);
  servo_leftMotor.writeMicroseconds(driveSpeed);
}
void turnRight(int driveSpeed, int speedModifier) {
  servo_rightMotor.writeMicroseconds(driveSpeed - speedModifier);
  servo_leftMotor.writeMicroseconds(driveSpeed + speedModifier);
}
void turnRightSharp(int driveSpeed, int speedModifier) {
  servo_rightMotor.writeMicroseconds(driveSpeed - speedModifier * 1.5);
  servo_leftMotor.writeMicroseconds(driveSpeed + speedModifier * 1.5);
}
void turnLeft(int driveSpeed, int speedModifier) {
  servo_rightMotor.writeMicroseconds(driveSpeed + speedModifier);
  servo_leftMotor.writeMicroseconds(driveSpeed - speedModifier);
}
void turnLeftSharp(int driveSpeed, int speedModifier) {
  servo_rightMotor.writeMicroseconds(driveSpeed + speedModifier * 1.5);
  servo_leftMotor.writeMicroseconds(driveSpeed - speedModifier * 1.5);
}




















