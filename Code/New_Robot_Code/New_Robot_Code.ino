#include <Servo.h>
#include <EEPROM.h>
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





//****************************************************************
//************************GLOBAL VARIABLES************************
// servo angle constants
const int ci_magnet_retract = 120;        //angle of servo_magnet with magnet in the "off" position
const int ci_magnet_extend = 10;          // angle of the servo_magnet with magnet in the "on" or "pickup" position
const int ci_wrist_scan = 10;             // angle of the servo_wrist in the max down position
const int ci_wrist_parallel = 98;         // angle of the servo_wrist when parallel with arm
const int ci_wrist_carry = 0;             // 0 is the correct value
const int ci_wrist_push_away = 35;        // wrist position to push away bad tesseracts
// encoder value constants
const int ci_turntable_left = 380;          // encoder ticks of the turntable at far left position
const int ci_turntable_right = 1280;        // encoder ticks of the turntable at far right position
const int ci_turntable_center = 800;        // encoder ticks of the turntable at center position (straight forward)
const int ci_arm_scanning_height = 20;       // encoder ticks with the arm at a height ideal for tesseract scanning/pickup
const int ci_arm_carry_height = 220;          // encoder ticks with the arm at height ideal for driving around and not blocking the front ping sensor
const int ci_arm_push_away_height = -46;      // encoder ticks with the arm dropped just above the ground, ready to push away bad tesseracts
// variables used in P control drive straight functions
unsigned long lastDriveStraightUpdateTime = 0;    //updates with las time motor speeds were adjusted in P control drive straight functions
unsigned int leftSpeedDriveStraight = 0;          // left speed for P control drive straight functions
long encoderTracker = 0;                          // trscks how many encoder ticks were travelled in P control drive straight functions
// variables used in P position control arm and turntable
int moveSpeed = 1500;   //used in move arm and move turntable P position control functions
int counter = 0;      //used in move arm and move turntable P position control functions

//******************************************************************
//************************PORT PIN CONSTANTS************************
// servo and motor pin constants
const int ci_magnet_servo_pin = 4;      // 157 retracted, 18 extended
const int ci_wrist_servo_pin = 5;
const int ci_right_motor_pin = 8;
const int ci_left_motor_pin = 9;
const int ci_arm_motor_pin = 10;
const int ci_turntable_motor_pin = 11;
// sensor pin constants, ping is synomous with ultrasonic
const int ci_front_ping_pin = 13;
const int ci_front_right_ping_pin = 12;
const int ci_front_left_ping_pin = 2;
const int ci_back_right_ping_pin = 6;
const int ci_back_left_ping_pin = 3;
const int ci_arm_linetracker_pin = A2;
const int ci_hall_effect_pin = A3;
const int ci_IR_crown_pin = 7;          //High when no tesseract, low when tesseract
// I2C pin constants. Don't connect these pins to anything else
const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow


//*******************************************************************
//************************OBJECT DECLARATIONS************************
// declare all servo objects
Servo left_motor;
Servo right_motor;
Servo arm_motor;
Servo turntable_motor;
Servo servo_wrist;
Servo servo_magnet;
// declare all encoder objects
I2CEncoder encoder_leftMotor;
I2CEncoder encoder_rightMotor;
I2CEncoder encoder_arm;
I2CEncoder encoder_turntable;
// setup NewPing objects (ultrasonic distance sensors);
NewPing frontPingSensor(ci_front_ping_pin, ci_front_ping_pin, 200);
NewPing frontRightPingSensor(ci_front_right_ping_pin, ci_front_right_ping_pin, 200);
NewPing frontLeftPingSensor(ci_front_left_ping_pin, ci_front_left_ping_pin, 200);
NewPing backRightPingSensor(ci_back_right_ping_pin, ci_back_right_ping_pin, 200);
NewPing backLeftPingSensor(ci_back_left_ping_pin, ci_back_left_ping_pin, 200);


//*********************************************************************************************************//
//*********************************************************************************************************//
//*********************************************************************************************************//
//*********************************************************************************************************//



void setup() {
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);  // pour a bowl of serial, a special type of soup

  //*******************************************************************
  //************************PIN SETUPS*********************************
  pinMode(ci_left_motor_pin, OUTPUT);
  left_motor.attach(ci_left_motor_pin);
  pinMode(ci_right_motor_pin, OUTPUT);
  right_motor.attach(ci_right_motor_pin);
  pinMode(ci_arm_motor_pin, OUTPUT);
  arm_motor.attach(ci_arm_motor_pin);
  pinMode(ci_turntable_motor_pin, OUTPUT);
  turntable_motor.attach(ci_turntable_motor_pin);
  pinMode(ci_wrist_servo_pin, OUTPUT);
  servo_wrist.attach(ci_wrist_servo_pin);
  pinMode(ci_magnet_servo_pin, OUTPUT);
  servo_magnet.attach(ci_magnet_servo_pin);
  // sensor setups
  pinMode(ci_IR_crown_pin, INPUT);
  pinMode(ci_arm_linetracker_pin, INPUT);
  pinMode(ci_hall_effect_pin, INPUT);


  //*********************************************************************
  //************************ENCODER SETUP*********************************
  // setup encoders. Must be initiliazed in the order that they are chained together,
  // starting with the encoder directly attached to the arduino
  delay(3000);      // give I2C circuits a chance to boot up before attempting to make connections, fixes yellow encoder light issue
  encoder_leftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_leftMotor.setReversed(false);   // adjust for positive count when moving forward
  encoder_rightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_rightMotor.setReversed(true);   // adjust for positive count when moving forward
  encoder_arm.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_arm.setReversed(false);         // adjust for positive count when moving upwards
  encoder_turntable.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_turntable.setReversed(false);   // adjust for positive count when moving to the right
  encoder_turntable.zero();       // Robot arm must be positioned on the standoffs during powerup to be properly zeroed
  encoder_arm.zero();             // pretty sure encoders start out at 0 anyway and these calls are redundant
  Serial.println("setup has completed");
}//****************end setup****************end setup****************end setup****************end setup****************end setup****************

char wall = 'l';  // which side the wall is on when calling wall follow
int dis = 15;     // distance to drive from wall
int mode = 1;     // switchcase variable
bool found = false; // has a tesseract been found
const int encodercm = 38.9;   //31.75encoder ticks per cm constant, derived experimentally
int mySpeed = 1630;
//#define maincode;

// garbage variables
int temps;
bool once = true;



void loop() {
  //printPingSensorReadings();
  // start with wall on left side
  // follow wall close by and look for tesseracts
  // if not found, continue
  // if found, go home
  //  followWall(int ci_drive_speed, char wallSide, int desiredDistance);

  // while not detecting a tesseract
  // 25.381 encoder ticks/cm;
  //followWall(mySpeed, 'r', 20);
  Serial.println("starting 20");
  moveArmSweep(20);
  Serial.println("done 20");
  delay(3000);
  Serial.println("starting 220");
  moveArmSweep(220);
  Serial.println("done 220");
  delay(3000);
  Serial.println("starting 650");
  moveArmSweep(650);
  Serial.println("done 650");
  delay(3000);
  Serial.println("starting 420");
  moveArmSweep(420);
  Serial.println("done 420");
  delay(3000);


#ifdef maincode
  switch (mode) {
    case 1:   // driveCase // needs work
      //follow wall looking for tesseract
      Serial.println("mode 1");
      Serial.println();
      Serial.print("   distabnce: ");
      Serial.print(dis);
      Serial.print("     wall: ");
      Serial.print(wall);
      followWall(mySpeed, wall, dis);

      // if a tesseract is found, jump to the arm scanning case
      if (!digitalRead(ci_IR_crown_pin)) {
        mode = 2;
      }

      // if you approach the end wall, goto turn around and follow wall again case
      temps = (frontPingSensor.convert_cm(frontPingSensor.ping_median(10)));   // averages 10 measurements, returns in cm
      Serial.println(temps);
      if (temps < 10) {
        Serial.println("hitting temps<5");
        mode = 3;
      }
      break;    // end switch case 1




    case 2:   //tesseractScan
      // scan for magnetic tesseract, if one is found its picked up and returns true, bad ones are shoved aside and return false
      Serial.println("mode 2");
      found = tesseractArmScan();

      // if bad tesseract was found, continue following wall looking for tesseract
      if (!found) {
        mode = 1;
      }
      // if the tesseract was found, special magic to return home and dropoff
      else if (found) {
        mode = 4;
      }
      break;    // end switch case 2



    case 3:   // when wall following and you approach an end wall, this case will turn the robot, move it further from wall
      Serial.println("mode 3");

      // if the wall is currently on the left side
      if (wall == 'l') {
        // skid 90 right, don't need to scan for tesseracts because the robot is round and pivots in place
        skidsteerNinetyRight(mySpeed);
        // drive forward appropriate amount while scanning for tesseracts
        found = driveStraightAheadEncoders(mySpeed, encodercm * 5);

        if (!found) {                       // if no tesseract was found during the short forward move
          skidsteerNinetyRight(mySpeed);    // turn right again (which finishes the 180)
          wall = 'r';                       // update wall
          dis += 5;                         // update distance
          mode = 1;                         // return to driveCase
        }

        // else if a magnetic tesseract was found during the short fwd move, perpendicular to main wall
        else if (found) {
          //empty, special magic to pickup and return home
        }
      }// end of if wall == 'l'


      // if the wall is currently on the right side
      else if (wall == 'r') {
        // skid 90 left, don't need to scan for tesseracts because the robot is round and pivots in place
        skidsteerNinetyLeft(mySpeed);
        // drive forward appropriate amount while scanning for tesseracts
        found = driveStraightAheadEncoders(mySpeed, encodercm * 5);

        if (!found) {                       // if no tesseract was found during the short forward move
          skidsteerNinetyLeft(mySpeed);    // turn left again (which finishes the 180)
          wall = 'l';                       // update wall
          dis += 5;                         // update distance
          mode = 1;                         // return to driveCase
        }
        if (found) {
          //empty, special magic to pickup and return home
        }
      }// end of if wall == 'r'

      break;    // end switch case 3


    case 4:   // carrying a tesseract -> go home
      Serial.println("mode 4");
      // if the main wall is on the right side, turn towards main wall
      if (wall = 'r') {
        skidsteerNinetyRight(mySpeed);
      }
      // else if the main wall is on the left side, turn towards main wall
      else if (wall = 'l') {
        // skidsteer towards wall
        skidsteerNinetyLeft(mySpeed);
      }

      // the robot is not somewhere in the arena, facing the main wall

      // drive straight up to main wall until reaching it
      while ((frontPingSensor.convert_cm(frontPingSensor.ping_median(10))) > 15) {
        driveStraight(mySpeed);
      }
      stopDrive();

      // skidsteer towards home
      skidsteerNinetyLeft(mySpeed);

      // follow close to the wall, heading towards start corner, until reaching the corner
      wall = 'r'; // main wall on right side
      dis = 15;   // reset distance so drives close to wall
      while ((frontPingSensor.convert_cm(frontPingSensor.ping_median(10))) > 15) {
        followWall(mySpeed, wall, dis);
      }
      stopDrive();
      //robot is now in the start corner, facing into the corner

      // goto putting the tesseract down between the tape marks code
      mode = 6;
      break;    // end switch case 4


  }//end of switch case curly bracket
#endif
}//end loop curly bracket



// this function sweeps the arm using the regular move arm function
void moveArmSweep(int desiredPosition) {
  int startPosition = encoder_arm.getRawPosition();
  int error = desiredPosition - startPosition;
  int iterations = error / 10;
  iterations = abs(iterations);

  if (error > 0) {   // arm needs to move up
    for (int i = 0; i < iterations; i++) {
      moveArm(startPosition + 10 * i);
    }
  }
  else if ( error < 0) { //arm needs to move down
    for (int i = 0; i < iterations; i++) {
      moveArm(startPosition - 10 * i);
    }
  }
}





void parallel() {

  //if the wall is on the left, use left ping sensors
  if (wall = 'l') {

    int error = (frontLeftPingSensor.ping_cm() - backLeftPingSensor.ping_cm());

    if (error > 0) { // while error is greater than 0, turn towards wall
      while ((frontLeftPingSensor.ping_cm() - backLeftPingSensor.ping_cm()) > 0) {
        left_motor.writeMicroseconds(3000 - mySpeed);
        right_motor.writeMicroseconds(mySpeed);
      }
    }
    else if (error < 0) { // while error is less than 0, turn towards wall
      while ((frontLeftPingSensor.ping_cm() - backLeftPingSensor.ping_cm()) < 0) {
        left_motor.writeMicroseconds(mySpeed);
        right_motor.writeMicroseconds(3000 - mySpeed);
      }
    }


  }


  stopDrive();
}



void parallelLeft(int ci_drive_speed) {
  // if it has been awhile since this function was called, update leftSpeed with the passed speed value and reset encoderTracker
  int desiredPosition = -439;   // 439 encoder ticks of each motor is a 90 degree turn
  encoder_leftMotor.zero();    // zero encoders
  encoder_rightMotor.zero();



  // assuming wall is on right side
  // spin wheel motors until

  // if the front sensor







  // while turned away from wall, turn closer to wall (right side fwd, left side rev)
  if ((frontLeftPingSensor.ping_cm() - backLeftPingSensor.ping_cm()) > 0) {
    leftSpeedDriveStraight = 3000 - ci_drive_speed;
    encoderTracker = 0;
    while ((frontLeftPingSensor.ping_cm() - backLeftPingSensor.ping_cm()) > 0) {
      // the left motor speed is updated every 20mS in this loop
      if ((millis() - lastDriveStraightUpdateTime) > 20) {
        int error = encoder_leftMotor.getRawPosition() + encoder_rightMotor.getRawPosition();  // error is the difference in .getRawPositions()
        if (error < 0) {        // if the left motor went too far, slow it down, closer to 1500
          leftSpeedDriveStraight += 15;
        }
        else if (error > 0) {       // else if the left motor didnt go far enough, speed it up, further from 1500, smaller
          leftSpeedDriveStraight -= 15;
        }
        leftSpeedDriveStraight = constrain(leftSpeedDriveStraight, 1000, 1500);   // constrain leftSpeedDriveStraight to values possible to send to servo
        left_motor.writeMicroseconds(leftSpeedDriveStraight);        // set leftSpeedDriveStraight
        right_motor.writeMicroseconds(ci_drive_speed);  // the right motor constantly runs at the passed speed
        encoderTracker += encoder_leftMotor.getRawPosition();  // tracks how far the encoder has moved
        encoder_leftMotor.zero();    // zero encoders to prevent overflow errors
        encoder_rightMotor.zero();
        lastDriveStraightUpdateTime = millis();          // update last time the speeds were updated
      }
    }
  }

  // while turned towards wall, turn away from wall (right side rev, left side fwd)
  if ((frontLeftPingSensor.ping_cm() - backLeftPingSensor.ping_cm()) < 0) {
    leftSpeedDriveStraight = ci_drive_speed;
    encoderTracker = 0;
    while ((frontLeftPingSensor.ping_cm() - backLeftPingSensor.ping_cm()) < 0) {
      // the left motor speed is updated every 20mS in this loop
      if ((millis() - lastDriveStraightUpdateTime) > 20) {
        int error = encoder_leftMotor.getRawPosition() + encoder_rightMotor.getRawPosition();  // error is the difference in .getRawPositions()
        if (error > 0) {        // if the left motor went too far, slow it down, closer to 1500
          leftSpeedDriveStraight -= 15;
        }
        else if (error < 0) {       // else if the left motor didnt go far enough, speed it up, further from 1500, smaller
          leftSpeedDriveStraight += 15;
        }
        leftSpeedDriveStraight = constrain(leftSpeedDriveStraight, 1500, 2000);   // constrain leftSpeedDriveStraight to values possible to send to servo
        left_motor.writeMicroseconds(leftSpeedDriveStraight);        // set leftSpeedDriveStraight
        right_motor.writeMicroseconds(3000 - ci_drive_speed); // the right motor constantly runs at the passed speed
        encoderTracker += encoder_leftMotor.getRawPosition();  // tracks how far the encoder has moved
        encoder_leftMotor.zero();    // zero encoders to prevent overflow errors
        encoder_rightMotor.zero();
        lastDriveStraightUpdateTime = millis();          // update last time the speeds were updated
      }
    }
  }




  stopDrive();            // stop motors when call has finished
  encoderTracker = 0;     // logically this is redundant as its reset at the start of the call
}











