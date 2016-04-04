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
int moveSpeed = 1500;
int counter = 0;
// servo angle constants
const int ci_magnet_retract = 120;        //angle of servo_magnet with magnet in the "off" position
const int ci_magnet_extend = 10;          // angle of the servo_magnet with magnet in the "on" or "pickup" position
const int ci_wrist_scan = 20;             // angle of the servo_wrist in the max down position
const int ci_wrist_parallel = 98;         // angle of the servo_wrist when parallel with arm
const int ci_wrist_carry = 0;             // 0 is the correct value
const int ci_wrist_push_away = 35;        // wrist position to push away bad tesseracts
// encoder value constants
const int ci_turntable_left = 0;          // encoder ticks of the turntable at far left position
const int ci_turntable_right = 1280;        // encoder ticks of the turntable at far right position
const int ci_turntable_center = 800;        // encoder ticks of the turntable at center position (straight forward)
const int ci_arm_scanning_height = 60;       // encoder ticks with the arm at a height ideal for tesseract scanning/pickup
const int ci_arm_carry_height = 220;          // encoder ticks with the arm at height ideal for driving around and not blocking the front ping sensor
const int ci_arm_push_away_height = -46;      // encoder ticks with the arm dropped just above the ground, ready to push away bad tesseracts


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



bool runOnce = true;


const int ci_left_gate_distance = 5;//values range from 3-5
const long ci_mode2_backup_after_pickup = 2000; //needs value
const int ci_mode2_backup = 90; //needs value
const int ci_mode2_drive_placement1 = 800;// 242
const int ci_mode2_drive_placement2 = 650;//160
const int ci_mode2_drive_placement3 = 650;//160
const int ci_mode2_movement_backward = 774; //needs value
const int ci_mode2_movement_forward = -774; //needs value
const int ci_restart_backup_distance = 550;//needs value
const int ci_mode2_wall_distance = 8;//values range from 5-8 mostly 6 and 7
const int ci_mode2_scan_distance = 6; //ranges 7-8

const int ci_arm_mode2_scaninng_height = 620; //needs val6ue
const int ci_arm_vertical_position = 420;//needs value
//const int ci_arm_horizontal_position = 0;//replace with drive through
const int ci_arm_mode2_tesseract_dropoff = 250; //needs value
const int ci_arm_mode2_tesseract_dropoff_bump = 300; //needs value
const int ci_arm_mode2_pickup_height = 675; //needs value
const int ci_arm_drive_through = 800;//787

const int ci_wrist_mode2_scanning_height = 180; //zero is the correct value
const int ci_wrist_vertical_position = 20;//needs value
const int ci_wrist_mode2_tesseract_dropoff = 127; //needs value

const int ci_turntable_mode2_placement1 = 800; //needs value
const int ci_turntable_mode2_placement2 = 1200; //needs value
const int ci_turntable_mode2_placement3 = 360; //needs value
int tesseract_counter = 0;

long encoder_rightMotor_raw = 0;
long encoder_leftMotor_raw = 0;

void loop() {
  // put your main code here, to run repeatedly:
  //PREPARATION
  /*set turntable motor to middle value*/
  //servo_turntable_motor.write(ci_turntable_left_position);
  moveTurntable(ci_turntable_center);
  /*set servo elbow value and wrist value to just over top
   * of the wall to find where there is a tessaract*/
  delay(200);
  moveArm(ci_arm_drive_through);

  sweepServo(servo_wrist, ci_wrist_parallel);
  /*get a good idea of where we are*/
  int frontLeftSensorData = frontLeftPingSensor.ping_cm();
  int backLeftSensorData = backLeftPingSensor.ping_cm();
  int frontRightSensorData = frontRightPingSensor.ping_cm();
  int backRightSensorData = backRightPingSensor.ping_cm();

  if (frontRightSensorData == backRightSensorData )
  {
    while (backLeftSensorData > ci_left_gate_distance || (backLeftSensorData == 0)) {
      driveStraightReverse(1400);
      backLeftSensorData = backLeftPingSensor.ping_cm();
    }
    stopDrive();
  }
  else {
    //backRightSensorData = backRightPingSensor.ping_cm();
    while (backRightSensorData != frontRightSensorData) {
      if (frontRightSensorData > backRightSensorData) {
        left_motor.writeMicroseconds(1600);
        frontRightSensorData = frontRightPingSensor.ping_cm();
        break;
      }
      if (frontRightSensorData < backRightSensorData) {
        left_motor.writeMicroseconds(1400);
        frontRightSensorData = frontRightPingSensor.ping_cm();
        break;
      }
    }
    stopDrive();
    backLeftSensorData = backLeftPingSensor.ping_cm();
    while (backLeftSensorData > ci_left_gate_distance || (backLeftSensorData == 0)) {
      driveStraightReverse(1400);
      backLeftSensorData = backLeftPingSensor.ping_cm();
    }
    stopDrive();
  }
  driveStraightAheadEncoders(1600, ci_restart_backup_distance);
  /*set arm, wrist, and turntable to mode2 tessaract pick up position*/
  //already in position
  /*set magnet motor to up position*/

  sweepServo(servo_magnet, ci_magnet_retract);
  sweepServo(servo_wrist, ci_wrist_parallel);
  moveTurntable(ci_turntable_left);

  //DETECTING

  //GET READY FOR SCANNING
  /*set turntable motor to right value*/


  /*set servo elbow value and wrist value to just over top
   * of the wall to find where there is a tessaract*/

  moveArm(ci_arm_mode2_scaninng_height);
  delay(20);
  sweepServo(servo_wrist, ci_wrist_mode2_scanning_height);

  bool magnet_found = false ;
  //encoder_leftMotor.zero();    // zero encoders to prevent overflow errors
  //encoder_rightMotor.zero();
  int hall_effect_raw;
  int hall_effect_raw2;
  int hall_effect_raw3;

  while (magnet_found == false) {

    driveStraightAheadEncoders(1600, 1000);
    Serial.println("why");
    stopDrive();
    hall_effect_raw = analogRead(3);
    delay(30);
    hall_effect_raw2 = analogRead(3);
    delay(30);
    hall_effect_raw3 = analogRead(3);
    hall_effect_raw = (hall_effect_raw + hall_effect_raw2 + hall_effect_raw3) / 3;
    /*calibrate hall effect sensor*/
    int hall_effect_min = hall_effect_raw - 14;
    int hall_effect_max = hall_effect_raw + 14;
    backRightSensorData = backRightPingSensor.ping_cm();
    delay(500);
    while ((backRightSensorData < (ci_mode2_wall_distance + 20)) && (magnet_found == false)) {
      Serial.println("in the loop");
      hall_effect_raw = analogRead(3);
      backRightSensorData = backRightPingSensor.ping_cm();
      followWallReverse(1400, 'R', ci_mode2_wall_distance);
      Serial.println(hall_effect_raw);
      if  ((hall_effect_min > hall_effect_raw) || (hall_effect_raw > hall_effect_max))
        magnet_found = true;
      Serial.println(magnet_found);
    }
    //driveStraightAheadEncoders(1600, 100);
    //parallel();
    stopDrive();


  }
  Serial.println("im out of everything");



  /*once a tesseract is detected stop robot. stop reading hall effect sensor */
  stopDrive();

  //PICKUP
  /*back up 2inches lower magnet to pick up position still using wall follower code*/
  encoder_leftMotor.zero();    // zero encoders to prevent overflow errors
  encoder_rightMotor.zero();
  driveStraightReverseEncoders(1400, -ci_mode2_backup);

  sweepServo(servo_magnet, ci_magnet_extend);
  delay(200);

  moveArm(ci_arm_mode2_pickup_height);
  Serial.println("pickup");
  delay(200);
  //mov);eArmSweep(30
  moveArmSweep(ci_arm_mode2_tesseract_dropoff);
  /*if we hit the gate during testing this has to be in*/
  encoder_leftMotor.zero();    // zero encoders to prevent overflow errors
  encoder_rightMotor.zero();


  //driveStraightAheadEncoders(1600,);
  /*after 3 inches driving forward raise arm, and rotate turntatble motor to middle*/



  moveTurntable(ci_turntable_center);
  moveArm(ci_arm_drive_through);


  /*turn wrist to verical position*/

  sweepServo(servo_wrist, ci_wrist_parallel);
  /*move elbow to horizontal position*/
  driveStraightAheadEncoders(1600, 1000);
  
  //parallel();

  Serial.println("about to go through gate");
  //THROUGH THE GATE
  /*using ultrasonic sensors make sure we are directly parallel to wall drive forward straight using wall
  follower code/encoders*/


  while ((backRightSensorData < (ci_mode2_scan_distance + 20))||(backRightSensorData ==0)) {

    backRightSensorData = backRightPingSensor.ping_cm();
    followWallReverse(1400, 'R', ci_mode2_scan_distance);
  }
  stopDrive();
driveStraightAheadEncoders(1600, 100);
  
  //parallel();

  Serial.println("paralelel to wall now i think");
  stopDrive();

  tesseract_counter++;
  if (tesseract_counter == 1)
  {
    Serial.println("tesseract 1");
    /*drive straight for some set distance after we get a reading from the right ultrasonic*/
    driveStraightReverseEncoders(1400, -ci_mode2_drive_placement1);
    //PLACEMENT
    Serial.println("tesseract 1 drove to placement value");
    /*set elbow and wrist value to predetermined value for
     * placement*/
    sweepServo(servo_wrist, ci_wrist_vertical_position);
    delay(500);
    moveArmSweep(ci_arm_mode2_tesseract_dropoff);
    sweepServo(servo_wrist, ci_wrist_mode2_tesseract_dropoff);
    delay(200);
    moveTurntable(ci_turntable_mode2_placement1);
    delay(200);



  }
  else if (tesseract_counter == 2) {
    Serial.println("tesseract 2");
    /*drive straight for some set distance after we get a reading from the right ultrasonic*/
    driveStraightReverseEncoders(1400, -ci_mode2_drive_placement2);
    //PLACEMENT
    Serial.println("tesseract 1 drove to placement value");
    /*set elbow and wrist value to predetermined value for
     * placement*/
    sweepServo(servo_wrist, ci_wrist_vertical_position);
    delay(500);
    moveArmSweep(ci_arm_mode2_tesseract_dropoff-100);
    sweepServo(servo_wrist, ci_wrist_mode2_tesseract_dropoff);
    delay(200);
    moveTurntable(ci_turntable_mode2_placement2);
    delay(200);
  }
  else if (tesseract_counter == 3) {
    Serial.println("tesseract 3");
    /*drive straight for some set distance after we get a reading from the right ultrasonic*/
    driveStraightReverseEncoders(1400, -ci_mode2_drive_placement3);
    //PLACEMENT
    Serial.println("tesseract 3 drove to placement value");
    /*set elbow and wrist value to predetermined value for
     * placement*/
    sweepServo(servo_wrist, ci_wrist_vertical_position);
    delay(500);
    moveArmSweep(ci_arm_mode2_tesseract_dropoff-100);
    sweepServo(servo_wrist, ci_wrist_mode2_tesseract_dropoff);
    delay(200);
    moveTurntable(ci_turntable_mode2_placement3);
    delay(200);
  }
  sweepServo(servo_wrist, ci_wrist_mode2_tesseract_dropoff);
  /*retract the magnet motor to drop magnet*/

  moveArm(ci_arm_mode2_tesseract_dropoff_bump);
  delay(200);
  sweepServo(servo_magnet, ci_magnet_retract);
  /*move elbow motor to horizontal position and move wrist motor to vertical position*/
  delay(200);
  moveArmSweep(ci_arm_mode2_tesseract_dropoff - 100);
  sweepServo(servo_wrist, ci_wrist_vertical_position);

  moveArm(ci_arm_drive_through - 250);
  moveTurntable(ci_turntable_center);
  moveArm(ci_arm_drive_through);
  sweepServo(servo_wrist, ci_wrist_parallel);
  //RESTART
  /*drive back through gate using drive straight code and then wall follower after a
   * reading from the right ultrasonic */
  while (backLeftSensorData > ci_left_gate_distance) {
    driveStraight(1600);
    backLeftSensorData = backLeftPingSensor.ping_cm();
  }
  stopDrive();
  driveStraightAheadEncoders(1600, ci_restart_backup_distance);
  /*start from the top again*/

}
/* if (frontLeftSensorData == backLeftSensorData )
  {
    while (backLeftSensorData > ci_left_gate_distance) {
      driveStraightReverse(1400);
      backLeftSensorData = backLeftPingSensor.ping_cm();
    }
    stopDrive();
  }
  else {
    while (frontLeftSensorData != backLeftSensorData) {
      if (frontLeftSensorData > backLeftSensorData) {
        right_motor.writeMicroseconds(1600);
        backLeftSensorData = backLeftPingSensor.ping_cm();
      }
      stopDrive();
      if (frontLeftSensorData < backLeftSensorData) {
        right_motor.writeMicroseconds(1400);
        backLeftSensorData = backLeftPingSensor.ping_cm();
      }
      stopDrive();
    }
    stopDrive();
    while (backLeftSensorData > ci_left_gate_distance) {
      driveStraightReverse(1400);
      backLeftSensorData = backLeftPingSensor.ping_cm();
    }
    stopDrive();
  }
*/




