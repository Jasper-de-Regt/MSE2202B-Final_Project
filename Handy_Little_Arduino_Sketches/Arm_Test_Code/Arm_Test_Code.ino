/*
  MSE 2202 MSEBot base code for Labs 3 and 4
  Language: Arduino
  Authors: Michael Naish and Eugen Porter with some addition from Michael Henderson and Chris Bayley
  Date: 16/01/17
  Rev 1 - Initial version
  Rev 2 - Update for MSEduino v. 2
*/

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

// Uncomment keywords to enable debugging output

//#define DEBUG_MODE_DISPLAY
//#define DEBUG_MOTORS
//#define DEBUG_LINE_TRACKERS
//#define DEBUG_ENCODERS
//#define DEBUG_ULTRASONIC
//#define DEBUG_LINE_TRACKER_CALIBRATION
//#define DEBUG_MOTOR_CALIBRATION
//#define DEBUG_CASE
#define LINE_TRACKING_CASES

boolean bt_Motors_Enabled = true;

//port pin constants
const int ci_Ultrasonic_Ping = 2;   //input plug
const int ci_Ultrasonic_Data = 3;   //output plug
const int ci_Charlieplex_LED1 = 4;
const int ci_Charlieplex_LED2 = 5;
const int ci_Charlieplex_LED3 = 6;
const int ci_Charlieplex_LED4 = 7;
const int ci_Mode_Button = 7;
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Arm_Motor = 10;
const int ci_Grip_Motor = 11;
const int ci_Motor_Enable_Switch = 12;
const int ci_Right_Line_Tracker = A0;
const int ci_Middle_Line_Tracker = A1;
const int ci_Left_Line_Tracker = A2;
const int ci_Light_Sensor = A3;
const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow

// Charlieplexing LED assignments
const int ci_Heartbeat_LED = 1;
const int ci_Indicator_LED = 4;
const int ci_Right_Line_Tracker_LED = 6;
const int ci_Middle_Line_Tracker_LED = 9;
const int ci_Left_Line_Tracker_LED = 12;

//constants

// EEPROM addresses
const int ci_Left_Line_Tracker_Dark_Address_L = 0;
const int ci_Left_Line_Tracker_Dark_Address_H = 1;
const int ci_Left_Line_Tracker_Light_Address_L = 2;
const int ci_Left_Line_Tracker_Light_Address_H = 3;
const int ci_Middle_Line_Tracker_Dark_Address_L = 4;
const int ci_Middle_Line_Tracker_Dark_Address_H = 5;
const int ci_Middle_Line_Tracker_Light_Address_L = 6;
const int ci_Middle_Line_Tracker_Light_Address_H = 7;
const int ci_Right_Line_Tracker_Dark_Address_L = 8;
const int ci_Right_Line_Tracker_Dark_Address_H = 9;
const int ci_Right_Line_Tracker_Light_Address_L = 10;
const int ci_Right_Line_Tracker_Light_Address_H = 11;
const int ci_Left_Motor_Offset_Address_L = 12;
const int ci_Left_Motor_Offset_Address_H = 13;
const int ci_Right_Motor_Offset_Address_L = 14;
const int ci_Right_Motor_Offset_Address_H = 15;

const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Grip_Motor_Open = 140;         // Experiment to determine appropriate value
const int ci_Grip_Motor_Closed = 15;        //  "
const int ci_Arm_Servo_Retracted = 30;//  "
const int ci_Arm_Servo_Scan = 75;//  "
const int ci_Arm_Servo_Middle = 90;//  "
const int ci_Arm_Servo_Extended = 150;//  "
const int ci_Display_Time = 500;
const int ci_Line_Tracker_Calibration_Interval = 100;
const int ci_Line_Tracker_Cal_Measures = 20;
const int ci_Line_Tracker_Tolerance = 50;   // May need to adjust this
const int ci_Motor_Calibration_Cycles = 3;
const int ci_Motor_Calibration_Time = 5000;
const int ci_Light_Sensor_Threshold = 50;

//variables
byte b_LowByte;
byte b_HighByte;
unsigned long ul_Echo_Time;
unsigned int ui_Left_Line_Tracker_Data;
unsigned int ui_Middle_Line_Tracker_Data;
unsigned int ui_Right_Line_Tracker_Data;
unsigned int ui_Motors_Speed = 1625;        // Default run speed was 1900
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;
long l_Left_Motor_Position;
long l_Right_Motor_Position;

unsigned long ul_3_Second_timer = 0;
unsigned long ul_Display_Time;
unsigned long ul_Calibration_Time;
unsigned long ui_Left_Motor_Offset;
unsigned long ui_Right_Motor_Offset;

unsigned int ui_Cal_Count;
unsigned int ui_Cal_Cycle;
unsigned int ui_Left_Line_Tracker_Dark;
unsigned int ui_Left_Line_Tracker_Light;
unsigned int ui_Middle_Line_Tracker_Dark;
unsigned int ui_Middle_Line_Tracker_Light;
unsigned int ui_Right_Line_Tracker_Dark;
unsigned int ui_Right_Line_Tracker_Light;
unsigned int ui_Line_Tracker_Tolerance;
unsigned int TimeTurning = 1;       //should be 1 if not im testing
unsigned int firstEncoderReadRight;
unsigned int firstEncoderReadLeft;
unsigned int operationPhase = 1;    //should be 1 if not im testing
unsigned int timesatthree = 0;
unsigned int prevDirection; // 0 for left, 1 for right
unsigned int ui_Best_Left_Encoder_Position = 0;
unsigned int ui_Best_Right_Encoder_Position = 0;
unsigned int ui_last_best_light_reading = 100;
unsigned int ui_current_light_reading = 0;
unsigned int ui_turnVar = 0;

bool turnLeftAtFirstStop = false;
bool activatedOnce = false;

unsigned int  ui_Robot_State_Index = 0;
//0123456789ABCDEF
unsigned int  ui_Mode_Indicator[6] = {
  0x00,    //B0000000000000000,  //Stop
  0x00FF,  //B0000000011111111,  //Run
  0x0F0F,  //B0000111100001111,  //Calibrate line tracker light level
  0x3333,  //B0011001100110011,  //Calibrate line tracker dark level
  0xAAAA,  //B1010101010101010,  //Calibrate motors
  0xFFFF   //B1111111111111111   //Unused
};

unsigned int  ui_Mode_Indicator_Index = 0;

//display Bits 0,1,2,3, 4, 5, 6,  7,  8,  9,  10,  11,  12,  13,   14,   15
int  iArray[16] = {
  1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 65536
};
int  iArrayIndex = 0;

boolean bt_Heartbeat = true;
boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;

void setup() {
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);

  CharliePlexM::setBtn(ci_Charlieplex_LED1, ci_Charlieplex_LED2,
                       ci_Charlieplex_LED3, ci_Charlieplex_LED4, ci_Mode_Button);

  // set up ultrasonic
  pinMode(ci_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Ultrasonic_Data, INPUT);

  // set up drive motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);

  // set up arm motors
  pinMode(ci_Arm_Motor, OUTPUT);
  servo_ArmMotor.attach(ci_Arm_Motor);
  pinMode(ci_Grip_Motor, OUTPUT);
  servo_GripMotor.attach(ci_Grip_Motor);

  // set up motor enable switch
  pinMode(ci_Motor_Enable_Switch, INPUT);

  // set up encoders. Must be initialized in order that they are chained together,
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward

  // set up line tracking sensors
  pinMode(ci_Right_Line_Tracker, INPUT);
  pinMode(ci_Middle_Line_Tracker, INPUT);
  pinMode(ci_Left_Line_Tracker, INPUT);
  ui_Line_Tracker_Tolerance = ci_Line_Tracker_Tolerance;

  // read saved values from EEPROM
  b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Left_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Left_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Middle_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Middle_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Right_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Right_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Left_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Motor_Offset_Address_H);
  ui_Left_Motor_Offset = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
  ui_Right_Motor_Offset = word(b_HighByte, b_LowByte);
}

void loop()
{
  if ((millis() - ul_3_Second_timer) > 3000)
  {
    bt_3_S_Time_Up = true;
  }

  // button-based mode selection
  if (CharliePlexM::ui_Btn)
  {
    if (bt_Do_Once == false)
    {
      bt_Do_Once = true;
      ui_Robot_State_Index++;
      ui_Robot_State_Index = ui_Robot_State_Index & 7;
      ul_3_Second_timer = millis();
      bt_3_S_Time_Up = false;
      bt_Cal_Initialized = false;
      ui_Cal_Cycle = 0;
    }
  }
  else
  {
    bt_Do_Once = LOW;
  }

  // check if drive motors should be powered
  bt_Motors_Enabled = digitalRead(ci_Motor_Enable_Switch);

#ifdef DEBUG_CASE
  Serial.println();
  Serial.print("Case: ");
  Serial.println(ui_Robot_State_Index);
#endif

  int left_Encoder_Position;
  int right_Encoder_Position;


  // modes
  // 0 = default after power up/reset
  // 1 = Press mode button once to enter. Run robot.
  // 2 = Press mode button twice to enter. Calibrate line tracker light level.
  // 3 = Press mode button three times to enter. Calibrate line tracker dark level.
  // 4 = Press mode button four times to enter. Calibrate motor speeds to drive straight.
  switch (ui_Robot_State_Index)
  {
    case 0:    //Robot stopped
      {
        readLineTrackers();
        Ping();
        servo_LeftMotor.writeMicroseconds(2000);
        servo_RightMotor.writeMicroseconds(2000);
        servo_ArmMotor.write(ci_Arm_Servo_Retracted);
        servo_GripMotor.write(ci_Grip_Motor_Closed);
        //encoder_LeftMotor.zero();
        //encoder_RightMotor.zero();
        ui_Mode_Indicator_Index = 0;

        //servo_LeftMotor.write(1650);
        //servo_RightMotor.write(1650);



#ifdef DEBUG_ENCODERS
        l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
        l_Right_Motor_Position = encoder_RightMotor.getRawPosition();

        /*
          left_Encoder_Position += l_Left_Motor_Position;
          left_Encoder_Position += l_Right_Motor_Position;
        */

        left_Encoder_Position = encoder_LeftMotor.getRawPosition();
        right_Encoder_Position = encoder_RightMotor.getRawPosition();


        Serial.print("Encoders L: ");
        Serial.print(left_Encoder_Position);
        Serial.print(", R: ");
        Serial.println(right_Encoder_Position);


#endif

        break;
      }

    case 1:    //Robot Run after 3 seconds
      {
        unsigned int pastCondition = 0;
        unsigned int currentCondition = 0;
        unsigned int previousTimeMeasurement; //= millis();

        unsigned int ui_Left_On_Yellow;
        unsigned int ui_Middle_On_Yellow;
        unsigned int ui_Right_On_Yellow;



        if (bt_3_S_Time_Up)
        {
          readLineTrackers();

#ifdef DEBUG_ENCODERS
          l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
          l_Right_Motor_Position = encoder_RightMotor.getRawPosition();


          Serial.print("Encoders L: ");
          Serial.print(l_Left_Motor_Position);
          Serial.print(", R: ");
          Serial.println(l_Right_Motor_Position);


#endif

          // set motor speeds
          ui_Left_Motor_Speed = constrain(ui_Motors_Speed + ui_Left_Motor_Offset, 1600, 2100);
          ui_Right_Motor_Speed = constrain(ui_Motors_Speed + ui_Right_Motor_Offset, 1600, 2100);

          ui_Left_On_Yellow = (ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ui_Line_Tracker_Tolerance));
          ui_Middle_On_Yellow = (ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ui_Line_Tracker_Tolerance));
          ui_Right_On_Yellow = (ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance));

          /****************************************
            Operational Phases
            1. Follow line to first block then stop
            2. Move arm forward and open claw.
            3. Make the 90o turn.
            4. Line tracking code again, stop at second block.
            5. Scan environment for highest light sensor read (lowest is brightest) and record encoder values at the brightest.
            6. Make wheels return to that position.
            7. Move 5cm away, get the light and retract the claw.
            8. Turn to find next line.
            9. Line tracking again.
            10. Turn 90o again.
            11. Drive straight to drop off point.
            12. Extend arm and release.
          *****************************************/


          Serial.print("OP ");
          Serial.print(operationPhase);

          if (operationPhase == 1) {

            trackLine(ui_Left_On_Yellow, ui_Middle_On_Yellow, ui_Right_On_Yellow);

            if ((ui_Left_On_Yellow && ui_Middle_On_Yellow && ui_Right_On_Yellow)) {
              timesatthree++;
              Serial.println("a");
            }
            if ((ui_Left_On_Yellow && ui_Middle_On_Yellow && ui_Right_On_Yellow) && (timesatthree > 50)) {
              stop();
              operationPhase++;
              Serial.println("b");
            }
          }


          else if (operationPhase == 2) {
            servo_ArmMotor.write(ci_Arm_Servo_Scan); //30 is retracted, Full extension is 150,
            servo_GripMotor.write(ci_Grip_Motor_Open); //Closed is 15, Open is 140
            operationPhase++;
            Serial.println("c");
            firstEncoderReadRight = encoder_RightMotor.getRawPosition();
            delay(1000);
          }


          else if (operationPhase == 3) {
            if (ui_Left_On_Yellow && ui_Middle_On_Yellow && ui_Right_On_Yellow) {
              Serial.println("d");
              while ((TimeTurning == 1 && ((firstEncoderReadRight + 700) >= encoder_RightMotor.getRawPosition()))) {
                readLineTrackers();
                Serial.println("e");
                servo_RightMotor.writeMicroseconds(1650);
                servo_LeftMotor.writeMicroseconds(1450);
                if (ui_Left_On_Yellow && ((firstEncoderReadRight + 400) <= encoder_RightMotor.getRawPosition())) {
                  Serial.println("eEeEeEeEeEeEeEeEeE");
                  stop();
                  break;
                }
              }
            }
            Serial.println("f");
            TimeTurning = 2;
            operationPhase++;
            
            timesatthree = 0;
          }

          else if (operationPhase == 4) {
            Serial.println("g");
            
            trackLine(ui_Left_On_Yellow, ui_Middle_On_Yellow, ui_Right_On_Yellow);

            if ((ui_Left_On_Yellow && ui_Middle_On_Yellow && ui_Right_On_Yellow)) {
              timesatthree++;
              Serial.println("h");
            }
            if ((ui_Left_On_Yellow && ui_Middle_On_Yellow && ui_Right_On_Yellow) && (timesatthree > 9)) {
              stop();
              operationPhase++;
              firstEncoderReadRight = encoder_RightMotor.getRawPosition();
              firstEncoderReadLeft = encoder_LeftMotor.getRawPosition();
              Serial.println("i");
            }
            delay(300);
          }

          else if (operationPhase == 5) {
            //Scan for the pedestal
            //Zeroing the encoders
            //encoder_LeftMotor.zero();
            //encoder_RightMotor.zero();

            Serial.println("j");


            //First Turn
            while (TimeTurning == 2 && ((firstEncoderReadRight + 200) >= encoder_RightMotor.getRawPosition())) {
              Serial.println("k");
              servo_RightMotor.writeMicroseconds(1625);
              servo_LeftMotor.writeMicroseconds(1500);
              ////Serial.println("Loop 1");
              ui_current_light_reading = analogRead(ci_Light_Sensor);
              if (((ui_current_light_reading < ci_Light_Sensor_Threshold) ) || (ui_last_best_light_reading > ui_current_light_reading) && (ui_current_light_reading != 0) ) {
                Serial.println("l");
                ui_last_best_light_reading = ui_current_light_reading;
                ui_Best_Right_Encoder_Position = encoder_RightMotor.getRawPosition();
                TimeTurning = 6;
                //Serial.print("WHY YOU NO WORK");
              }

              /*
              if ((firstEncoderReadRight + 150) <= encoder_RightMotor.getRawPosition()) {
                TimeTurning = 3;
              }
              */
            }
            //Should only run after the encoder difference is greater than 150
            TimeTurning = 3;

            Serial.println("m");
            //Diagnostic
            ////Serial.print("Best Encoder Right: ");
            //Serial.println(ui_Best_Right_Encoder_Position);
            //Serial.print("Best Encoder Left: ");
            //Serial.println(ui_Best_Left_Encoder_Position);

            stop();

            //Second Turn
            while ((TimeTurning == 3 && (firstEncoderReadRight <= encoder_RightMotor.getRawPosition()))) {
              Serial.println("n");
              servo_RightMotor.writeMicroseconds(1350);
              servo_LeftMotor.writeMicroseconds(1500);
              //Serial.println("Loop 2");
              ui_current_light_reading = analogRead(ci_Light_Sensor);
              if (((ui_current_light_reading < ci_Light_Sensor_Threshold) ) || (ui_last_best_light_reading > ui_current_light_reading) && (ui_current_light_reading != 0)) {
                Serial.println("o");
                ui_last_best_light_reading = ui_current_light_reading;
                ui_Best_Right_Encoder_Position = encoder_RightMotor.getRawPosition();
                TimeTurning = 6;
              }
              if (firstEncoderReadRight >= encoder_RightMotor.getRawPosition()) {
                Serial.println("p");
                TimeTurning = 4;
              }
            }


            //Diagnostic
            //Serial.print("Best Encoder Right: ");
            //Serial.println(ui_Best_Right_Encoder_Position);
            //Serial.print("Best Encoder Left: ");
            //Serial.println(ui_Best_Left_Encoder_Position);


            stop();


            //Third Turn
            while ((TimeTurning == 4 && ((firstEncoderReadLeft + 150) >= encoder_LeftMotor.getRawPosition()))) {
              servo_RightMotor.writeMicroseconds(1500);
              servo_LeftMotor.writeMicroseconds(1600);
              Serial.println("q");
              //Serial.println("Loop 3");
              ui_current_light_reading = analogRead(ci_Light_Sensor);
              if (((ui_current_light_reading < ci_Light_Sensor_Threshold) ) || (ui_last_best_light_reading > ui_current_light_reading) && (ui_current_light_reading != 0)) {
                ui_last_best_light_reading = ui_current_light_reading;
                ui_Best_Left_Encoder_Position = encoder_LeftMotor.getRawPosition();
                //Serial.println(ui_Best_Left_Encoder_Position);
                Serial.println("r");
                TimeTurning = 6;
              }
              if ((firstEncoderReadLeft + 150) <= encoder_LeftMotor.getRawPosition()) {
                TimeTurning = 5;
                Serial.println("s");
              }
            }

            //Diagnostic
            //Serial.print("Best Encoder Right: ");
            //Serial.println(ui_Best_Right_Encoder_Position);
            //Serial.print("Best Encoder Left: ");
            //Serial.println(ui_Best_Left_Encoder_Position);

            stop();

            //Fourth Turn
            while ((TimeTurning == 5 && (firstEncoderReadLeft <= encoder_LeftMotor.getRawPosition()))) {
              Serial.println("t");
              servo_RightMotor.writeMicroseconds(1500);
              servo_LeftMotor.writeMicroseconds(1350);
              //Serial.println("Loop 4");
              ui_current_light_reading = analogRead(ci_Light_Sensor);
              if (((ui_current_light_reading < ci_Light_Sensor_Threshold)  ) || (ui_last_best_light_reading > ui_current_light_reading) && (ui_current_light_reading != 0)) {
                ui_last_best_light_reading = ui_current_light_reading;
                ui_Best_Left_Encoder_Position = encoder_LeftMotor.getRawPosition();
                //Serial.println(ui_Best_Left_Encoder_Position);
                Serial.println("u");
                TimeTurning = 6;
              }
              if (firstEncoderReadLeft >= encoder_LeftMotor.getRawPosition()) {
                TimeTurning = 6;
                Serial.println("v");
              }
            }


            //Diagnostic
            //Serial.print("Best Encoder Right: ");
            //Serial.println(ui_Best_Right_Encoder_Position);
            //Serial.print("Best Encoder Left: ");
            //Serial.println(ui_Best_Left_Encoder_Position);

            Serial.println("w");
            stop();
            operationPhase++;
          }



          else if (operationPhase == 6) {
            //Return to the encoder position registered by the sensors of the brightest light
            while ((TimeTurning == 6 && (firstEncoderReadRight <= ui_Best_Right_Encoder_Position))) {
              Serial.println("x");
              servo_RightMotor.writeMicroseconds(1600);
              servo_LeftMotor.writeMicroseconds(1500);
              if (encoder_RightMotor.getRawPosition() >= ui_Best_Right_Encoder_Position) {
                TimeTurning = 7;
                stop();
                //Serial.println(encoder_RightMotor.getRawPosition());
                Serial.println("y");
              }
            }
            while ((TimeTurning == 7 && (firstEncoderReadLeft <= ui_Best_Left_Encoder_Position))) {
              servo_RightMotor.writeMicroseconds(1500);
              servo_LeftMotor.writeMicroseconds(1600);
              Serial.println("z");
              if (encoder_LeftMotor.getRawPosition() >= ui_Best_Left_Encoder_Position) {
                //TimeTurning = 8;
                stop();
                Serial.println("A");
              }
            }

            //Serial.print("Best Encoder Right: ");
            //Serial.println(ui_Best_Right_Encoder_Position);
            //Serial.print("Best Encoder Left: ");
            //Serial.println(ui_Best_Left_Encoder_Position);
            TimeTurning = 8;
            operationPhase++;
            Serial.println("B");
          }



          else if (operationPhase == 7) {
            Serial.println("C");
            Ping();
            //int distance = (ul_Echo_Time / 24);
            servo_LeftMotor.writeMicroseconds(1600);
            servo_RightMotor.writeMicroseconds(1600);
            //Ping();
            //Serial.print("Loop condition 2: ");
            //Serial.println( ( (ul_Echo_Time / 24) > 10 ) );


            //Serial.println( (ul_Echo_Time / 24) );

            if (((ul_Echo_Time / 24) < 10) && (ul_Echo_Time / 24)) {
              Serial.println("D");
              stop();

              //Arm & Grip commands
              servo_GripMotor.write(ci_Grip_Motor_Open);
              moveArmSlowly(ci_Arm_Servo_Scan, ci_Arm_Servo_Extended);
              servo_GripMotor.write(ci_Grip_Motor_Closed);
              moveArmSlowly(ci_Arm_Servo_Extended, ci_Arm_Servo_Middle);
              //Updates the encoder values.
              firstEncoderReadLeft = encoder_LeftMotor.getRawPosition();
              firstEncoderReadRight = encoder_RightMotor.getRawPosition();

              previousTimeMeasurement = millis();
              //Essential to get OP 8 to work
              //TimeTurning = 8;
              operationPhase++;
            }
            //servo_LeftMotor.write(1500);
            //servo_RightMotor.write(1500);
            //operationPhase++;
            Serial.println("F");

          }


          else if (operationPhase == 8) {//Turn and then follow the line to the block


            while ((TimeTurning == 8 && ((firstEncoderReadRight - 400) <= encoder_RightMotor.getRawPosition()))) {
              servo_RightMotor.writeMicroseconds(1400);
              servo_LeftMotor.writeMicroseconds(1400);
              Serial.println("G");
            }
            /*
            firstEncoderReadLeft = encoder_LeftMotor.getRawPosition();
            while ((firstEncoderReadLeft + 900) >= encoder_LeftMotor.getRawPosition()) {
              servo_LeftMotor.writeMicroseconds(1600);
              servo_LeftMotor.writeMicroseconds(1400);
            }
            */
            encoder_LeftMotor.zero();
            encoder_RightMotor.zero();
            firstEncoderReadLeft = encoder_LeftMotor.getRawPosition();
            firstEncoderReadRight = encoder_RightMotor.getRawPosition();
            //Serial.print("Pre: L "); //Serial.println(firstEncoderReadLeft);
            //Serial.print("Pre: R "); //Serial.println(firstEncoderReadRight);
            //Serial.print("Upper left bound: "); //Serial.println(firstEncoderReadLeft + 800);
            //Serial.print((firstEncoderReadLeft + 800) >= encoder_RightMotor.getRawPosition());
            Serial.println("H");
            if (ui_Best_Left_Encoder_Position >= ui_Best_Right_Encoder_Position) {
              ui_turnVar = 700;
              Serial.println("H");
            }
            else {
              ui_turnVar = 850;
              Serial.println("H'");
            }
            while ((TimeTurning == 8 && ((firstEncoderReadLeft + 600) >= encoder_LeftMotor.getRawPosition()))) {
              readLineTrackers();
              servo_RightMotor.writeMicroseconds(1500);
              servo_LeftMotor.writeMicroseconds(1700);
              if (ui_Right_On_Yellow || ui_Middle_On_Yellow) {
                break;
              }
              //Serial.print("During: L "); //Serial.println(firstEncoderReadLeft);
              //Serial.print("During: R "); //Serial.println(firstEncoderReadRight);
              //Serial.print("Raw: L "); //Serial.println(encoder_LeftMotor.getRawPosition());
              //Serial.print("Raw: R "); //Serial.println(encoder_RightMotor.getRawPosition());
              Serial.println("I");
            }
            operationPhase++;
            TimeTurning++;
            Serial.println("J");

          }



          else if (operationPhase == 9) {
            trackLine(ui_Left_On_Yellow, ui_Middle_On_Yellow, ui_Right_On_Yellow);
            if ((ui_Left_On_Yellow && ui_Middle_On_Yellow && ui_Right_On_Yellow)) {
              timesatthree++;
              Serial.println("K");
            }
            if ((ui_Left_On_Yellow && ui_Middle_On_Yellow && ui_Right_On_Yellow) && (timesatthree > 30)) {
              stop();
              operationPhase++;
              firstEncoderReadRight = encoder_RightMotor.getRawPosition();
              Serial.println("L");
            }
          }



          else if (operationPhase == 10) {
            if (ui_Left_On_Yellow && ui_Middle_On_Yellow && ui_Right_On_Yellow) {
              Serial.println("M");
              while ((firstEncoderReadRight - 600) <= encoder_RightMotor.getRawPosition()) {
                servo_RightMotor.writeMicroseconds(1400);
                servo_LeftMotor.writeMicroseconds(1400);
                
              }
              

            }
            stop();
            firstEncoderReadRight = encoder_RightMotor.getRawPosition();
            
            stop();
            TimeTurning = 10;
            operationPhase++;
            timesatthree = 0;
            Serial.println("O");
          }



          else if (operationPhase == 11) {


            Ping();
            servo_LeftMotor.writeMicroseconds(1600);
            servo_RightMotor.writeMicroseconds(1700);
            Serial.println("P");
            //Serial.print("Echo cm: "); //Serial.println(ul_Echo_Time / 24);
            if (((ul_Echo_Time / 24) < 10) && ul_Echo_Time) {
              stop();
              operationPhase++;
              Serial.println("Q");
            }
            previousTimeMeasurement = millis();
            Serial.println("R");
          }



          else if (operationPhase == 12) {
            stop();
            moveArmSlowly(ci_Arm_Servo_Middle, ci_Arm_Servo_Extended);
            Serial.println("S");
            if ((millis() - previousTimeMeasurement) >= 1000) {
              //Serial.println("Fuck ");
              servo_GripMotor.write(ci_Grip_Motor_Open);
              Serial.println("T");
            }
            operationPhase++;
            delay(500);
            moveArmSlowly(ci_Arm_Servo_Extended, ci_Arm_Servo_Retracted);
            previousTimeMeasurement = millis();
            Serial.println("U");
          }

          else if (operationPhase == 13) { //Celebration

            servo_LeftMotor.writeMicroseconds(1000);
            servo_RightMotor.writeMicroseconds(1000);
            //Serial.println(millis() - previousTimeMeasurement);
            Serial.println("V");
            while ((millis() - previousTimeMeasurement) <= 500) {
              servo_LeftMotor.writeMicroseconds(1000);
              servo_RightMotor.writeMicroseconds(1000);
              //Serial.println("FUCK YES!!!!!!");
              operationPhase++;
              Serial.println("W");
            }
            stop();
            Serial.println("X");

          }
          else if (operationPhase == 14) {
            servo_LeftMotor.writeMicroseconds(1000);
            servo_RightMotor.writeMicroseconds(2000);
            Serial.println("Y");
          }



#ifdef DEBUG_MOTORS
          Serial.print("Motors enabled: ");
          Serial.print(bt_Motors_Enabled);
          Serial.print(", Default: ");
          Serial.print(ui_Motors_Speed);
          Serial.print(", Left = ");
          Serial.print(ui_Left_Motor_Speed);
          Serial.print(", Right = ");
          Serial.println(ui_Right_Motor_Speed);
#endif
          ui_Mode_Indicator_Index = 1;
        }
        break;
      }

    case 2:    //Calibrate line tracker light levels after 3 seconds
      {
        if (bt_3_S_Time_Up)
        {
          if (!bt_Cal_Initialized)
          {
            bt_Cal_Initialized = true;
            ui_Left_Line_Tracker_Light = 0;
            ui_Middle_Line_Tracker_Light = 0;
            ui_Right_Line_Tracker_Light = 0;
            ul_Calibration_Time = millis();
            ui_Cal_Count = 0;
          }
          else if ((millis() - ul_Calibration_Time) > ci_Line_Tracker_Calibration_Interval)
          {
            ul_Calibration_Time = millis();
            readLineTrackers();
            ui_Left_Line_Tracker_Light += ui_Left_Line_Tracker_Data;
            ui_Middle_Line_Tracker_Light += ui_Middle_Line_Tracker_Data;
            ui_Right_Line_Tracker_Light += ui_Right_Line_Tracker_Data;
            ui_Cal_Count++;
          }
          if (ui_Cal_Count == ci_Line_Tracker_Cal_Measures)
          {
            ui_Left_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
            ui_Middle_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
            ui_Right_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
#ifdef DEBUG_LINE_TRACKER_CALIBRATION
            Serial.print("Light Levels: Left = ");
            Serial.print(ui_Left_Line_Tracker_Light, DEC);
            Serial.print(", Middle = ");
            Serial.print(ui_Middle_Line_Tracker_Light, DEC);
            Serial.print(", Right = ");
            Serial.println(ui_Right_Line_Tracker_Light, DEC);
#endif
            EEPROM.write(ci_Left_Line_Tracker_Light_Address_L, lowByte(ui_Left_Line_Tracker_Light));
            EEPROM.write(ci_Left_Line_Tracker_Light_Address_H, highByte(ui_Left_Line_Tracker_Light));
            EEPROM.write(ci_Middle_Line_Tracker_Light_Address_L, lowByte(ui_Middle_Line_Tracker_Light));
            EEPROM.write(ci_Middle_Line_Tracker_Light_Address_H, highByte(ui_Middle_Line_Tracker_Light));
            EEPROM.write(ci_Right_Line_Tracker_Light_Address_L, lowByte(ui_Right_Line_Tracker_Light));
            EEPROM.write(ci_Right_Line_Tracker_Light_Address_H, highByte(ui_Right_Line_Tracker_Light));
            ui_Robot_State_Index = 0;    // go back to Mode 0
          }
          ui_Mode_Indicator_Index = 2;
        }
        break;
      }

    case 3:    // Calibrate line tracker dark levels after 3 seconds
      {
        if (bt_3_S_Time_Up)
        {
          if (!bt_Cal_Initialized)
          {
            bt_Cal_Initialized = true;
            ui_Left_Line_Tracker_Dark = 0;
            ui_Middle_Line_Tracker_Dark = 0;
            ui_Right_Line_Tracker_Dark = 0;
            ul_Calibration_Time = millis();
            ui_Cal_Count = 0;
          }
          else if ((millis() - ul_Calibration_Time) > ci_Line_Tracker_Calibration_Interval)
          {
            ul_Calibration_Time = millis();
            readLineTrackers();
            ui_Left_Line_Tracker_Dark += ui_Left_Line_Tracker_Data;
            ui_Middle_Line_Tracker_Dark += ui_Middle_Line_Tracker_Data;
            ui_Right_Line_Tracker_Dark += ui_Right_Line_Tracker_Data;
            ui_Cal_Count++;
          }
          if (ui_Cal_Count == ci_Line_Tracker_Cal_Measures)
          {
            ui_Left_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
            ui_Middle_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
            ui_Right_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
#ifdef DEBUG_LINE_TRACKER_CALIBRATION
            Serial.print("Dark Levels: Left = ");
            Serial.print(ui_Left_Line_Tracker_Dark, DEC);
            Serial.print(", Middle = ");
            Serial.print(ui_Middle_Line_Tracker_Dark, DEC);
            Serial.print(", Right = ");
            Serial.println(ui_Right_Line_Tracker_Dark, DEC);
#endif
            EEPROM.write(ci_Left_Line_Tracker_Dark_Address_L, lowByte(ui_Left_Line_Tracker_Dark));
            EEPROM.write(ci_Left_Line_Tracker_Dark_Address_H, highByte(ui_Left_Line_Tracker_Dark));
            EEPROM.write(ci_Middle_Line_Tracker_Dark_Address_L, lowByte(ui_Middle_Line_Tracker_Dark));
            EEPROM.write(ci_Middle_Line_Tracker_Dark_Address_H, highByte(ui_Middle_Line_Tracker_Dark));
            EEPROM.write(ci_Right_Line_Tracker_Dark_Address_L, lowByte(ui_Right_Line_Tracker_Dark));
            EEPROM.write(ci_Right_Line_Tracker_Dark_Address_H, highByte(ui_Right_Line_Tracker_Dark));
            ui_Robot_State_Index = 0;    // go back to Mode 0
          }
          ui_Mode_Indicator_Index = 3;
        }
        break;
      }

    case 4:    //Calibrate motor straightness after 3 seconds.
      {
        if (bt_3_S_Time_Up)
        {
          if (!bt_Cal_Initialized)
          {
            bt_Cal_Initialized = true;
            encoder_LeftMotor.zero();
            encoder_RightMotor.zero();
            ul_Calibration_Time = millis();
            servo_LeftMotor.writeMicroseconds(ui_Motors_Speed);
            servo_RightMotor.writeMicroseconds(ui_Motors_Speed);
          }
          else if ((millis() - ul_Calibration_Time) > ci_Motor_Calibration_Time)
          {
            servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
            servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
            l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
            l_Right_Motor_Position = encoder_RightMotor.getRawPosition();
            if (l_Left_Motor_Position > l_Right_Motor_Position)
            {
              // May have to update this if different calibration time is used
              ui_Right_Motor_Offset = 0;
              ui_Left_Motor_Offset = (l_Left_Motor_Position - l_Right_Motor_Position) / 4;
            }
            else
            {
              // May have to update this if different calibration time is used
              ui_Right_Motor_Offset = (l_Right_Motor_Position - l_Left_Motor_Position) / 4;
              ui_Left_Motor_Offset = 0;
            }

#ifdef DEBUG_MOTOR_CALIBRATION
            Serial.print("Motor Offsets: Left = ");
            Serial.print(ui_Left_Motor_Offset);
            Serial.print(", Right = ");
            Serial.println(ui_Right_Motor_Offset);
#endif
            EEPROM.write(ci_Right_Motor_Offset_Address_L, lowByte(ui_Right_Motor_Offset));
            EEPROM.write(ci_Right_Motor_Offset_Address_H, highByte(ui_Right_Motor_Offset));
            EEPROM.write(ci_Left_Motor_Offset_Address_L, lowByte(ui_Left_Motor_Offset));
            EEPROM.write(ci_Left_Motor_Offset_Address_H, highByte(ui_Left_Motor_Offset));

            ui_Robot_State_Index = 0;    // go back to Mode 0
          }
#ifdef DEBUG_MOTOR_CALIBRATION
          Serial.print("Encoders L: ");
          Serial.print(encoder_LeftMotor.getRawPosition());
          Serial.print(", R: ");
          Serial.println(encoder_RightMotor.getRawPosition());
#endif
          ui_Mode_Indicator_Index = 4;
        }
        break;
      }
  }

  if ((millis() - ul_Display_Time) > ci_Display_Time)
  {
    ul_Display_Time = millis();

#ifdef DEBUG_MODE_DISPLAY
    Serial.print("Mode: ");
    Serial.println(ui_Mode_Indicator[ui_Mode_Indicator_Index], DEC);
#endif
    bt_Heartbeat = !bt_Heartbeat;
    CharliePlexM::Write(ci_Heartbeat_LED, bt_Heartbeat);
    digitalWrite(13, bt_Heartbeat);
    Indicator();
  }
}

// set mode indicator LED state
void Indicator()
{
  //display routine, if true turn on led
  CharliePlexM::Write(ci_Indicator_LED, !(ui_Mode_Indicator[ui_Mode_Indicator_Index] &
                                          (iArray[iArrayIndex])));
  iArrayIndex++;
  iArrayIndex = iArrayIndex & 15;
}

// read values from line trackers and update status of line tracker LEDs
void readLineTrackers()
{
  ui_Left_Line_Tracker_Data = analogRead(ci_Left_Line_Tracker);
  ui_Middle_Line_Tracker_Data = analogRead(ci_Middle_Line_Tracker);
  ui_Right_Line_Tracker_Data = analogRead(ci_Right_Line_Tracker);

  if (ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(ci_Left_Line_Tracker_LED, HIGH);
  }
  else
  {
    CharliePlexM::Write(ci_Left_Line_Tracker_LED, LOW);
  }
  if (ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(ci_Middle_Line_Tracker_LED, HIGH);
  }
  else
  {
    CharliePlexM::Write(ci_Middle_Line_Tracker_LED, LOW);
  }
  if (ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(ci_Right_Line_Tracker_LED, HIGH);
  }
  else
  {
    CharliePlexM::Write(ci_Right_Line_Tracker_LED, LOW);
  }

#ifdef DEBUG_LINE_TRACKERS
  Serial.print("Trackers: Left = ");
  Serial.print(ui_Left_Line_Tracker_Data, DEC);
  Serial.print(", Middle = ");
  Serial.print(ui_Middle_Line_Tracker_Data, DEC);
  Serial.print(", Right = ");
  Serial.println(ui_Right_Line_Tracker_Data, DEC);
#endif

}

// measure distance to target using ultrasonic sensor
void Ping()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Ping, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW
  ul_Echo_Time = pulseIn(ci_Ultrasonic_Data, HIGH, 10000);

  // Print Sensor Readings
#ifdef DEBUG_ULTRASONIC
  Serial.print("Time (microseconds): ");
  Serial.print(ul_Echo_Time, DEC);
  Serial.print(", Inches: ");
  Serial.print(ul_Echo_Time / 148); //divide time by 148 to get distance in inches
  Serial.print(", cm: ");
  Serial.println(ul_Echo_Time / 58); //divide time by 58 to get distance in cm
#endif
}


void trackLine(unsigned int ui_Left_On_Yellow, unsigned int ui_Middle_On_Yellow, unsigned int ui_Right_On_Yellow ) {
  if (bt_Motors_Enabled) {

    //Serial.print("Past: ");
    //Serial.println(pastCondition);


    //If nothing
    if ( !((ui_Left_On_Yellow) || (ui_Middle_On_Yellow) || (ui_Right_On_Yellow)) ) {
      Serial.println("Nothing"); // 0 is for left , 1 is for right.
      if (prevDirection == 0) {
        servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
        servo_LeftMotor.writeMicroseconds(1500);
      }
      else {
        servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
        servo_RightMotor.writeMicroseconds(1500);
      }
      //return 0;
    }

    //If all activated, stop
    else if ((ui_Left_On_Yellow) && (ui_Middle_On_Yellow) && (ui_Right_On_Yellow)) {
      servo_LeftMotor.writeMicroseconds(1500);
      servo_RightMotor.writeMicroseconds(1500);

    }
    //If something (either L, M or R)
    else if ((ui_Left_On_Yellow) || (ui_Middle_On_Yellow) || (ui_Right_On_Yellow)) {

      // If L&M or M&R
      if ((ui_Left_On_Yellow && ui_Middle_On_Yellow) || (ui_Middle_On_Yellow && ui_Right_On_Yellow)) {
        servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
        servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
        //return 0;
      }
      //If L
      else if (ui_Left_On_Yellow) {
        Serial.println("Left");
        servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed + 50);
        servo_LeftMotor.writeMicroseconds(1500);
        prevDirection = 0;
        //return 1;
      }

      //If M
      else if (ui_Middle_On_Yellow) {
        Serial.println("Middle");
        servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
        servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
        //return 0;
      }

      //If R
      else if (ui_Right_On_Yellow) {
        Serial.println("Right");
        servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
        servo_RightMotor.writeMicroseconds(1500);
        prevDirection = 1;
        //return 2;
      }
    }

  }
}


void moveArmSlowly(int initialPosition, int finalPosition) {
  if (initialPosition < finalPosition) {
    for (int i = initialPosition; i < finalPosition; i++) {
      servo_ArmMotor.write(i);
      delay(20);
    }
  }
  else if (initialPosition > finalPosition) {
    for (int i = initialPosition; i > finalPosition; i--) {
      servo_ArmMotor.write(i);
      delay(20);
    }
  }
}

void stop() {
  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1500);
}
