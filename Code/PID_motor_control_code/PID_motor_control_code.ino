//#include <PID_AutoTune_v0.h>





#include <PID_v1.h>
#include <Servo.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_rightMotor;
I2CEncoder encoder_rightMotor;

// port pin constants
const int ci_enable = 9;          // to enable
const int ci_fwd = 8;          // to in2
const int ci_rev = 10;         // to in1

const int ci_Charlieplex_LED1 = 4;    //     CharliePlexM::Write(ci_Middle_Line_Tracker_LED, HIGH);
const int ci_Charlieplex_LED2 = 5;
const int ci_Charlieplex_LED3 = 6;
const int ci_Charlieplex_LED4 = 7;
const int ci_Mode_Button = 7;
const int ci_I2C_SDA = A4;             // I2C data = white
const int ci_I2C_SCL = A5;             // I2C clock = yellow
const int proportionalPotPin = A0;
const int derivativePotPin = A1;
const int integralPotPin = A2;


// variables
double setpoint, input, output;
double kp, ki, kd;
long previousMillis = 0;


PID myPID(&input, &output, &setpoint, 1, 0, 0, DIRECT);

//*********************************************************************************************************************************************************************************************************************************************************************//
//*********************************************************************************************************************************************************************************************************************************************************************//

void setup() {

  Wire.begin();	      // Wire library required for I2CEncoder library
  Serial.begin(9600);

  // setup charlieplex leds and pushbutton
  CharliePlexM::setBtn(ci_Charlieplex_LED1, ci_Charlieplex_LED2, ci_Charlieplex_LED3, ci_Charlieplex_LED4, ci_Mode_Button);

  // set up drive motors
  pinMode(ci_enable, OUTPUT);
  pinMode(ci_fwd, OUTPUT);
  pinMode(ci_rev, OUTPUT);

  // set up encoders. Must be initialized in order that they are chained together,
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  encoder_rightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  //  encoder_rightMotor.setReversed(false);  // adjust for positive count when moving forward

  //initiliaze pid variables
  input = encoder_rightMotor.getPosition();
  setpoint = 1;

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);


  pinMode(proportionalPotPin, INPUT);
  pinMode(derivativePotPin, INPUT);
  pinMode(integralPotPin, INPUT);
}

//*********************************************************************************************************************************************************************************************************************************************************************//
//*********************************************************************************************************************************************************************************************************************************************************************//

void loop() {


  if (CharliePlexM::ui_Btn)
  {
    setpoint++;
  }















  // set charlieplex light on so you know code is uploaded
  CharliePlexM::Write(1, HIGH);

  // set pid values based on potentiometer values
  kp = map(analogRead(proportionalPotPin), 0, 1023, 0, 5000);
  ki = map(analogRead(integralPotPin), 0, 1023, 0, 5000);
  kd = map(analogRead(derivativePotPin), 0, 1023, 0, 10);
  myPID.SetTunings(kp, ki, kd);

  //print pid values, input, output, and setpoint to serial
  Serial.println();
  Serial.print(myPID.GetKp());
  Serial.print("   ");
  Serial.print(myPID.GetKi());
  Serial.print("   ");
  Serial.print(myPID.GetKd());
  Serial.print("   ");
  Serial.print(input);
  Serial.print("   ");
  Serial.print(output);
  Serial.print("   ");
  Serial.print(setpoint);



  // getSpeed is a piece of shit; it takes 4 secodns to return 0 after the motor stops
  // pid control of speed
  //  input = encoder_rightMotor.getSpeed();
  //getVelocityBits isn't any better
  //  input = encoder_rightMotor.getVelocityBits();

  input = encoder_rightMotor.getPosition();
  myPID.Compute();

  // output is a value from 0-255
  // set motor speed to a direction


  //  output=map(analogRead(derivativePotPin),0,1023,0,1000);
  int deadband = 30;
  
  if (output == 0) {
    //do nothing, brake maybe?
  }

  else  if (output > 0) {
    digitalWrite(ci_fwd, HIGH);
    digitalWrite(ci_rev, LOW);
    int speed = map(output, 0, 255, deadband, 255); //map(output,500,1000,0,255);
    analogWrite(ci_enable, speed);
    Serial.print("       dir  ");
    Serial.print("fwd");
    Serial.print(" speed  ");
    Serial.print(speed);
  }
  else {
    digitalWrite(ci_fwd, LOW);
    digitalWrite(ci_rev, HIGH);
    int speed = map(output, 0, -255, deadband, 255);
    //    int speed = map(output, 500, 0, 0, 255);
    analogWrite(ci_enable, speed);
    Serial.print("       dir  ");
    Serial.print("rev");
    Serial.print(" speed  ");
    Serial.print(speed);
  }



}
