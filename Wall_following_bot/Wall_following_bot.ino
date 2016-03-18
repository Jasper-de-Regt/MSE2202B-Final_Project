
#include <PID_v1.h>
#include <Servo.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_rightMotor;
Servo servo_leftMotor;

//#define DEBUG_ULTRASONIC;

// port pin constants
const int rightServoPin = 8;
const int leftServoPin = 9;
const int ci_Ultrasonic_Data = 10;
const int ci_Ultrasonic_Ping = 11;


const int ci_Charlieplex_LED1 = 4;    //     CharliePlexM::Write(ci_Middle_Line_Tracker_LED, HIGH);
const int ci_Charlieplex_LED2 = 5;
const int ci_Charlieplex_LED3 = 6;
const int ci_Charlieplex_LED4 = 7;
const int ci_Mode_Button = 7;

unsigned long ul_Echo_Time;

// variables
double setpoint, input, output;
//double kp, ki, kd;

PID myPID(&input, &output, &setpoint, 1, 0, 0, DIRECT);

//*********************************************************************************************************************************************************************************************************************************************************************//
//*********************************************************************************************************************************************************************************************************************************************************************//

void setup() {

  Serial.begin(9600);

  // setup charlieplex leds and pushbutton
  CharliePlexM::setBtn(ci_Charlieplex_LED1, ci_Charlieplex_LED2, ci_Charlieplex_LED3, ci_Charlieplex_LED4, ci_Mode_Button);

  // set up drive motors
  pinMode(rightServoPin, OUTPUT);
  pinMode(leftServoPin, OUTPUT);
  pinMode(ci_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Ultrasonic_Data, INPUT);


servo_rightMotor.attach(rightServoPin);
servo_leftMotor.attach(leftServoPin);

  //initiliaze pid variables

  setpoint = 400;//***************************************************************//

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(20);
  myPID.SetOutputLimits(50,130);
}

//*********************************************************************************************************************************************************************************************************************************************************************//
//*********************************************************************************************************************************************************************************************************************************************************************//

void loop() {


Ping();

  // 400 seems to be a good ul_Echo_time to drive a reasonable distance away
input = ul_Echo_Time;
  myPID.Compute();

//servo_leftMotor.write(85);

//Serial.println(output);

servo_rightMotor.write(output);
servo_leftMotor.write(70);

Serial.println();
Serial.print("input  ");
Serial.print(input);
Serial.print("   output   ");
Serial.print(output);

}




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
  Serial.print("   HI MEGAN!!    "):
  Serial.print(", Inches: ");
  Serial.print(ul_Echo_Time / 148); //divide time by 148 to get distance in inches
  Serial.print(", cm: ");
  Serial.println(ul_Echo_Time / 58); //divide time by 58 to get distance in cm
#endif
}
