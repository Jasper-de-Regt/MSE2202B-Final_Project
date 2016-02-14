


#include <PID_v1.h>

#include <Servo.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_rightMotor;
I2CEncoder encoder_rightMotor;

// port pin constants
const int ci_rightMotor = 8;          // to right motor ESC
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
  pinMode(ci_rightMotor, OUTPUT);
  servo_rightMotor.attach(ci_rightMotor);


  // set up encoders. Must be initialized in order that they are chained together,
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  encoder_rightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_rightMotor.setReversed(false);  // adjust for positive count when moving forward

  //initiliaze pid variables
  input = encoder_rightMotor.getPosition();
  setpoint = 30;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(1570, 2000);

  pinMode(proportionalPotPin, INPUT);
  pinMode(derivativePotPin, INPUT);
  pinMode(integralPotPin, INPUT);
}

//*********************************************************************************************************************************************************************************************************************************************************************//
//*********************************************************************************************************************************************************************************************************************************************************************//

void loop() {



  kp = map(analogRead(proportionalPotPin), 0, 1023, 0, 50);
  ki = map(analogRead(integralPotPin), 0, 1023, 0, 50);
  kd = map(analogRead(derivativePotPin), 0, 1023, 0, 50);
  myPID.SetTunings(kp, ki, kd);


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



  input = encoder_rightMotor.getSpeed();
  myPID.Compute();
  servo_rightMotor.writeMicroseconds(output);
  CharliePlexM::Write(1, HIGH);

}
