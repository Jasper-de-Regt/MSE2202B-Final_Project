//    Code written by Jasper de Regt on 22/01/2016
//    This code was created to interface a standard RC receiver (or microcontroller sending standard RC signals)
//    to an L298 dual H bridge motor driver module. An arduino serves as the translating device between the RC
//    Receiver and the L298 Module.
//
//    The combined Arduino and L298 module essentially turn into a 2 channel RC ESC. The idea is to use a ~$3
//    Arduino pro mini and to treat the arduino/L298 as a single device that is easy to interface with
//    Note: an arduino has PWM pins 3, 5, 6, 9, 10, and 11

//#define TESTINGMODE;    // leaving this uncommented will enable some serial comments

// Pin connections to L298 Dual H Bridge Module
const unsigned int rightPwmPin = 3;           // Right side PWM pin
const unsigned int rightFwdEnablePin = 4;     // Right side forward enable pin
const unsigned int rightRevEnablePin = 5;     // Right side reverse enable pin
const unsigned int leftPwmPin = 9;            // Left side PWM pin
const unsigned int leftFwdEnablePin = 8;      // Left side forward enable pin
const unsigned int leftRevEnablePin = 7;      // Left side reverse enable pin
// Pin connections to RC receiver
const unsigned int rightRcInputSignalPin = 10;    // connect to an RC receiver, this will control the right side ESC
const unsigned int leftRcInputSignalPin = 11;     // connect to an RC receiver, this will control the left side ESC
// other pin connections
const unsigned int led13 = 13;                    // connects to an led, installed on pcb on most arduinos

// other Global variables
int leftSideRcInputSignal = 1500;           // holds a microsecond value received from rc transmitter, typically in the 1000-2000 range with 1500 being at rest
int rightSideRcInputSignal = 1500;          // holds a microsecond value received from rc transmitter, typically in the 1000-2000 range with 1500 being at rest
int rMotorSpeed = 0;                        // value for right side pwm value (0-255)
int lMotorSpeed = 0;                        // value for left side pwm value (0-255)
const unsigned int deadBand = 50;           // A tolerance for dealing with RcInputSignal s

bool led13State = false;                                 // a bool that is used to turn led13 on and off
unsigned long previousled13StateChangeMillis = 0;        // tracks the last time that the LED changed states
unsigned int heartBeatInterval = 0;                  // sets the heartbeat blink rate

void setup() {
  // Pin connections to L298 Dual H Bridge Module are output pins
  pinMode(rightPwmPin, OUTPUT);
  pinMode(rightFwdEnablePin, OUTPUT);
  pinMode(rightRevEnablePin, OUTPUT);
  pinMode(leftPwmPin, OUTPUT);
  pinMode(leftFwdEnablePin, OUTPUT);
  pinMode(leftRevEnablePin, OUTPUT);
  // set pin 13 (connected to an LED on most arduinos) to output
  pinMode(led13, OUTPUT);
  // Pin connections to RC receiver are input pins
  pinMode(rightRcInputSignalPin, INPUT);
  pinMode(leftRcInputSignalPin, INPUT);

#ifdef TESTINGMODE
  Serial.begin(9600); // Pour a bowl of serial for troubleshooting
#endif
}

void loop() {
  // put your main code here, to run repeatedly:
  readRcSignalsFromReceiver();      // reads channels and sets a value from ~1000 to ~2000 to variable leftSideRcInputSignal, rightSideRcInputSignal
  failsafe();                       // stops both motors if a radio signal is not present
  writeCommandsToL298Module();      // writes commands to L298 dual H bridge module
  heartBeat();                      // calls heartbeat function, blinks led13

#ifdef TESTINGMODE
  safety();                         // makes sure h bridges are never shorted. This function is useless and wastes memory. It is here for code testing purposes. You can leave it commented.
  Serial.println();
  Serial.print("leftSideRcInputSignal = ");
  Serial.print(leftSideRcInputSignal);
  Serial.print("\t");
  Serial.print("lMotorSpeed = ");
  Serial.print(lMotorSpeed);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("rightSideRcInputSignal = ");
  Serial.print(rightSideRcInputSignal);
  Serial.print("\t");
  Serial.print("rMotorSpeed = ");
  Serial.print(rMotorSpeed);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("\t");
#endif
} // end of loop

//************************************************************************************************************************************************************//

void writeCommandsToL298Module() {
  // sets motor speed to rc stick position
  lMotorSpeed = abs(leftSideRcInputSignal - 1500) / 500.0 * 255;
  rMotorSpeed = abs(rightSideRcInputSignal - 1500) / 500.0 * 255;
  // some RC transmitter/receivers do not like outputting a full 1000 or 2000 microsecond signal, and some output more like 800-2200
  // if a motor speed was over 100%, this will set it to 100%. This prevents overflow issues for controllers than output outside of 1000-2000 microseconds (motors would slow down at RC transmitter stick extremes)
  // if a motor speed was close to 100%, this will change it to 100%. This allows 100% power for transmitters that work slightly inside the 1000-2000 range
  if (lMotorSpeed > 250) {
    lMotorSpeed = 255;
  }
  if (rMotorSpeed > 250) {
    rMotorSpeed = 255;
  }

  // LEFT SIDE
  // if the signal is at 1500 +/- deadband, set speed to 0 and disable direction enables.
  // this results in a motor coast situation
  if ((leftSideRcInputSignal < (1500 + deadBand)) && (leftSideRcInputSignal > (1500 - deadBand))) {
    digitalWrite(leftFwdEnablePin, LOW);
    digitalWrite(leftRevEnablePin, LOW);
    lMotorSpeed = 0;
    analogWrite(leftPwmPin, lMotorSpeed);
  }
  // if the signal is greater than 1500, move motor fwd at speed
  else  if (leftSideRcInputSignal > (1500 + deadBand)) {
    digitalWrite(leftFwdEnablePin, HIGH);
    digitalWrite(leftRevEnablePin, LOW);
    analogWrite(leftPwmPin, lMotorSpeed);
  }
  // if the signal is greater than 1500, move motor rev at speed
  else if (leftSideRcInputSignal < (1500 - deadBand)) {
    digitalWrite(leftFwdEnablePin, LOW);
    digitalWrite(leftRevEnablePin, HIGH);
    analogWrite(leftPwmPin, lMotorSpeed);
  }

  // RIGHT SIDE
  // if the signal is at 1500 +/- deadband, set speed to 0 and disable direction enables.
  // this results in a motor coast situation
  if ((rightSideRcInputSignal < (1500 + deadBand)) && (rightSideRcInputSignal > (1500 - deadBand))) {
    digitalWrite(rightFwdEnablePin, LOW);
    digitalWrite(rightRevEnablePin, LOW);
    rMotorSpeed = 0;
    analogWrite(rightPwmPin, rMotorSpeed);
  }
  // if the signal is greater than 1500, move motor fwd at speed
  else  if (rightSideRcInputSignal > (1500 + deadBand)) {
    digitalWrite(rightFwdEnablePin, HIGH);
    digitalWrite(rightRevEnablePin, LOW);
    analogWrite(rightPwmPin, rMotorSpeed);
  }
  // if the signal is greater than 1500, move motor rev at speed
  else  if (rightSideRcInputSignal < (1500 - deadBand)) {
    digitalWrite(rightFwdEnablePin, LOW);
    digitalWrite(rightRevEnablePin, HIGH);
    analogWrite(rightPwmPin, rMotorSpeed);
  }
  return;
}

//************************************************************************************************************************************************************//

// this function reads the RC receiver signal and sets rightSideRcInputSignal/leftSideRcInputSignal to a microsecond value
// generally the microsecond value would be in the range of 1000-2000, but some RC radios may go slightly outside this range
// If radio signal is not present, this will read 0 with a Turnigy 9x and frSky radio
void readRcSignalsFromReceiver() {
  rightSideRcInputSignal = pulseIn(rightRcInputSignalPin, HIGH, 25000);
  leftSideRcInputSignal = pulseIn(leftRcInputSignalPin, HIGH, 25000);
  return;
}

//************************************************************************************************************************************************************//

// this function makes sure that a radio signal is present.
// If a radio signal is not present then readRcSignalsFromReceiver() will read 0.
// this should stop both motors and set the heartbeat to blink very quickly
// if one channel loses signal; both motors should stop.
void failsafe() {
  // if signal is outside of the normal range; set the signal to neutral
  if ((rightSideRcInputSignal < 700) || (leftSideRcInputSignal < 700)) {
    leftSideRcInputSignal = 1500;
    rightSideRcInputSignal = 1500;
    heartBeatInterval = 10;
#ifdef TESTINGMODE
    Serial.print("SIGNAL LOST! IN FAILSAFE MODE!");
#endif
  }
  else {
    heartBeatInterval = 900;
  }
  return;
}

//************************************************************************************************************************************************************//

// this function will blink led 13. You can change the blink rate by changing heartBeatInterval
// a slow beat indicates normal operation
// a quick beat indicates failsafe mode
void heartBeat() {
  if ((millis() - previousled13StateChangeMillis) > heartBeatInterval) {
    previousled13StateChangeMillis = millis();
    led13State = !led13State;
    digitalWrite(led13, led13State);
  }
  return;
}

//************************************************************************************************************************************************************//

#ifdef TESTINGMODE
// this function ensures an h bridge is never shorted.
// For example, enabling both right side direction pins AND sending a pwm signal would short out that H bridge.
// This function will ensure that if an enable pin is HIGH, its compliment is LOW
// note that this function means it is not possible to brake the motor. Braking would be setting both enables to HIGH and sending a PWM of 0.
// this function is here for code testing purposes only. Slight paranoia is a good way to prevent hardware damage.
// This function should not be called inside finished code
void safety() {
  int i = 0;
  if (leftFwdEnablePin == HIGH) {
    digitalWrite(leftRevEnablePin, LOW);
    i++;
  }
  if (leftRevEnablePin == HIGH) {
    digitalWrite(leftFwdEnablePin, LOW);
    i++;
  }
  if (rightFwdEnablePin == HIGH) {
    digitalWrite(rightRevEnablePin, LOW);
    i++;
  }
  if (rightRevEnablePin == HIGH) {
    digitalWrite(rightFwdEnablePin, LOW);
    i++;
  }
  if (i != 0) {
    Serial.println("EXTREME Danger: the program entered an if statement inside the safety loop! THIS SHORTS OUT THE H BRIDGE");
  }
  return;
}
#endif
