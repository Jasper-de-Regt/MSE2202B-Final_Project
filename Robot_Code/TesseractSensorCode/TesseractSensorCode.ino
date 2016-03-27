#include <EEPROM.h>

const unsigned int sensorHalfDelta = 130;

const int numberOfSensors = 2;
const int eepromAddress[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
unsigned int thresholdValues[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// pin constants
const unsigned int analogPins[16] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15};
const unsigned int tesseractDetectedPin = 13;
const unsigned int buttonPin = 14;
const unsigned int ledPin = 13;


bool tesseractDetected = false;              // this bool is modified by sensor read call. If a tesseract is found, this bool will be set to true


//**********************************************************************************************************************************************************************************************//
//**********************************************************************************************************************************************************************************************//
//**********************************************************************************************************************************************************************************************//

void setup() {
  // set sensor pins as inputs
  for (int i = 0; i < numberOfSensors; i++) {
    pinMode(analogPins[i], INPUT);
  }

  // set other pins
  pinMode(tesseractDetectedPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);        // the button should be wired as a pull down button, the internal pullup resistor has been activated
  if (!digitalRead(buttonPin)) {
    calibrateSensors();
  }

  // populates the thresholdValues[16] array with values from EEPROM
  populateThresholdsFromEeprom();
}

//**********************************************************************************************************************************************************************************************//
//**********************************************************************************************************************************************************************************************//
//**********************************************************************************************************************************************************************************************//

void loop() {
  // put your main code here, to run repeatedly:
  tesseractDetected = false;                                  // initial condition
  readSensors();                                              // this function reads all the sensors and sets tesseractDetected to true if a sensor detects a tesseract
  digitalWrite(tesseractDetectedPin, tesseractDetected);      // if a tesseract was found, this will set tesseractDetectedPin to HIGH.
}


//**********************************************************************************************************************************************************************************************//
//**********************************************************************************************************************************************************************************************//
//**********************************************************************************************************************************************************************************************//

// reads the stored EEPROM values (BYTES), scales them from 0-255 to 0-1023, and stores that to the thresholdValues array to be used while code is running
void populateThresholdsFromEeprom() {
  for (int i = 0; i < numberOfSensors; i++) {
    thresholdValues[i] = (EEPROM.read(eepromAddress[i])) * 4;
  }
}

//**********************************************************************************************************************************************************************************************//
//**********************************************************************************************************************************************************************************************//
//**********************************************************************************************************************************************************************************************//

// read the sensors, set tesseractDetected to true if any sensor is over threshold
void readSensors() {
  for (int i = 0; i < numberOfSensors; i++) {

    analogRead(analogPins[i]);    // threw this in here to give the ADC time to settle before an important reading, maybe this line isn't required.

    unsigned int reading = 0;
    for (int j = 0; j < 3; j++) {                        // read sensor 3x, sum values
      reading = reading + analogRead(analogPins[i]);
    }
    reading = reading / 3;                              // average

    if (reading < (thresholdValues[i])) {
      tesseractDetected = true;
    }
  }
}
// NOTES: tesseracts make the sensor return a lower analogRead() value. The value is roughly 2*sensorHalfDelta lower than a normal reading

//**********************************************************************************************************************************************************************************************//
//**********************************************************************************************************************************************************************************************//
//**********************************************************************************************************************************************************************************************//

// this function calibrates the sensors. It creates new threshold values and stores those in eeprom.
void calibrateSensors() {
  unsigned int newThresholdValues[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // create an array to hold sensor reading data

  // this block does the actual sensor reading.
  // it will read the sensor 10x, take the average value, and store that average value in the array
  for (int i = 0; i < numberOfSensors; i++) {
    analogRead(analogPins[i]);                                                      // just a garbage measurement to give ADC time to settle, probably not needed
    for (int j = 0; j < 10; j++) {                                                  // read sensor 10 times, sum the values
      newThresholdValues[i] = newThresholdValues[i] + analogRead(analogPins[i]);
    }
    newThresholdValues[i] = newThresholdValues[i] / 10;                            //average
  }

  // this loop prepares the avg sensor values for storage
  // it will subtract the sensorhalfdelta value from each sensor value to create a threshold
  // then it will divide the value by 4, which essentially moves the range from 0-1023 to 0-255 (8 byte EEPROM)
  for (int i = 0; i < numberOfSensors; i++) {
    newThresholdValues[i] = (newThresholdValues[i] - sensorHalfDelta) / 4;
  }

  // this loop burns the array values to EEPROM
  for (int i = 0; i < numberOfSensors; i++) {
    EEPROM.write(eepromAddress[i], newThresholdValues[i]);
  }
  // NOTES: all the above for loops could be combined into one. This is my first time working with EEPROM and I will have
  // to explain this code to group members. Breaking it up like makes it easier to explain.

  // quick blinky lights to show that calibration has finished
  bool ledstate;
  for (int i = 0; i < 40; i++) {
    digitalWrite(ledPin,  ledstate);
    ledstate = !ledstate;
    delay(50);
  }
  digitalWrite(ledPin,  LOW);      // ensure led is off
}

//**********************************************************************************************************************************************************************************************//
//**********************************************************************************************************************************************************************************************//
//**********************************************************************************************************************************************************************************************//
