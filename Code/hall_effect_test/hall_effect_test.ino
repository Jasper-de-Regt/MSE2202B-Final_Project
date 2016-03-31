unsigned long previousMillis = 0;
const long interval = 100;

void setup()
{
  Serial.begin(9600);
}

void DoMeasurement()
{
  // measure magnetic field
  int raw = analogRead(3);   // Range : 0..1024

  //  Uncomment this to get a raw reading for calibration of no-field point
  Serial.print("Raw reading: ");
  Serial.println(raw);

  //long compensated = raw - 508; //- NOFIELD;                 // adjust relative to no applied field
  // long gauss = compensated; //* TOMILLIGAUSS / 1000;   // adjust scale to Gauss

  //Serial.print(gauss);
  //Serial.print(" Gauss ");

  //if (gauss > 0)     Serial.println("(South pole)");
  //else if (gauss < 0) Serial.println("(North pole)");
  //else               Serial.println();
}

void loop()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    DoMeasurement();
  }
}
