
//#define rawData;
#define trueFalse;

int threshold = 800;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  for (int i = 14; i <= 19; i++) {
    pinMode(i, INPUT);
  }
}

void loop() {
  // put your main code here, to run repeatedly:



#ifdef rawData
  Serial.println();
  for (int i = 14; i <= 19; i++) {
    Serial.print(analogRead(i));
    Serial.print("   ");
  }
#endif

#ifdef trueFalse
  Serial.println();
  for (int i = 14; i <= 19; i++) {
    if ((analogRead(i)) < threshold) {
      Serial.print("T");
      Serial.print("   ");
    }
    else {
      Serial.print(".");
      Serial.print("   ");
    }
  }
#endif

}
