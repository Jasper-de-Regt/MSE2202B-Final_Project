// turns on an LED using serial commands 'n' and 'f'
// usefull for testing bluetooth, etc

char blueToothVal;           //value sent over via bluetooth
char lastValue;              //stores last state of device (on/off)
 
void setup()
{
 Serial.begin(9600); //pour a bowl of serial
 pinMode(13,OUTPUT);
}
 
 
void loop()
{
  if(Serial.available())
  {//if there is data being recieved
    blueToothVal=Serial.read(); //read it
  }
  if (blueToothVal=='n')
  {//if value from bluetooth serial is n
    digitalWrite(13,HIGH);            //switch on LED
    if (lastValue!='n')
      Serial.println(F("LED is on")); //print LED is on
    lastValue=blueToothVal;n
  }
  else if (blueToothVal=='f')
  {//if value from bluetooth serial is n
    digitalWrite(13,LOW);             //turn off LED
    if (lastValue!='f')
      Serial.println(F("LED is off")); //print LED is on
    lastValue=blueToothVal;
  }
  delay(1000);
}

