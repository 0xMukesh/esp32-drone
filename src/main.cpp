#include <Arduino.h>

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  String data = Serial.readString();
  Serial.println(data);
  delay(10);
}
