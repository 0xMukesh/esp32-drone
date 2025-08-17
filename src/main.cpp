#include <Arduino.h>

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  if (Serial.available() > 0)
  {
    String data = Serial.readStringUntil('\n');
    data.trim();

    if (data.length() > 0)
    {
      Serial.printf("DEBUG:recieved from webots: %s\n", data);

      if (data == "UP")
      {
        Serial.printf("DATA:5,5,5,5\n");
      }
    }

    delay(10);
  }
}
