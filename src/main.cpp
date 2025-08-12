#include <Arduino.h>
#include <WiFi.h>

#include <secrets.hpp>

void setup()
{
  Serial.begin(9600);
  WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("connecting to wifi...");
    delay(1000);
  }

  Serial.println("connected to wifi");
}

void loop()
{
}
