#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>

#include <secrets.hpp>

struct DroneState
{
  float throttle;
  float pitch;
  float roll;
  float yaw;
  bool emergencyStop;
};

WebSocketsServer ws = WebSocketsServer(80);

void wsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_DISCONNECTED:
    Serial.printf("[%u] disconnected\n", num);
    break;
  case WStype_CONNECTED:
  {
    IPAddress ip = ws.remoteIP(num);
    Serial.printf("[%u] connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
  }
  break;
  case WStype_TEXT:
    Serial.printf("[%u] recieved text: %s\n", num, payload);
    ws.sendTXT(num, "recieved - " + String((char *)(payload)));
    break;
  }
}

void setup()
{
  Serial.begin(9600);
  WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("connecting to wifi...");
    delay(1000);
  }

  Serial.printf("connected to wifi - %s\n", WiFi.localIP().toString());

  ws.begin();
  ws.onEvent(wsEvent);
}

void loop()
{
  ws.loop();
}
