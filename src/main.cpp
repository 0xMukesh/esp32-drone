#include <Arduino.h>

#include <./include/drone/utils.hpp>
#include <./include/drone/handlers.hpp>

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
      std::vector<String> tokens = Utils::splitString(data, ':');

      if (tokens.size() >= 1)
      {
        Handlers::DroneHandler droneHandler = Handlers::DroneHandler(tokens);

        if (tokens[0] == "ALT")
        {
          droneHandler.handleUpdateAltitude();
        }
        else if (tokens[0] == "SET")
        {
          droneHandler.updateSetpoint();
        }
      }
    }
  }
}
