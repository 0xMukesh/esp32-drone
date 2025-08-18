#include <./include/drone/handlers.hpp>

namespace Handlers
{
    DroneHandler::DroneHandler(std::vector<String> tokens) : tokens(tokens)
    {
    }

    void DroneHandler::sendMotorSpeeds(float speeds[4])
    {
        String msg = "MOTOR:";

        for (int i = 0; i < 4; i++)
        {
            float speed = speeds[i] * pow(-1, i + 1);
            msg += String(speed, 4);

            if (i < 3)
            {
                msg += ",";
            }
        }

        Serial.println(msg);
    };

    void DroneHandler::handleUpdateAltitude()
    {
        unsigned long current_time = millis();

        if (current_time - last_pid_update >= PID_UPDATE_INTERVAL && motors_active)
        {
            float pid_output = altitude_pid.update(tokens[1].toFloat());
            float motor_speed = BASE_HOVER_SPEED + pid_output;

            float speeds[4] = {motor_speed, motor_speed, motor_speed, motor_speed};
            sendMotorSpeeds(speeds);
        }
    }

    void DroneHandler::updateSetpoint()
    {
        float new_setpoint = tokens[1].toFloat();
        altitude_pid.setSetpoint(new_setpoint);
        motors_active = true;
    }
}
