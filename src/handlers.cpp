#include <./include/drone/handlers.hpp>

namespace Handlers
{
    DroneHandler::DroneHandler()
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

    void DroneHandler::handleArm()
    {
        if (drone_state == DroneState::DISARMED)
        {
            drone_state = DroneState::ARMED;
            altitude_pid.reset();
            float speeds[4] = {BASE_HOVER_SPEED, BASE_HOVER_SPEED, BASE_HOVER_SPEED, BASE_HOVER_SPEED};
            sendMotorSpeeds(speeds);
        }
    }

    void DroneHandler::handleLand()
    {
        drone_state = DroneState::DISARMED;
        altitude_pid.reset();
        float speeds[4] = {0.0, 0.0, 0.0, 0.0};
        sendMotorSpeeds(speeds);
    }

    void DroneHandler::updateSetpoint(const std::vector<String> &tokens)
    {
        if (drone_state == DroneState::ARMED || drone_state == DroneState::FLYING)
        {
            float new_setpoint = tokens[1].toFloat();
            float current_setpoint = altitude_pid.getSetpoint();
            float setpoint_change = new_setpoint - current_setpoint;

            if (abs(setpoint_change) > MAX_SETPOINT_CHANGE_RATE)
            {
                if (setpoint_change > 0)
                {
                    new_setpoint = current_setpoint + MAX_SETPOINT_CHANGE_RATE;
                }
                else
                {
                    new_setpoint = current_setpoint - MAX_SETPOINT_CHANGE_RATE;
                }
            }

            altitude_pid.setSetpoint(new_setpoint);
            drone_state = DroneState::FLYING;
        }
    }

    void DroneHandler::handleUpdateAltitude(const std::vector<String> &tokens)
    {
        unsigned long current_time = millis();

        if (drone_state == DroneState::FLYING && (current_time - last_pid_update >= PID_UPDATE_INTERVAL))
        {
            float current_altitude = tokens[1].toFloat();
            float pid_output = altitude_pid.update(current_altitude);
            float error = altitude_pid.getSetpoint() - current_altitude;

            // when it is close to the target, reduce the output a bit
            if (abs(error) < ALTITUDE_DEADBAND)
            {
                pid_output *= 0.5;
            }

            float motor_speed = BASE_HOVER_SPEED + pid_output;
            motor_speed = constrain(motor_speed, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);

            float speeds[4] = {motor_speed, motor_speed, motor_speed, motor_speed};
            sendMotorSpeeds(speeds);

            last_pid_update = current_time;
        }
    }
}