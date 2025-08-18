#pragma once

#include <Arduino.h>
#include <vector>

#include <./include/drone/helpers.hpp>

namespace Handlers
{
    class DroneHandler
    {
    private:
        std::vector<String> tokens;
        Helpers::PIDController altitude_pid = Helpers::PIDController(8.0, 1.5, 3.5, 0.0);

        const float MOTOR_MIN_SPEED = 45;
        const float MOTOR_MAX_SPEED = 70;
        const float BASE_HOVER_SPEED = 60;

        const unsigned long PID_UPDATE_INTERVAL = 20; // every 20ms

        unsigned long last_pid_update = 0;
        bool motors_active = true;

        void sendMotorSpeeds(float speeds[4]);

    public:
        DroneHandler(std::vector<String> tokens);

        void handleUpdateAltitude();
        void updateSetpoint();
    };
}
