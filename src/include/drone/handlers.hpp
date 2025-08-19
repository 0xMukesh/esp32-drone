#pragma once

#include <Arduino.h>
#include <vector>

#include <./include/drone/helpers.hpp>

namespace Handlers
{
    enum DroneState
    {
        DISARMED,
        ARMED,
        FLYING
    };

    class DroneHandler
    {
    private:
        Helpers::PIDController altitude_pid = Helpers::PIDController(8, 3, 6, 0);

        const float MOTOR_MIN_SPEED = 45;
        const float MOTOR_MAX_SPEED = 150;
        const float BASE_HOVER_SPEED = 55;

        const unsigned long PID_UPDATE_INTERVAL = 20; // every 20 ms

        // reduces PID output whenever the target is close and in deadband range
        const float ALTITUDE_DEADBAND = 0.05;
        // avoids large changes when up arrow key or down arrow key is held for a long time
        // while causes issues with PID
        // with rate limiting, the altitude is smoothly ramps up
        const float MAX_SETPOINT_CHANGE_RATE = 2.0;

        unsigned long last_pid_update = 0;

        DroneState drone_state = DISARMED;

        void sendMotorSpeeds(float speeds[4]);

    public:
        DroneHandler();

        void handleArm();
        void handleLand();

        void updateSetpoint(const std::vector<String> &tokens);
        void handleUpdateAltitude(const std::vector<String> &tokens);
    };
}