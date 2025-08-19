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

        const unsigned long PID_UPDATE_INTERVAL = 20;

        const float ALTITUDE_DEADBAND = 0.05;
        const float MAX_SETPOINT_CHANGE_RATE = 2.0;

        unsigned long last_pid_update = 0;

        DroneState drone_state = DISARMED;

        void sendMotorSpeeds(float speeds[4]);

    public:
        DroneHandler();

        void handleUpdateAltitude(const std::vector<String> &tokens);
        void updateSetpoint(const std::vector<String> &tokens);

        void handleArm();
        void handleLand();
    };
}