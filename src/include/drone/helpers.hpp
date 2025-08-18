#pragma once

namespace Helpers
{
    class PIDController
    {
    private:
        float kp, ki, kd;
        float setpoint;
        float previous_error;
        float integral;
        unsigned long previous_time;

        float INTEGRAL_MAX = 25.0;
        float INTEGRAL_MIN = -25.0;

        float OUTPUT_MAX = 40.0;
        float OUTPUT_MIN = -40.0;

    public:
        PIDController(float kp, float ki, float kd, float sp);

        float update(float current_value);
        void setSetpoint(float sp);
        float getSetpoint();
        void reset();
    };
}
