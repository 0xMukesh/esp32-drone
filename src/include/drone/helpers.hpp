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

        float INTEGRAL_MAX = 15.0;
        float INTEGRAL_MIN = -15.0;

        float OUTPUT_MAX = 30.0;
        float OUTPUT_MIN = -30.0;

    public:
        PIDController(float kp, float ki, float kd, float sp);

        float update(float current_value);
        void setSetpoint(float sp);
        float getSetpoint();
        void reset();
    };
}
