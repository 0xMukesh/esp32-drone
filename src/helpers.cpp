#include <./include/drone/helpers.hpp>
#include <Arduino.h>

namespace Helpers
{
    PIDController::PIDController(float kp, float ki, float kd, float sp) : kp(kp), ki(ki), kd(kd), setpoint(sp)
    {
    }

    float PIDController::update(float current_value)
    {
        unsigned long current_time = millis();
        float dt = (current_time - previous_time) / 1000.0;

        if (dt <= 0.001)
        {
            dt = 0.001;
        }

        float error = setpoint - current_value;

        float p_part = kp * error;

        integral += error * dt;
        integral = constrain(integral, INTEGRAL_MIN, INTEGRAL_MAX);
        float i_part = ki * integral;

        float d_part = kd * (error - previous_error) / dt;

        float output = p_part + i_part + d_part;
        output = constrain(output, OUTPUT_MIN, OUTPUT_MAX);

        previous_error = error;
        previous_time = current_time;

        return output;
    }

    void PIDController::setSetpoint(float sp)
    {
        setpoint = sp;
        integral = 0.0;
    }

    float PIDController::getSetpoint()
    {
        return setpoint;
    }

    void PIDController::reset()
    {
        previous_error = 0.0;
        integral = 0.0;
        previous_time = millis();
    }
}
