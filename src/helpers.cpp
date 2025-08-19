#include <./include/drone/helpers.hpp>
#include <Arduino.h>

namespace Helpers
{
    PIDController::PIDController(float kp, float ki, float kd, float sp)
        : kp(kp), ki(ki), kd(kd), setpoint(sp), previous_setpoint(sp), setpoint_changed(false)
    {
        reset();
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

        integral += error * dt;

        // clamp the integral term more aggressively during transition changes i.e. when setpoint changes
        if (setpoint_changed)
        {
            integral = constrain(integral, INTEGRAL_MIN * 0.5, INTEGRAL_MAX * 0.5);
            setpoint_changed = false;
        }
        else
        {
            integral = constrain(integral, INTEGRAL_MIN, INTEGRAL_MAX);
        }

        float p_part = kp * error;
        float i_part = ki * integral;
        float d_part = 0;

        // calculate d-part only after first iteration
        if (previous_time > 0)
        {
            d_part = -kd * (current_value - previous_measurement) / dt;
        }

        float output = p_part + i_part + d_part;

        float unconstrained_output = output;
        output = constrain(output, OUTPUT_MIN, OUTPUT_MAX);

        // if the unconstrained output is saturated and the target altitude has not been reached, then adding the integral contribution would make it worse
        if ((unconstrained_output > OUTPUT_MAX && error > 0) ||
            (unconstrained_output < OUTPUT_MIN && error < 0))
        {
            integral -= error * dt;
        }

        previous_measurement = current_value;
        previous_time = current_time;

        return output;
    }

    void PIDController::setSetpoint(float sp)
    {
        if (abs(sp - setpoint) > 0.01)
        {
            setpoint_changed = true;

            if (abs(sp - setpoint) > 0.5)
            {
                integral *= 0.3;
            }
            else
            {
                integral *= 0.7;
            }
        }

        previous_setpoint = setpoint;
        setpoint = sp;
    }

    float PIDController::getSetpoint()
    {
        return setpoint;
    }

    void PIDController::reset()
    {
        previous_measurement = 0.0;
        integral = 0.0;
        previous_time = millis();
        setpoint_changed = false;
    }

    void PIDController::resetIntegral()
    {
        integral = 0.0;
    }
}