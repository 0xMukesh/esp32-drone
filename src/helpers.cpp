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

        // Proportional term
        float p_part = kp * error;

        // Integral term with windup protection
        // Only accumulate integral if we're not saturated
        integral += error * dt;

        // More aggressive integral clamping during setpoint changes
        if (setpoint_changed)
        {
            integral = constrain(integral, INTEGRAL_MIN * 0.5, INTEGRAL_MAX * 0.5);
            setpoint_changed = false;
        }
        else
        {
            integral = constrain(integral, INTEGRAL_MIN, INTEGRAL_MAX);
        }

        float i_part = ki * integral;

        // Derivative term - calculate from measurement to avoid derivative kick
        float d_part = 0;
        if (previous_time > 0)
        { // Only calculate derivative after first iteration
            d_part = -kd * (current_value - previous_measurement) / dt;
        }

        float output = p_part + i_part + d_part;

        // Check for output saturation and prevent integral windup
        float unconstrained_output = output;
        output = constrain(output, OUTPUT_MIN, OUTPUT_MAX);

        // Anti-windup: if output is saturated, don't let integral grow further in that direction
        if ((unconstrained_output > OUTPUT_MAX && error > 0) ||
            (unconstrained_output < OUTPUT_MIN && error < 0))
        {
            integral -= error * dt; // Remove the integral contribution we just added
        }

        previous_error = error;
        previous_measurement = current_value;
        previous_time = current_time;

        return output;
    }

    void PIDController::setSetpoint(float sp)
    {
        if (abs(sp - setpoint) > 0.01)
        { // Only flag as changed if significant difference
            setpoint_changed = true;

            // For large setpoint changes, reduce integral more aggressively
            if (abs(sp - setpoint) > 0.5)
            {
                integral *= 0.3; // Reduce integral to 30% of current value
            }
            else
            {
                integral *= 0.7; // Reduce integral to 70% of current value
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
        previous_error = 0.0;
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