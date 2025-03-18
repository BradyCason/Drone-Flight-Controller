/*
 * PID.c
 *
 *  Created on: Feb 26, 2025
 *      Author: brady
 */


#include "pid.h"

void PID_Init(PIDController *pid, float Kp, float Ki, float Kd, float min_output, float max_output) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->previous_error = 0;
    pid->integral = 0;
    pid->min_output = min_output;
    pid->max_output = max_output;
}

float PID_Compute(PIDController *pid, float setpoint, float measured_value, float dt) {
    float error = setpoint - measured_value;
    pid->integral += error * dt;
    float derivative = (error - pid->previous_error) / dt;

    float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

    // Clamp output
    if (output > pid->max_output) output = pid->max_output;
    if (output < pid->min_output) output = pid->min_output;

    pid->previous_error = error;
    return output;
}
