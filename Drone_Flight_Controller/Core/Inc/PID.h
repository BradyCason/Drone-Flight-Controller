/*
 * PID.h
 *
 *  Created on: Feb 26, 2025
 *      Author: brady
 */

#ifndef SRC_PID_H_
#define SRC_PID_H_

typedef struct {
    float Kp, Ki, Kd;  // PID coefficients
    float previous_error;
    float integral;
    float output;
    float min_output, max_output;
} PIDController;

void PID_Init(PIDController *pid, float Kp, float Ki, float Kd, float min_output, float max_output);
float PID_Compute(PIDController *pid, float setpoint, float measured_value, float dt);

#endif /* SRC_PID_H_ */
