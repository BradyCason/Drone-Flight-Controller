/*
 * receiver.h
 *
 *  Created on: Apr 6, 2025
 *      Author: brady
 */

#ifndef INC_RECEIVER_H_
#define INC_RECEIVER_H_

#include "stm32f4xx_hal.h"

#define MAX_MOTOR_SPEED 2047

#define MIN_ESC_PWM 1000
#define MAX_ESC_PWM 2000

#define ROLL_MIN -200
#define ROLL_MAX 200
#define PITCH_MIN -200
#define PITCH_MAX 200
#define YAW_MIN -200
#define YAW_MAX 200
#define THROTTLE_MIN 500
#define THROTTLE_MAX 500

float scaled(float value, float old_min, float old_max, float new_min, float new_max);

void receive_command(uint8_t sbusData[], float *desired_roll, float *desired_pitch, float *desired_yaw, float *desired_throttle);

#endif /* INC_RECEIVER_H_ */
