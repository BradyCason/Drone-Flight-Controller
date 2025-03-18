/*
 * motors.h
 *
 *  Created on: Feb 26, 2025
 *      Author: brady
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "stm32f4xx.h"

extern TIM_HandleTypeDef htim1;

#define MOTOR1_PIN TIM1->CCR1
#define MOTOR2_PIN TIM1->CCR2
#define MOTOR3_PIN TIM1->CCR3
#define MOTOR4_PIN TIM1->CCR4

float throttle = 1500;

void set_motor_speed(float roll_correction, float pitch_correction, float yaw_correction);

#endif /* INC_MOTORS_H_ */
