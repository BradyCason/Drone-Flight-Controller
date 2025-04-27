/*
 * motors.h
 *
 *  Created on: Feb 26, 2025
 *      Author: brady
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "stm32f4xx.h"

#define MIN_THROTTLE_US 125   // Oneshot125 minimum pulse width (motor off)
#define MAX_THROTTLE_US 250   // Oneshot125 maximum pulse width (full speed)

#define MOTOR_ARM_TIME_MS 2000 // 2 seconds of arming

extern TIM_HandleTypeDef htim1;

void Motor_SetThrottle(uint32_t Channel, uint16_t microseconds);
void Motors_Arm(void);
void Motors_Stop(void);
void Motors_Start();
void set_motor_speeds(float throttle, float roll_correction, float pitch_correction, float yaw_correction);

#endif /* INC_MOTORS_H_ */
