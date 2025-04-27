/*
 * motors.h
 *
 *  Created on: Feb 26, 2025
 *      Author: brady
 */

#include "motors.h"

// Helper function to send a throttle command (in microseconds)
void Motor_SetThrottle(uint32_t Channel, uint16_t microseconds)
{
    if (microseconds < MIN_THROTTLE_US) microseconds = MIN_THROTTLE_US;
    if (microseconds > MAX_THROTTLE_US) microseconds = MAX_THROTTLE_US;

    __HAL_TIM_SET_COMPARE(&htim1, Channel, microseconds);
}

void Motors_Arm(void){
	Motors_Start();

	// Send minimum throttle to arm the ESC
	Motor_SetThrottle(TIM_CHANNEL_1, MIN_THROTTLE_US);
	Motor_SetThrottle(TIM_CHANNEL_2, MIN_THROTTLE_US);
	Motor_SetThrottle(TIM_CHANNEL_3, MIN_THROTTLE_US);
	Motor_SetThrottle(TIM_CHANNEL_4, MIN_THROTTLE_US);
	HAL_Delay(MOTOR_ARM_TIME_MS); // wait 2 seconds while ESC arms

	// Set a higher throttle
	Motor_SetThrottle(TIM_CHANNEL_1, 180);
	Motor_SetThrottle(TIM_CHANNEL_2, 180);
	Motor_SetThrottle(TIM_CHANNEL_3, 180);
	Motor_SetThrottle(TIM_CHANNEL_4, 180);
	HAL_Delay(MOTOR_ARM_TIME_MS);

	// Set Motors to 0
	Motor_SetThrottle(TIM_CHANNEL_1, MIN_THROTTLE_US);
	Motor_SetThrottle(TIM_CHANNEL_2, MIN_THROTTLE_US);
	Motor_SetThrottle(TIM_CHANNEL_3, MIN_THROTTLE_US);
	Motor_SetThrottle(TIM_CHANNEL_4, MIN_THROTTLE_US);
	HAL_Delay(1000);
}

// Function to stop PWM
void Motors_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
}

void Motors_Start(){
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // Start PWM on Channel 1
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // Start PWM on Channel 2
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  // Start PWM on Channel 3
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  // Start PWM on Channel 4
}

void set_motor_speeds(float throttle, float roll_correction, float pitch_correction, float yaw_correction){
	float m1 = throttle + pitch_correction + roll_correction - yaw_correction;
	float m2 = throttle + pitch_correction - roll_correction + yaw_correction;
	float m3 = throttle - pitch_correction + roll_correction + yaw_correction;
	float m4 = throttle - pitch_correction - roll_correction - yaw_correction;

	// Constrain values to safe PWM range
	if (m1 > MAX_THROTTLE_US) m1 = MAX_THROTTLE_US; else if (m1 < MIN_THROTTLE_US) m1 = MIN_THROTTLE_US;
	if (m2 > MAX_THROTTLE_US) m2 = MAX_THROTTLE_US; else if (m2 < MIN_THROTTLE_US) m2 = MIN_THROTTLE_US;
	if (m3 > MAX_THROTTLE_US) m3 = MAX_THROTTLE_US; else if (m3 < MIN_THROTTLE_US) m3 = MIN_THROTTLE_US;
	if (m4 > MAX_THROTTLE_US) m4 = MAX_THROTTLE_US; else if (m4 < MIN_THROTTLE_US) m4 = MIN_THROTTLE_US;

	static uint32_t motor_pwm[4];
	motor_pwm[0] = (uint32_t)m1;
	motor_pwm[1] = (uint32_t)m2;
	motor_pwm[2] = (uint32_t)m3;
	motor_pwm[3] = (uint32_t)m4;

	// Send values to ESCs
	Motor_SetThrottle(TIM_CHANNEL_1, motor_pwm[0]);
	Motor_SetThrottle(TIM_CHANNEL_2, motor_pwm[1]);
	Motor_SetThrottle(TIM_CHANNEL_3, motor_pwm[2]);
	Motor_SetThrottle(TIM_CHANNEL_4, motor_pwm[3]);
}
