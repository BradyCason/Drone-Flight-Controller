/*
 * motors.h
 *
 *  Created on: Feb 26, 2025
 *      Author: brady
 */

#include "motors.h"

void set_motor_speed(float roll_correction, float pitch_correction, float yaw_correction){
	float m1 = throttle + pitch_correction + roll_correction - yaw_correction;
	float m2 = throttle + pitch_correction - roll_correction + yaw_correction;
	float m3 = throttle - pitch_correction + roll_correction + yaw_correction;
	float m4 = throttle - pitch_correction - roll_correction - yaw_correction;

	// Constrain values to safe PWM range (1000µs - 2000µs)
	if (m1 > 2000) m1 = 2000; else if (m1 < 1000) m1 = 1000;
	if (m2 > 2000) m2 = 2000; else if (m2 < 1000) m2 = 1000;
	if (m3 > 2000) m3 = 2000; else if (m3 < 1000) m3 = 1000;
	if (m4 > 2000) m4 = 2000; else if (m4 < 1000) m4 = 1000;

	// Send values to ESCs
	MOTOR1_PIN = m1;
	MOTOR2_PIN = m2;
	MOTOR3_PIN = m3;
	MOTOR4_PIN = m4;
}
