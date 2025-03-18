/*
 * sensors.h
 *
 *  Created on: Feb 26, 2025
 *      Author: brady
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "stm32f4xx.h"

uint32_t last_IMU_read = 0;
uint32_t last_baro_read = 0;
uint32_t last_mag_read = 0;
uint32_t IMU_cycle_length = 2;
uint32_t baro_cycle_length = 50;
uint32_t mag_cycle_length = 50;

extern volatile uint32_t millisCounter;

float roll;
float pitch;
float yaw;
float mag_heading;
float pressure;
float altitude;

uint32_t millis();
void read_IMU();
void read_mag();
void read_Barometer();
void read_sensors();

#endif /* INC_SENSORS_H_ */
