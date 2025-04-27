/*
 * sensors.h
 *
 *  Created on: Feb 26, 2025
 *      Author: brady
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "stm32f4xx.h"

#define MPU6050_ADDRESS (0x68 << 1)
#define BMP180_ADDRESS (0x77 << 1)
#define HMC5883L_ADDRESS (0x1E << 1)

uint32_t millis(void);

void init_Mag(I2C_HandleTypeDef *hi2c);
void read_Mag(I2C_HandleTypeDef *hi2c, float *mag_x, float *mag_y, float *mag_z);

void init_Baro(I2C_HandleTypeDef *hi2c);
void read_Baro(I2C_HandleTypeDef *hi2c, float *temperature, float *pressure);

void init_IMU(I2C_HandleTypeDef *hi2c);
void read_IMU(I2C_HandleTypeDef *hi2c, float *accel_x, float *accel_y, float *accel_z, float *gyro_x, float *gyro_y, float *gyro_z);

void init_sensors(I2C_HandleTypeDef *hi2c);

#endif /* INC_SENSORS_H_ */
