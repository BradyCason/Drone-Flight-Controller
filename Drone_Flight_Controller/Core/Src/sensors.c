/*
 * sensors.c
 *
 *  Created on: Feb 26, 2025
 *      Author: brady
 */

#include "sensors.h"

HAL_StatusTypeDef result;

void init_IMU(I2C_HandleTypeDef *hi2c)
{
	uint8_t mpu_config = 0x00;
	uint8_t mpu_set_sample_rate = 0x07;
	uint8_t mpu_set_fs_range = 0x00;
	uint8_t clockSource = 0x01;

	// wake up sensor
	HAL_I2C_Mem_Write(hi2c, MPU6050_ADDRESS, 0x6B, 1,&mpu_config, 1, 1000);

	// set sample rate to 1kHz, config ranges
	HAL_I2C_Mem_Write(hi2c, MPU6050_ADDRESS, 0x19, 1, &mpu_set_sample_rate, 1, 1000);
	HAL_I2C_Mem_Write(hi2c, MPU6050_ADDRESS, 0x1B, 1, &mpu_set_fs_range, 1, 1000);
	HAL_I2C_Mem_Write(hi2c, MPU6050_ADDRESS, 0x1c, 1, &mpu_set_fs_range, 1, 1000);
	HAL_I2C_Mem_Write(hi2c, MPU6050_ADDRESS, 0x6B, I2C_MEMADD_SIZE_8BIT, &clockSource, 1, HAL_MAX_DELAY);
}

void read_IMU(I2C_HandleTypeDef *hi2c, float *accel_x, float *accel_y, float *accel_z, float *gyro_x, float *gyro_y, float *gyro_z) {
	uint8_t imu_addr = 0x3B;
	uint8_t gyro_addr = 0x43;
	HAL_StatusTypeDef mpu_ret;
	uint8_t mpu_buf[6];
	int16_t raw_accel_x;
	int16_t raw_accel_y;
	int16_t raw_accel_z;
	int16_t raw_gyro_x = 0;
	int16_t raw_gyro_y = 0;
	int16_t raw_gyro_z = 0;

	mpu_ret = HAL_I2C_IsDeviceReady(hi2c, MPU6050_ADDRESS, 3, 5);
    if (mpu_ret == HAL_OK){
		mpu_ret = HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDRESS, &imu_addr, 1, 100);
		if ( mpu_ret == HAL_OK ) {
			mpu_ret = HAL_I2C_Master_Receive(hi2c, MPU6050_ADDRESS, mpu_buf, 6, 100);
			if ( mpu_ret == HAL_OK ) {
				// shift first byte left, add second byte
				raw_accel_x = (int16_t)(mpu_buf[0] << 8 | mpu_buf[1]);
				raw_accel_y = (int16_t)(mpu_buf[2] << 8 | mpu_buf[3]);
				raw_accel_z = (int16_t)(mpu_buf[4] << 8 | mpu_buf[5]);

				// get float values in g
				*accel_x = raw_accel_x/16384.0;
				*accel_y = raw_accel_y/16384.0;
				*accel_z = raw_accel_z/16384.0;
			}
		}

		mpu_ret = HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDRESS, &gyro_addr, 1, 100);
		if ( mpu_ret == HAL_OK ) {
			mpu_ret = HAL_I2C_Master_Receive(hi2c, MPU6050_ADDRESS, mpu_buf, 6, 100);
			if ( mpu_ret == HAL_OK ) {
				// shift first byte left, add second byte
				raw_gyro_x = (int16_t)(mpu_buf[0] << 8 | mpu_buf [1]);
				raw_gyro_y = (int16_t)(mpu_buf[2] << 8 | mpu_buf [3]);
				raw_gyro_z = (int16_t)(mpu_buf[4] << 8 | mpu_buf [5]);

				// convert to deg/sec
				*gyro_x = raw_gyro_x/131.0;
				*gyro_y = raw_gyro_y/131.0;
				*gyro_z = raw_gyro_z/131.0;
			}
		}
    }
}

void init_Baro(I2C_HandleTypeDef *hi2c){
	uint8_t ctrl_meas = 0x34; // Temperature and Pressure control register
	uint8_t cal_data[22]; // To store calibration data

	// Read calibration data (22 bytes)
	HAL_I2C_Mem_Read(hi2c, BMP180_ADDRESS, 0xAA, I2C_MEMADD_SIZE_8BIT, cal_data, 22, HAL_MAX_DELAY);

	// Send control register to start temperature and pressure measurement
	HAL_I2C_Mem_Write(hi2c, BMP180_ADDRESS, 0xF4, 1, &ctrl_meas, 1, 1000);
}
void read_Baro(I2C_HandleTypeDef *hi2c, float *temperature, float *pressure){
	uint8_t temp_addr = 0xF6; // Address for temperature/pressure data
	uint8_t data[3];
	int32_t raw_temp, raw_pressure;
	uint16_t dig_T1, dig_P1;

	// Read raw temperature
	result = HAL_I2C_Mem_Read(hi2c, BMP180_ADDRESS, temp_addr, I2C_MEMADD_SIZE_8BIT, data, 3, 1000);
	raw_temp = (data[0] << 8) | data[1];  // Combine bytes

	// Read raw pressure
	HAL_I2C_Mem_Read(hi2c, BMP180_ADDRESS, temp_addr, I2C_MEMADD_SIZE_8BIT, data, 3, 1000);
	raw_pressure = (data[0] << 8) | data[1];  // Combine bytes

	// Assuming compensation formulas exist or you have the calibration data
	*temperature = (float)raw_temp / 10.0;  // Example conversion (depends on calibration)
	*pressure = (float)raw_pressure / 100.0;  // Example conversion
}

void init_Mag(I2C_HandleTypeDef *hi2c){
	uint8_t mode = 0x00; // Continuous measurement mode

	// Write to mode register to start continuous measurement
	HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDRESS, 0x02, 1, &mode, 1, 1000);
}

void read_Mag(I2C_HandleTypeDef *hi2c, float *mag_x, float *mag_y, float *mag_z){
	uint8_t mag_addr = 0x03;  // Address for magnetometer data
	uint8_t data[6];
	int16_t raw_mag_x, raw_mag_y, raw_mag_z;

	// Read 6 bytes of magnetometer data (3 axis)
	HAL_I2C_Mem_Read(hi2c, HMC5883L_ADDRESS, mag_addr, I2C_MEMADD_SIZE_8BIT, data, 6, 1000);

	// Combine the bytes for each axis
	raw_mag_x = (int16_t)(data[0] << 8 | data[1]);
	raw_mag_y = (int16_t)(data[2] << 8 | data[3]);
	raw_mag_z = (int16_t)(data[4] << 8 | data[5]);

	// Conversion to appropriate units (depends on the sensor's configuration)
	*mag_x = (float)raw_mag_x * 0.92;  // Example scale factor for HMC5883L (depends on gain)
	*mag_y = (float)raw_mag_y * 0.92;
	*mag_z = (float)raw_mag_z * 0.92;
}

void init_sensors(I2C_HandleTypeDef *hi2c){
	init_IMU(hi2c);
//	init_Mag(hi2c);
	init_Baro(hi2c);
}
