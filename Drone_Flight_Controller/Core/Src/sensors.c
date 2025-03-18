/*
 * sensors.c
 *
 *  Created on: Feb 26, 2025
 *      Author: brady
 */

#include "sensors.h"

extern volatile uint32_t millisCounter;
uint32_t millis(void) {
    return millisCounter;
}

void read_IMU(){

}
void read_mag(){

}
void read_Barometer(){

}

void read_sensors(){
	uint32_t currentTime = millis();

	if (currentTime - last_IMU_read >= IMU_cycle_length) {
		read_IMU();
		last_IMU_read = currentTime;
	}

	if (currentTime - last_mag_read >= mag_cycle_length) {
		read_mag();
		last_mag_read = currentTime;
	}

	if (currentTime - last_baro_read >= baro_cycle_length) {
	    read_Barometer();
	    last_baro_read = currentTime;
	}
}
