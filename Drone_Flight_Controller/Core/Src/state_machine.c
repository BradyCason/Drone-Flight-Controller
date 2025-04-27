/*
 * state_machine.c
 *
 *  Created on: Apr 6, 2025
 *      Author: brady
 */

#include "state_machine.h"

unsigned char IMU_ticks;
const unsigned char IMU_desired_ticks = 1;
unsigned char Mag_ticks;
const unsigned char Mag_desired_ticks = 100;
unsigned char Baro_ticks;
const unsigned char Baro_desired_ticks = 20;

extern volatile unsigned char IMU_ready;
extern volatile unsigned char Mag_ready;
extern volatile unsigned char Baro_ready;

extern volatile unsigned char armed;

mainSM_states mainSM_state = INIT;

void mainSM_tick(void){
	// Transition States
	switch(mainSM_state){
		case(INIT):
			mainSM_state = IDLE;
			break;
		case(IDLE):
			if (armed){
				mainSM_state = ARMED;
				IMU_ticks = IMU_desired_ticks;
				Mag_ticks = Mag_desired_ticks;
				Baro_ticks = Baro_desired_ticks;
			}
			else{
				mainSM_state = IDLE;
			}
			break;
		case(ARMED):
			if (armed){
				mainSM_state = ARMED;
				IMU_ticks++;
				Mag_ticks++;
				Baro_ticks++;
			}
			else{
				mainSM_state = IDLE;
			}
			break;
		default:
			// Error: Invalid State
			break;
	}

	// Perform State Actions
	switch(mainSM_state){
		case(IDLE):
			break;
		case(ARMED):
			if (IMU_ticks >= IMU_desired_ticks){
				IMU_ready = 1;
				IMU_ticks = 0;
			}
			if (Mag_ticks >= Mag_desired_ticks){
				Mag_ready = 1;
				Mag_ticks = 0;
			}
			if (Baro_ticks >= Baro_desired_ticks){
				Baro_ready = 1;
				Baro_ticks = 0;
			}
			break;
		default:
			// Error: Invalid State
			break;
	}
}
