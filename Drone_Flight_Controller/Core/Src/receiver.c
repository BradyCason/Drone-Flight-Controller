/*
 * receiver.c
 *
 *  Created on: Apr 6, 2025
 *      Author: brady
 */

#include "receiver.h"

float scaled(float value, float old_min, float old_max, float new_min, float new_max){
	return (value - old_min) * (new_max - new_min)  / (old_max - old_min) + new_min;
}

void receive_command(uint8_t sbusData[], float *desired_roll, float *desired_pitch, float *desired_yaw, float *desired_throttle) {
	uint16_t channels[16];

	channels[0]  = (sbusData[1] | (sbusData[2] << 8)) & 0x07FF;
	channels[1]  = ((sbusData[2] >> 3) | (sbusData[3] << 5)) & 0x07FF;
	channels[2]  = ((sbusData[3] >> 6) | (sbusData[4] << 2) | (sbusData[5] << 10)) & 0x07FF;
	channels[3]  = ((sbusData[5] >> 1) | (sbusData[6] << 7)) & 0x07FF;
	channels[4]  = ((sbusData[6] >> 4) | (sbusData[7] << 4)) & 0x07FF;
	channels[5]  = ((sbusData[7] >> 7) | (sbusData[8] << 1) | (sbusData[9] << 9)) & 0x07FF;
	channels[6]  = ((sbusData[9] >> 2) | (sbusData[10] << 6)) & 0x07FF;
	channels[7]  = ((sbusData[10] >> 5) | (sbusData[11] << 3)) & 0x07FF;
	channels[8]  = (sbusData[12] | (sbusData[13] << 8)) & 0x07FF;
	channels[9]  = ((sbusData[13] >> 3) | (sbusData[14] << 5)) & 0x07FF;
	channels[10] = ((sbusData[14] >> 6) | (sbusData[15] << 2) | (sbusData[16] << 10)) & 0x07FF;
	channels[11] = ((sbusData[16] >> 1) | (sbusData[17] << 7)) & 0x07FF;
	channels[12] = ((sbusData[17] >> 4) | (sbusData[18] << 4)) & 0x07FF;
	channels[13] = ((sbusData[18] >> 7) | (sbusData[19] << 1) | (sbusData[20] << 9)) & 0x07FF;
	channels[14] = ((sbusData[20] >> 2) | (sbusData[21] << 6)) & 0x07FF;
	channels[15] = ((sbusData[21] >> 5) | (sbusData[22] << 3)) & 0x07FF;

	// Read failsafe & frame lost flags
	uint8_t failsafe = sbusData[23] & 0x08;
	uint8_t frame_lost = sbusData[23] & 0x04;

	// Process commands and normalize around 0
	*desired_roll = scaled(channels[0], 0, 2047, ROLL_MIN, ROLL_MAX);
	*desired_pitch = scaled(channels[1], 0, 2047, PITCH_MAX, PITCH_MAX);
	*desired_throttle = scaled(channels[2], 0, 2047, THROTTLE_MIN, THROTTLE_MAX);
	*desired_yaw = scaled(channels[3], 0, 2047, YAW_MIN, YAW_MAX);
}
