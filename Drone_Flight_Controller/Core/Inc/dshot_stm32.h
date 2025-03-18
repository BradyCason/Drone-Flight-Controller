/*
 * dshot_stm32.h
 *
 *  Created on: Mar 17, 2025
 *      Author: brady
 */

#ifndef INC_DSHOT_STM32_H_
#define INC_DSHOT_STM32_H_

void DSHOT_Send(uint16_t throttle) {
    uint16_t packet = (throttle << 5) | (0b0000);  // Add CRC later
    for (int i = 0; i < 16; i++) {
        dshotBuffer[i] = (packet & (1 << (15 - i))) ? htim1.Init.Period * 3 / 4 : htim1.Init.Period / 2;
    }

    HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)dshotBuffer, DSHOT_FRAME_SIZE);
}

#endif /* INC_DSHOT_STM32_H_ */
