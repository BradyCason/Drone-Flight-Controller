/*
 * state_machine.h
 *
 *  Created on: Apr 6, 2025
 *      Author: brady
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_

typedef enum {INIT, IDLE, ARMED} mainSM_states;
void mainSM_tick(void);

#endif /* INC_STATE_MACHINE_H_ */
