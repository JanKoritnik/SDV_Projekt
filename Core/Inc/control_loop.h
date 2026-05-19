/*
 * control_loop.h
 *
 *  20 ms TIM2 control loop: BNO read → CyberGear → DDSM115
 */

#ifndef INC_CONTROL_LOOP_H_
#define INC_CONTROL_LOOP_H_

#include "main.h"

/* Per-section execution times in microseconds (updated every tick) */
extern volatile uint32_t dt_bno;
extern volatile uint32_t dt_cg;
extern volatile uint32_t dt_ddsm;
extern volatile uint32_t dt_total;

/* Set to 1 by ISR when LOOP_DURATION_MS expires — main() must call motors stop */
extern volatile uint8_t stop_request;

void Control_Loop_Init(void);

#endif /* INC_CONTROL_LOOP_H_ */
