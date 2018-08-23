/*
 * timer.h
 *
 *  Created on: 24. 6. 2016
 *      Author: priesolv
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "stm32l0xx.h"

typedef void(*PtrSysTickCallback) (void);

void Timer_Init();
void Timer_Delay_ms(uint32_t delay_ms);
uint32_t Timer_GetTicks_ms();
void Timer_SetSysTickCallback(PtrSysTickCallback pFunction);


void TimerUs_init(void);
void TimerUs_start(void);
uint16_t TimerUs_get_microseconds(void);
void TimerUs_delay(uint16_t microseconds);
void TimerUs_clear(void);
void TimerUs_stop(void);
#endif /* TIMER_H_ */
