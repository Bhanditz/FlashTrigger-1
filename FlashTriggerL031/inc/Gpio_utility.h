/*
 * Gpio_utility.h
 *
 *  Created on: 24. 8. 2016
 *      Author: priesolv
 */

#ifndef GPIO_UTILITY_H_
#define GPIO_UTILITY_H_

#include "stm32l0xx.h"
#include <stdbool.h>

typedef void (*Ptr_OnAdcConv)(uint16_t nValue);
typedef void (*Ptr_OnTimer)();

void Gpio_Init(void);
void Gpio_OptoInit(Ptr_OnAdcConv pOnAdcConv);
void Gpio_DisableADC();

void Gpio_TimoutTimerConfig_ms(uint32_t nTime_ms, Ptr_OnTimer pOnTimer);
void Gpio_TimoutTimerState(uint8_t nEnable);

bool Gpio_IsMaster();
void Gpio_LedBlink(uint16_t nDuration_ms);
void Gpio_FlashBlink();
void Gpio_LedOffDiming();
uint16_t Gpio_IsButtonPressed_ms();
void Gpio_Off(void);
void Gpio_StandbyMode(void);

void Gpio_SysTickCallback();

#endif /* GPIO_UTILITY_H_ */
