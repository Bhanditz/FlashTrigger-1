/*
 * spirit_gpio.h
 *
 *  Created on: 24. 8. 2016
 *      Author: priesolv
 */

#ifndef SPIRIT_H_
#define SPIRIT_H_


#include "stm32l0xx.h"
#include <stdbool.h>
#include "SPIRIT_Types.h"
#include "SPIRIT_Calibration.h"

#define USE_SPIRIT1_915MHz

typedef void(*Ptr_OnGPIO3_EXTI)(void);

void Spirit_Init(Ptr_OnGPIO3_EXTI pOnGPIO3Exti);

void Spirit_EnterShutdown(void);
void Spirit_ExitShutdown(void);

void Spirit_EnableIRQ(void);
void Spirit_DisableIRQ(void);

void Spirit_WriteReg(uint8_t nRegAddr, uint8_t nValue);
void Spirit_WriteCommand(uint8_t nCommand, SpiritState state);
uint8_t Spirit_ReadReg(uint8_t nRegAddr);
void Spirit_ReadRegs(uint8_t nRegAddr, uint8_t nLenght, uint8_t *pBuffer);

void Spirit_InitRegs(bool bMaster);
void Spirit_SetFrequency();
void Spirit_Calibrate(bool bMaster);
void Spirit_SetPowerRegs(void);
void Spirit_ProtocolInitRegs(void);
void Spirit_EnableSQI(void);
void Spirit_SetRssiThreshold(void);

#endif /* SPIRIT_H_ */
