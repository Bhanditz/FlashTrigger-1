/*
 * spirit1_app.h
 *
 *  Created on: 21. 6. 2016
 *      Author: Priesol Vladimir
 */

#ifndef SPIRIT1_APP_H_
#define SPIRIT1_APP_H_

#include "stm32l0xx.h"
#include <stdbool.h>
#include "SPIRIT_Config.h"

/* Exported constants --------------------------------------------------------*/


typedef enum
{
  APP_STATE_IDLE = 0,
  APP_STATE_PROG,
  APP_STATE_START,
  APP_STATE_MANUAL_TRIGGER,
  APP_STATE_START_RX,
  APP_STATE_WAIT_FOR_RX_DONE,
  APP_STATE_DATA_RECEIVED,
  APP_STATE_SEND_CHECK,
  APP_STATE_SEND_FLASH,
  APP_STATE_WAIT_FOR_TX_DONE_CHECK,
  APP_STATE_WAIT_FOR_TX_DONE_FLASH,
} AppState_t;

typedef enum
{
  APP_MODE_OPTO = 0,
  APP_MODE_MANUAL,
  APP_MODE_PRG,
} AppMode_t;

typedef struct
{
  uint8_t Cmdtag;
  uint8_t CmdType;
  uint8_t CmdLen;
  uint8_t Cmd;
  uint8_t DataLen;
  uint8_t* DataBuff;
}AppliFrame_t;

void App_Exec(void);
void App_Init();
void App_SendBuffer(uint8_t* pBuffer, uint8_t nLength);
void App_ReceiveBuffer(uint8_t *RxFrameBuff, uint8_t cRxlen);
void OnSpiritInterruptHandlerSlaveSniffer(void);
void OnSpiritInterruptHandlerSlave(void);
void OnSpiritInterruptHandlerMaster(void);
bool App_CheckFlash(void);
void Programming();
void App_WaitAfterFlash(void);
void App_ADCGetConv(uint16_t nValue);
void App_OnTimeoutTimer();
void App_SniffConfigure(void);
void App_PrepareSniffing();

#endif /* SPIRIT1_APP_H_ */
