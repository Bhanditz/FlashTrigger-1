/*
 * spirit1_app.c
 *
 *  Created on: 21. 6. 2016
 *      Author: Priesol Vladimir
 */

#include "trigger_app.h"
#include "SPIRIT1_Util.h"
#include "spirit_spi.h"
#include "timer.h"
#include "spirit.h"
#include "Gpio_utility.h"
#include "Eeprom.h"

#include <string.h>

/**
* @brief GPIO structure fitting
*/
SGpioInit xGpioIRQ =
{
  SPIRIT_GPIO_3,
  SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,
  SPIRIT_GPIO_DIG_OUT_IRQ
};

#define CHECK_INTERVAL_MS            2500
#define FLASH_TRANSMIT_COUNT         3

#define FLASH_LIMIT_STD              2
#define FLASH_LIMIT_PRG              8

#define STD_OFF_INTERVAL_MS          (1000 * 60 * 3)   // off interval po zapnuti (bez signalu) - 5 minut
#define STD_OFF_INTERVAL_RCV_MS      (1000 * 60 * 15)  // off interval po prijeti signalu - 15 minut

#define PRG_OFF_INTERVAL_MS          (1000 * 60 * 1)    // 1 minuta

#define EEPROM_FLASH_INTERVAL        0                                             // interval mezi zablesky
#define EEPROM_FLASHES               (EEPROM_FLASH_INTERVAL + sizeof(uint32_t))    // pocet zablesku

uint8_t aCheckBroadcast[] = { 'C','H','K' };  // check
uint8_t aFlashBroadcast[] = { 'F','L','S' };  // flash
uint8_t RxBuffer[MAX_BUFFER_LEN];

SpiritIrqs xIrqStatus;

volatile FlagStatus g_RxDoneFlag = RESET;
volatile FlagStatus g_RXtimeout = RESET;
volatile FlagStatus g_RXdataDisc = RESET;

volatile FlagStatus g_TxDoneFlag = RESET;
volatile FlagStatus cmdFlag = RESET;
volatile FlagStatus xStartRx = RESET;

uint16_t exitCounter = 0;
uint8_t TxFrameBuff[MAX_BUFFER_LEN];

AppState_t g_eState = APP_STATE_IDLE;

uint8_t g_bMaster = 0;

volatile uint16_t g_nOptoValue;

volatile bool g_bFlashFlag;          // detekovan zablesk
volatile bool g_bFlashEnable = false;  // povoleni detekce zablesku

volatile uint16_t g_nDelayTimer;    // odmerovani intervalu
volatile bool g_bDelayOver;

volatile uint16_t g_nTimeCounter;       // citac poctu preruseni ADC (50 us)
volatile bool g_bTimeCounterStop;       // citat start/stop, po preteceni se sam vypne

volatile uint16_t g_nFlashLimit;               // citlivost detekce zablesku
uint16_t g_nFlashInterval;            // naprogramovana vzdalenost mezi zablesky
uint16_t g_nFlashes = 1;              // pocet naprogramovanych zablesku
uint8_t  g_nTransmitCounter;          // citani poctu vysilani pro FLASH

AppMode_t g_eMode;                // mode MASTERa

void App_Exec(void)
{
  uint8_t nLength;
  static uint32_t nLastCheckTime = 0;

  switch(g_eState)
  {
  case APP_STATE_START:  // reset
    NVIC_SystemReset();
    break;

  case APP_STATE_PROG:
    Programming();
    break;

  case APP_STATE_MANUAL_TRIGGER:  // rucni odpalovani SLAVE blesku
      //g_eState = APP_STATE_SEND_FLASH;
      break;

  case APP_STATE_START_RX:
    {
      App_ReceiveBuffer(RxBuffer, MAX_BUFFER_LEN);
      g_eState = APP_STATE_WAIT_FOR_RX_DONE;
    }
    break;

  case APP_STATE_WAIT_FOR_RX_DONE:
    if (g_RxDoneFlag)
    {
      // paket prijat
      g_RxDoneFlag = RESET;
      g_eState = APP_STATE_DATA_RECEIVED;
    }
    else if (g_RXtimeout || g_RXdataDisc)
    {
      // vyprsel timeout, nastavime novy prijem
      g_RXtimeout = RESET;
      g_RXdataDisc = RESET;
      g_eState = APP_STATE_START_RX;
    }

    break;

  case APP_STATE_DATA_RECEIVED:
    {
      Spirit1GetRxPacket(RxBuffer, &nLength);

      /* Kontrola sily signalu (pocet bliknuti by mohl byt: RSSI/30)
       *
       * vzdalenost 20cm: pozice paralelne RSSI=130
       *                         kolmo     RSSI=150
       * vzdalenost   2m: pozice paralelne RSSI=113
       *                         kolmo     RSSI=100
       * vzdalenost >30m: pozice paralelne RSSI= 55 (za rohem)
       *                         kolmo     RSSI=<50 (vypadava spojeni)
      */
//      uint8_t RSSIValue = SpiritQiGetRssi();

      g_eState = APP_STATE_START_RX;
      Gpio_SetOffInterval(STD_OFF_INTERVAL_RCV_MS);
      if (memcmp(RxBuffer, aCheckBroadcast, sizeof (aCheckBroadcast)) == 0)
      {
        Gpio_LedBlink(50);
      }
      else if (memcmp(RxBuffer, aFlashBroadcast, sizeof (aFlashBroadcast)) == 0)
      {
        // tahle sekvence se muze opakovat pro vsechny (3x) vysilaci pokusy mastera, ale blesk je stejne vybity
        Gpio_FlashBlink();
        Gpio_LedBlink(50);
      }
    }
    break;

  case APP_STATE_SEND_CHECK:
    {
      App_SendBuffer(aCheckBroadcast, sizeof (aCheckBroadcast));
      Gpio_LedBlink(100);
      g_eState = APP_STATE_WAIT_FOR_TX_DONE_CHECK;
    }
    break;

  case APP_STATE_SEND_FLASH:
      App_SendBuffer(aFlashBroadcast, sizeof (aFlashBroadcast));
      g_eState = APP_STATE_WAIT_FOR_TX_DONE_FLASH;
    break;

  case APP_STATE_WAIT_FOR_TX_DONE_FLASH:
    if(g_TxDoneFlag)
    {
      g_TxDoneFlag = RESET;
//      Spirit_Calibrate(g_bMaster);
      if (g_nTransmitCounter)
      {
        g_nTransmitCounter--;
        g_eState = APP_STATE_SEND_FLASH;
      }
      else
      {
        Gpio_LedBlink(100);
        App_WaitAfterFlash();
        g_eState = APP_STATE_IDLE;
      }
    }
    break;

  case APP_STATE_WAIT_FOR_TX_DONE_CHECK:
    if(g_TxDoneFlag)
    {
      g_TxDoneFlag = RESET;
      g_eState = APP_STATE_IDLE;
    }
    break;

  case APP_STATE_IDLE:
    if (g_bMaster)
    {
      if (nLastCheckTime + CHECK_INTERVAL_MS < Timer_GetTicks_ms())
      {
        g_eState = APP_STATE_SEND_CHECK;
        nLastCheckTime = Timer_GetTicks_ms();
      }
    }

    break;
  }

  // kontrola casovace vypnuti (do Standby modu)
  if (Gpio_GetOffTime() == 0)
  {
    Gpio_StandbyMode();
  }

  // kontrola stisku tlacitka
  if (Gpio_IsButtonPressed_ms())
  {
    // Todo: zde by se mel resit stisk tlacitka pri MANUAL modu
    while (!Gpio_IsButtonPressed_ms());
    Gpio_Off();
  }

  // vyhodnoceni zablesku (MASTER)
  if (g_bFlashFlag)
  {
    if (App_CheckFlash())
    {
      // naprogramovana sekvence souhlasi, posleme zablesk
      g_eState = APP_STATE_SEND_FLASH;
      g_nTransmitCounter = FLASH_TRANSMIT_COUNT;
      Gpio_SetOffInterval(STD_OFF_INTERVAL_RCV_MS);
    }
    else
    {
      g_eState = APP_STATE_IDLE;
    }
  }
}

void App_Init()
{
  Timer_Init();
  Gpio_Init();

  // zjistime, jestli jsme MASTER nebo SLAVE
  g_bMaster = Gpio_IsMaster();

  // povesime funkci na preruseni podle toho jestli budeme vysilat (MASTER) nebo prijimat (SLAVE)
  if (g_bMaster)
  {
    Spirit_Init(OnSpiritInterruptHandlerMaster);
    Gpio_SetOffInterval(STD_OFF_INTERVAL_RCV_MS);
  }
  else
  {
    // Spirit_Init(OnSpiritInterruptHandlerSlaveSniffer);
    Spirit_Init(OnSpiritInterruptHandlerSlave);
    Gpio_SetOffInterval(STD_OFF_INTERVAL_MS);
  }

  // cekat na uvolneni tlacitka a pro MASTER zmerime delku stisknuti
  uint32_t nStartTime = Timer_GetTicks_ms();
  while (Gpio_IsButtonPressed_ms());
  uint32_t nPressDuration = Timer_GetTicks_ms() - nStartTime;

  g_eMode = APP_MODE_OPTO;
  g_eState = APP_STATE_IDLE;
  if (g_bMaster)
  {
    if (nPressDuration > 1000)
    {
      g_eMode = APP_MODE_MANUAL;
      g_eState = APP_STATE_MANUAL_TRIGGER;
    }
    if (nPressDuration > 3000)
    {
      g_eMode = APP_MODE_PRG;
      g_eState = APP_STATE_PROG;
    }
  }
  else
  {
    g_eState = APP_STATE_START_RX;
  }

  // SPIRIT management
  Spirit_EnterShutdown();
  Spirit_ExitShutdown();

  // wait for SPIRIT RESET duration
  uint32_t nTime = Timer_GetTicks_ms();
  while (Timer_GetTicks_ms() - nTime < 2);

  SpiritManagementWaExtraCurrent();

//  uint8_t v;
//  StatusBytes sb;
//  sb = SpiritSpiReadRegisters(DEVICE_INFO1_PARTNUM, 1, &v);
//  sb = SpiritSpiReadRegisters(DEVICE_INFO0_VERSION, 1, &v);

  // wait for READY state and set XTAL frequency
  SpiritManagementIdentificationRFBoard();

  // Spirit IRQ config
  Spirit1GpioIrqInit(&xGpioIRQ);

  // Spirit Radio config
  Spirit_InitRegs(g_bMaster);

  Spirit_SetPowerRegs();  // Spirit Radio set power

  Spirit_BasicProtocolInit();

  // set SPIRIT1 IRQ
  SpiritIrqDeInit(NULL);
  SpiritIrqClearStatus();
  if (g_bMaster)
  {
    SpiritIrq(TX_DATA_SENT, S_ENABLE);
    SpiritIrq(TX_FIFO_ERROR, S_ENABLE);
    SpiritIrq(MAX_RE_TX_REACH, S_ENABLE);
  }
  else
  {
    SpiritIrq(RX_DATA_READY,S_ENABLE);
    SpiritIrq(RX_DATA_DISC, S_ENABLE);
    SpiritIrq(RX_TIMEOUT, S_ENABLE);
    SpiritIrq(RX_FIFO_ERROR, S_ENABLE);
//    SpiritIrq(VALID_SYNC,S_ENABLE);
  }

  // Todo: musi to tu byt pro master???
  // EnableSQI nastavuje reset hodnotu, tak by to tu nemuselo byt nai pro SLAVE
  // RSSI threshold je nastaven mnohem niže nez reset hodnota
  Spirit_EnableSQI();
  Spirit_SetRssiThreshold();

//  SRadioInit RadioInitStruct;
//  SpiritRadioGetInfo(&RadioInitStruct);

  // start blik
  Gpio_LedBlink(200);

  // pro MASTER nakonfigurovat optodiodu
  if (g_bMaster)
  {
    Gpio_OptoInit(App_ADCGetConv);
  }
  else
  {
//    App_SniffConfigure();
//    while(1);
  }

  // pro MASTER konfigurace opto snimani
  if (g_bMaster)
  {
    g_nTimeCounter = 0;
    g_bTimeCounterStop = false;
    g_bDelayOver = false;
    g_bFlashFlag = false;

    //g_bFlashEnable = false;
    g_nFlashLimit = FLASH_LIMIT_STD;

    g_bFlashEnable = true;
    g_nFlashInterval = Eeprom_ReadUint32(EEPROM_FLASH_INTERVAL);
    g_nFlashes = Eeprom_ReadUint32(EEPROM_FLASHES);
    if (g_nFlashes == 0)
    {
      g_nFlashes = 1;
    }

    if (g_nFlashes > 1)
    {
      uint8_t nCount = g_nFlashes - 1;
      while (nCount--)
      {
        Timer_Delay_ms(500);
        Gpio_LedBlink(200);
      }
    }
  }

  Spirit_EnableIRQ();
}

bool App_CheckFlash(void)
{
  uint16_t nFlashes = 1;

  if (g_nFlashes > 1)
  {
    // budeme pocitat vice nez jeden zablesk
    g_nTimeCounter = 0;
    g_bTimeCounterStop = false;

    while (nFlashes < g_nFlashes)
    {
      App_WaitAfterFlash();
      while (!g_bFlashFlag)
      {
        if (g_bTimeCounterStop)
        {
          return false;  // dalsi zablesk nedorazil
        }
      }

      nFlashes++;
      if (nFlashes == g_nFlashes)
      {
        if (g_nTimeCounter < (g_nFlashInterval + g_nFlashInterval / 3) &&
            g_nTimeCounter > (g_nFlashInterval - g_nFlashInterval / 3))
        {
          return true;
        }
      }
    }

    return false;  // pocet
  }

  return true;

}

void Programming()
{
  uint8_t nFlashes = 0;
  g_nFlashLimit = FLASH_LIMIT_PRG;

  g_nFlashInterval = 0;
  Gpio_SetOffInterval(PRG_OFF_INTERVAL_MS);

  // cekani na prvni zablesk
  while (!g_bFlashFlag)
  {
    if (Gpio_IsButtonPressed_ms())
    {
      // testovaci zablesk a vypnout
      Gpio_Off();
    }

    if (!Gpio_GetOffTime())
    {
      g_eState = APP_STATE_START;
      return;
    }
  }

  nFlashes++;
  App_WaitAfterFlash();

  g_nTimeCounter = 0;
  g_bTimeCounterStop = false;

  // pocitani zablesku
  while (!g_bTimeCounterStop)
  {
    if (g_bFlashFlag)
    {
      g_bTimeCounterStop = true;
      g_nFlashInterval = g_nTimeCounter;
      nFlashes++;

      g_nTimeCounter = 0;
      g_bTimeCounterStop = false;

      App_WaitAfterFlash();
    }
  }

  // ulozit data
  Eeprom_UnlockPELOCK();
  Eeprom_WriteUint32(EEPROM_FLASH_INTERVAL, g_nFlashInterval);
  Eeprom_WriteUint32(EEPROM_FLASHES, nFlashes);
  Eeprom_LockNVM();

  g_eState = APP_STATE_START;
}

void App_WaitAfterFlash(void)
{
  // wait 5 ms pro odezneni zablesku
  g_nDelayTimer = 100;  // 100 * 50 us
  while (g_nDelayTimer);
  g_bFlashFlag = false;
}

/**
* @brief  This function handles the point-to-point packet transmission
* @param  AppliFrame_t *xTxFrame = Pointer to AppliFrame_t structure
*         uint8_t cTxlen = Length of aTransmitBuffer
* @retval None
*/
void App_SendBuffer(uint8_t* pBuffer, uint8_t nLength)
{
  memcpy(TxFrameBuff, pBuffer, nLength);

  // Spirit IRQs disable
//  Spirit_DisableIRQ();

  // Spirit IRQs enable
  Spirit1EnableTxIrq();

  // payload length config
  Spirit1SetPayloadlength(nLength);

  // rx timeout config
  Spirit1SetRxTimeout(RECEIVE_TIMEOUT);

  // IRQ registers blanking
  Spirit1ClearIRQ();

  // destination address
  Spirit1SetDestinationAddress(DESTINATION_ADDRESS);

  // send the TX command
  Spirit1StartTx(TxFrameBuff, nLength);
}


/**
* @brief  This function handles the point-to-point packet reception
* @param  uint8_t *RxFrameBuff = Pointer to ReceiveBuffer
*         uint8_t cRxlen = length of ReceiveBuffer
* @retval None
*/
void App_ReceiveBuffer(uint8_t *RxFrameBuff, uint8_t cRxlen)
{
  /* Spirit IRQs enable */
  Spirit1EnableRxIrq();

  /* payload length config */
  Spirit1SetPayloadlength(PAYLOAD_LEN);

  // rx timeout config
  Spirit1SetRxTimeout(RECEIVE_TIMEOUT);

  // destination address
  Spirit1SetDestinationAddress(DESTINATION_ADDRESS);

  // IRQ registers blanking
  Spirit1ClearIRQ();

  // RX command
  Spirit1StartRx();
}


//void App_FlashActive()
//{
//  g_eState = APP_STATE_SEND_FLASH;
//  Gpio_SetOffInterval(STD_OFF_INTERVAL_MS);
//}

void App_ADCGetConv(uint16_t ADCValue)
{
  // obsluha citacu 50us
  if (!g_bTimeCounterStop)
  {
    g_nTimeCounter++;
    if (g_nTimeCounter == 0)
    {
      g_bTimeCounterStop = true;
    }
  }

  if (g_nDelayTimer)
  {
    g_nDelayTimer--;
  }

  /* Zparacovat data z ADC */
  ADCValue = ADCValue >> 7;

  if (ADCValue == g_nOptoValue)
  {
    return;
  }
  else if (ADCValue > g_nOptoValue)
  {
    g_nOptoValue++;
    if (g_bFlashEnable)
    {
      if (ADCValue > (g_nOptoValue + g_nFlashLimit))
      {
        g_bFlashFlag = true;
        return;
      }
    }
  }
  else if (ADCValue < g_nOptoValue && g_nOptoValue)
  {
    g_nOptoValue--;
  }
}

void OnSpiritInterruptHandlerMaster(void)
{
  SpiritIrqGetStatus(&xIrqStatus);
  if(xIrqStatus.IRQ_TX_DATA_SENT || xIrqStatus.IRQ_MAX_RE_TX_REACH)
  {
    g_TxDoneFlag = SET;
  }
}

void OnSpiritInterruptHandlerSlave(void)
{
  SpiritIrqGetStatus(&xIrqStatus);

  /* Check the SPIRIT RX_DATA_READY IRQ flag */
  if((xIrqStatus.IRQ_RX_DATA_READY))
  {
    g_RxDoneFlag = SET;
  }

  /* Restart receive after receive timeout*/
  if (xIrqStatus.IRQ_RX_TIMEOUT)
  {
    g_RXtimeout = SET;
  }

  /* Check the SPIRIT RX_DATA_DISC IRQ flag */
  if(xIrqStatus.IRQ_RX_DATA_DISC)
  {
    /* RX command - to ensure the device will be ready for the next reception */
    g_RXdataDisc = SET;
  }
}

/**
* @brief  This function handles External interrupt request. In this application it is used
*         to manage the Spirit IRQ configured to be notified on the Spirit GPIO_3.
* @param  None
* @retval None
*/
void OnSpiritInterruptHandlerSlaveSniffer(void)
{
  SpiritIrqGetStatus(&xIrqStatus);

  if (xIrqStatus.IRQ_WKUP_TOUT_LDC)
  {
    SpiritCmdStrobeLdcReload();
  }

  if(xIrqStatus.IRQ_RSSI_ABOVE_TH)
  {
    SpiritIrqs IRQmask;
    SpiritIrqGetMask(&IRQmask);

    SpiritIrq(RSSI_ABOVE_TH, S_DISABLE);
    SpiritIrq(VALID_SYNC, S_ENABLE);
    /* disable LDC mode to avoid LDC timer falling during block Reception*/
    SpiritTimerLdcrMode(S_DISABLE);

    /* start MCU Timer to detect SYNC : SYNC_TIMEOUT_DURATION */
    Gpio_TimoutTimerState(true);
  }

  if(xIrqStatus.IRQ_VALID_PREAMBLE)
  {
    SpiritIrq(VALID_PREAMBLE, S_DISABLE);
    SpiritIrq(VALID_SYNC, S_ENABLE);
//    uint8_t pqi = SpiritQiGetPqi();

    /* disable LDC mode to avoid LDC timer falling during block Reception*/
    SpiritTimerLdcrMode(S_DISABLE);

    /* start MCU Timer to detect SYNC : SYNC_TIMEOUT_DURATION */
    Gpio_TimoutTimerState(true);
  }

  /* Check the SPIRIT IRQ_VALID_SYNC IRQ flag */
  if(xIrqStatus.IRQ_VALID_SYNC)
  {
    SpiritIrq(VALID_SYNC, S_DISABLE);
    SpiritIrq(RX_DATA_DISC,S_ENABLE);

    /* SYNC IRQ received => disable SYNC_TIMEOUT_DURATION timeout */
    Gpio_TimoutTimerState(false);
  }

  /* Check the SPIRIT RX_DATA_READY IRQ flag */
  if((xIrqStatus.IRQ_RX_DATA_READY))
  {
    g_RxDoneFlag = SET;
    uint8_t nLength;
    Spirit1GetRxPacket(RxBuffer, &nLength);

    if (memcmp(RxBuffer, aCheckBroadcast, sizeof (aCheckBroadcast)) == 0)
    {
      Gpio_LedBlink(50);
    }
    else if (memcmp(RxBuffer, aFlashBroadcast, sizeof (aFlashBroadcast)) == 0)
    {
      Gpio_FlashBlink();
      Gpio_SetOffInterval(STD_OFF_INTERVAL_RCV_MS);
    }

    App_PrepareSniffing();
  }

  /* Restart receive after receive timeout*/
  if (xIrqStatus.IRQ_RX_TIMEOUT)
  {
    g_RXtimeout = SET;
    SpiritTimerReloadStrobe();
    App_PrepareSniffing();
//    SpiritCmdStrobeRx();
  }
  /* Check the SPIRIT RX_DATA_DISC IRQ flag */
  if(xIrqStatus.IRQ_RX_DATA_DISC)
  {
    /* RX command - to ensure the device will be ready for the next reception */
    App_PrepareSniffing();
  }

}

void App_SniffConfigure(void)
{
  // configure sniff mode
  SpiritIrqDeInit(&xIrqStatus);

  // set RSSI treshold
  SpiritQiSetRssiThresholddBm(-120);
  SpiritQiSetCsMode(CS_MODE_STATIC_3DB);

//  SpiritTimerSetRxTimeoutStopCondition(RSSI_ABOVE_THRESHOLD);  // timeout zrusen pri detekci signalu

  SpiritQiSetPqiThreshold(PQI_TH_1);
  SpiritQiPqiCheck(S_ENABLE);
  SpiritTimerSetRxTimeoutStopCondition(PQI_ABOVE_THRESHOLD);  // timeout zrusen pri detekci signalu

  /* enable SQI check */
  SpiritQiSetSqiThreshold(SQI_TH_0);
  SpiritQiSqiCheck(S_ENABLE);

  /* RX timeout config => CS evaluation */
  SpiritTimerSetRxTimeoutMs(0.3);   // 300 us

  //#define SYNC_TIMEOUT_DURATION   ((1.0E3*(PREAMBLE_LENGTH_BITS + SYNC_LENGTH_BITS+10))/DATARATE)
  Gpio_TimoutTimerConfig_ms(2, App_OnTimeoutTimer); // 1000 * (40+32+10) / 38400 (datarate)

  // ((1.0E3*(PREAMBLE_LENGTH_BITS - 16))/DATARATE)
  SpiritTimerSetWakeUpTimerMs(10); // 1000*(40-16)/32400

  float fWUtimer;
  uint8_t nWUconter, nWUprescaler;
  SpiritTimerGetWakeUpTimer(&fWUtimer, &nWUconter, &nWUprescaler);

  SpiritTimerSetWakeUpTimerReload(nWUconter, nWUprescaler);

  SpiritIrq(RX_DATA_READY,S_ENABLE);
  SpiritIrq(RX_TIMEOUT, S_ENABLE);

  App_PrepareSniffing();

}

void App_PrepareSniffing()
{
  Gpio_TimoutTimerState(false);

  SpiritIrq(VALID_SYNC, S_DISABLE);
  SpiritIrq(RX_DATA_DISC, S_DISABLE);

//  SpiritIrq(RSSI_ABOVE_TH, S_ENABLE);
  SpiritIrq(VALID_PREAMBLE, S_ENABLE);

  SpiritIrqs IRQmask;
  SpiritIrqGetMask(&IRQmask);

  /* enable LDC mode and start Rx */
  SpiritTimerLdcrMode(S_ENABLE);
  SpiritIrqClearStatus();
  SpiritCmdStrobeFlushRxFifo();
  SpiritCmdStrobeRx();
}

void App_OnTimeoutTimer()
{
  App_PrepareSniffing();
}
