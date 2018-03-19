/*
 * spirit1_app.c
 *
 *  Created on: 21. 6. 2016
 *      Author: priesolv
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

/**
* @brief Radio structure fitting
*/
SRadioInit xRadioInit =
{
  XTAL_OFFSET_PPM,
  BASE_FREQUENCY,
  CHANNEL_SPACE,
  CHANNEL_NUMBER,
  MODULATION_SELECT,
  DATARATE,
  FREQ_DEVIATION,
  BANDWIDTH
};


/**
* @brief Packet Basic structure fitting
*/
PktBasicInit xBasicInit =
{
  PREAMBLE_LENGTH,
  SYNC_LENGTH,
  SYNC_WORD,
  LENGTH_TYPE,
  LENGTH_WIDTH,
  CRC_MODE,
  CONTROL_LENGTH,
  EN_ADDRESS,
  EN_FEC,
  EN_WHITENING
};

/**
* @brief Address structure fitting
*/
PktBasicAddressesInit xAddressInit =
{
  EN_FILT_MY_ADDRESS,
  MY_ADDRESS,
  EN_FILT_MULTICAST_ADDRESS,
  MULTICAST_ADDRESS,
  EN_FILT_BROADCAST_ADDRESS,
  BROADCAST_ADDRESS
};

#define CHECK_INTERVAL_MS         2500

#define FLASH_LIMIT_STD           2
#define FLASH_LIMIT_PRG           8

#define STD_OFF_INTERVAL_MS       (1000*60*20)  // 20 minut
#define PRG_OFF_INTERVAL_MS       (1000*60*1)   // 1 minuta

#define EEPROM_FLASH_INTERVAL     0                                             // interval mezi zablesky
#define EEPROM_FLASHES            (EEPROM_FLASH_INTERVAL + sizeof(uint32_t))    // pocet zablesku

uint8_t aCheckBroadcast[] = {'C','H','E','C','K'};
uint8_t aFlashBroadcast[] = {'F','L','A','S','H'};
uint8_t RxBuffer[MAX_BUFFER_LEN];

SpiritIrqs xIrqStatus;

volatile FlagStatus xRxDoneFlag = RESET;
volatile FlagStatus xTxDoneFlag = RESET;
volatile FlagStatus cmdFlag = RESET;
volatile FlagStatus xStartRx = RESET;
volatile FlagStatus rx_timeout = RESET;
volatile FlagStatus exitTime = RESET;

uint16_t exitCounter = 0;
uint8_t TxFrameBuff[MAX_BUFFER_LEN];

AppState_t g_eState = APP_STATE_IDLE;

uint8_t g_Master = 0;

uint16_t g_nOptoValue;

volatile bool g_bFlashFlag;          // detekovan zablesk
volatile bool g_bFlashEnable = false;  // povoleni detekce zablesku

volatile uint16_t g_nDelayTimer;    // odmerovani intervalu
volatile bool g_bDelayOver;

volatile uint16_t g_nTimeCounter;       // citac poctu preruseni ADC (50 us)
volatile bool g_bTimeCounterStop;       // citat start/stop, po preteceni se sam vypne

uint16_t g_nFlashLimit;               // citlivost detekce zablesku
uint16_t g_nFlashInterval;            // naprogramovana vzdalenost mezi zablesky
uint16_t g_nFlashes = 1;              // pocet naprogramovanych zablesku

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
    if((RESET != xRxDoneFlag)||(RESET != rx_timeout)||(SET != exitTime))
    {
      if((rx_timeout == SET)||(exitTime == RESET))
      {
        rx_timeout = RESET;
        g_eState = APP_STATE_START_RX;
      }
      else if(xRxDoneFlag)
      {
        xRxDoneFlag = RESET;
        g_eState = APP_STATE_DATA_RECEIVED;
      }
    }
    break;

  case APP_STATE_DATA_RECEIVED:
    {
      Spirit1GetRxPacket(RxBuffer, &nLength);
      /*rRSSIValue = Spirit1GetRssiTH();*/

      g_eState = APP_STATE_START_RX;
      if (memcmp(RxBuffer, aCheckBroadcast, sizeof (aCheckBroadcast)) == 0)
      {
        Gpio_LedBlink(50);
      }
      else if (memcmp(RxBuffer, aFlashBroadcast, sizeof (aFlashBroadcast)) == 0)
      {
        Gpio_FlashBlink();
        Gpio_LedBlink(50);
        SetOffInterval(STD_OFF_INTERVAL_MS);
      }
    }
    break;

  case APP_STATE_SEND_CHECK:
    {
      Gpio_LedBlink(100);
      App_SendBuffer(aCheckBroadcast, sizeof (aCheckBroadcast));
      g_eState = APP_STATE_WAIT_FOR_TX_DONE;
    }
    break;

  case APP_STATE_SEND_FLASH:
      App_SendBuffer(aFlashBroadcast, sizeof (aFlashBroadcast));
      g_eState = APP_STATE_WAIT_FOR_TX_DONE;
      Gpio_LedBlink(100);
      App_WaitAfterFlash();
    break;

  case APP_STATE_WAIT_FOR_TX_DONE:
    if(xTxDoneFlag)
    {
      xTxDoneFlag = RESET;
      g_eState = APP_STATE_IDLE;
    }
    break;

  case APP_STATE_IDLE:
    if (g_Master)
    {
      if (nLastCheckTime + CHECK_INTERVAL_MS < GetTicks_ms())
      {
        g_eState = APP_STATE_SEND_CHECK;
        nLastCheckTime = GetTicks_ms();
      }
    }

    break;
  }

  // kontrola casovace vypnuti (do Standby modu)
  if (GetOffTime() == 0)
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

  // vyhodnoceni zablesku
  if (g_bFlashFlag)
  {
    if (App_CheckFlash())
    {
      // naprogramovana sekvence souhlasi
      g_eState = APP_STATE_SEND_FLASH;
      SetOffInterval(STD_OFF_INTERVAL_MS);
    }
    else
    {
      g_eState = APP_STATE_IDLE;
    }
  }
}

void App_Init()
{
  TimerInit();

  Gpio_Init();

  SetOffInterval(STD_OFF_INTERVAL_MS);

  // zjistime, jestli jsme MASTER nebo SLAVE
  g_Master = Gpio_IsMaster();

  // povesime funkci na preruseni podle toho jestli budeme vysilat (MASTER) nebo prijimat (SLAVE)
  if (g_Master)
  {
    Spirit_Init(OnSpiritInterruptHandlerMaster);
  }
  else
  {
    // Spirit_Init(OnSpiritInterruptHandlerSlaveSniffer);
    Spirit_Init(OnSpiritInterruptHandlerSlave);
  }

  // cekat na uvolneni tlacitka a pro MASTER zmerime delku stisknuti
  uint32_t nStartTime = GetTicks_ms();
  while (Gpio_IsButtonPressed_ms());
  uint32_t nPressDuration = GetTicks_ms() - nStartTime;

  g_eMode = APP_MODE_OPTO;
  g_eState = APP_STATE_IDLE;
  if (g_Master)
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
  uint32_t nTime = GetTicks_ms();
  while (GetTicks_ms() - nTime < 2);

  SpiritManagementWaExtraCurrent();

  uint8_t v;
  StatusBytes sb;
  sb = SpiritSpiReadRegisters(DEVICE_INFO1_PARTNUM, 1, &v);
  sb = SpiritSpiReadRegisters(DEVICE_INFO0_VERSION, 1, &v);

  // wait for READY state and set XTAL frequency
  SpiritManagementIdentificationRFBoard();

  // Spirit IRQ config
  Spirit1GpioIrqInit(&xGpioIRQ);

  // Spirit Radio config
  Spirit_InitRegs(g_Master);

  Spirit_SetPowerRegs();  // Spirit Radio set power
  // Spirit_ProtocolInitRegs();  // Spirit Packet config

  App_SpiritBasicProtocolInit();

  // Todo: musi to tu byt pro master???
  Spirit_EnableSQIRegs();
  Spirit_SetRssiTHRegs();

//  SRadioInit RadioInitStruct;
//  SpiritRadioGetInfo(&RadioInitStruct);
  /*
   * RadioInitStruct  SRadioInit  {...}
    nXtalOffsetPpm     int16_t           0
    lFrequencyBase     uint32_t          914999962
    nChannelSpace      uint32_t          21350
    cChannelNumber     uint8_t           0 '\0'
    xModulationSelect  ModulationSelect  FSK
    lDatarate          uint32_t          115203
    lFreqDev           uint32_t          61035
    lBandwidth         uint32_t          216057
   */

  // start blik
  Gpio_LedBlink(200);

  // pro MASTER nakonfigurovat optodiodu
  if (g_Master)
  {
    Gpio_OptoInit(App_ADCGetConv);
  }
  else
  {
//    App_SniffConfigure();
//    while(1);
  }

  // pro MASTER konfigurace opto snimani
  if (g_Master)
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
        Delay_ms(500);
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
  SetOffInterval(PRG_OFF_INTERVAL_MS);

  // cekani na prvni zablesk
  while (!g_bFlashFlag)
  {
    if (Gpio_IsButtonPressed_ms())
    {
      // testovaci zablesk a vypnout
      Gpio_Off();
    }

    if (!GetOffTime())
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
  /*float rRSSIValue = 0;*/
  exitTime = SET;
  exitCounter = TIME_TO_EXIT_RX;

//  PktBasicAddressesInit PktBasicAddresses;
//  SpiritPktBasicGetAddressesInfo(&PktBasicAddresses);

  /* Spirit IRQs disable */
//  Spirit1DisableIrq();

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

/** @brief  This function initializes the BASIC Packet handler of spirit1
* @param  None
* @retval None
*/

/*
 *   |   1-32   | 1-4  | 0-16 bit |   0-1   |   0-4   | 0-65535 | 0-3 |
 *   | Preamble | Sync |  Length  | Address | Control | Payload | CRC |
 *
 *  Preamble (programmable field): the length of the preamble is programmable from 1 to 32
    bytes by the PREAMBLE_LENGTH field of the PCKTCTRL2 register. Each preamble byte is
    a '10101010' binary sequence.

    Sync (programmable field): the length of the synchronization field is programmable (from 1
    to 4 bytes) through dedicated registers. The SYNC word is programmable through registers
    SYNC1, SYNC2, SYNC3, and SYNC4. If the programmed sync length is 1, then only SYNC
    word is transmitted; if the programmed sync length is 2 then only SYNC1 and SYNC2 words
    are transmitted and so on.

    Length (programmable/optional field): the packet length field is an optional field that is
    defined as the cumulative length of Address, Control, and Payload fields. It is possible to
    support fixed and variable packet length. In fixed mode, the field length is not used.

    Destination address (programmable/optional field): when the destination address filtering
    is enabled in the receiver, the packet handler engine compares the destination address field
    of the packet received with the value of register TX_SOURCE_ADDR. If broadcast address
    and/or multicast address filtering are enabled, the packet handler engine compares the
    destination address with the programmed broadcast and/or multicast address.

    Control (programmable/optional field): is programmable from 0 to 4 bytes through the
    CONTROL_LEN field of the PCKTCTRL4 register. Control fields of the packet can be set
    using the TX_CTRL_FIELD[3:0] register.

    Payload (programmable/optional field): the device supports both fixed and variable payload
    length transmission from 0 to 65535 bytes.
 */

void App_SpiritBasicProtocolInit(void)
{
  SpiritPktBasicSetFormat();

  PktBasicInit xBasicInit=
  {
    .xPreambleLength = PREAMBLE_LENGTH,
    .xSyncLength = SYNC_LENGTH,
    .lSyncWords = SYNC_WORD,
    .xFixVarLength = LENGTH_TYPE,
    .cPktLengthWidth = LENGTH_WIDTH,
    .xCrcMode = CRC_MODE,
    .xControlLength = CONTROL_LENGTH,
    .xAddressField = S_ENABLE,
    .xFec = EN_FEC,
    .xDataWhitening = EN_WHITENING
  };

  // Spirit Packet config
  SpiritPktBasicInit(&xBasicInit);

  PktBasicAddressesInit xAddressInit=
  {
    .xFilterOnMyAddress = S_DISABLE,
    .cMyAddress = MY_ADDRESS,
    .xFilterOnMulticastAddress = S_DISABLE,
    .cMulticastAddress = MULTICAST_ADDRESS,
    .xFilterOnBroadcastAddress = S_DISABLE,
    .cBroadcastAddress = BROADCAST_ADDRESS
  };

  SpiritPktBasicAddressesInit(&xAddressInit);
}

void App_FlashActive()
{
  g_eState = APP_STATE_SEND_FLASH;
  SetOffInterval(STD_OFF_INTERVAL_MS);
}

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
    xTxDoneFlag = SET;
  }
}

void OnSpiritInterruptHandlerSlave(void)
{
  SpiritIrqGetStatus(&xIrqStatus);

  /* Check the SPIRIT RX_DATA_READY IRQ flag */
  if((xIrqStatus.IRQ_RX_DATA_READY))
  {
    xRxDoneFlag = SET;
    uint8_t nLength;
    Spirit1GetRxPacket(RxBuffer, &nLength);

    if (memcmp(RxBuffer, aCheckBroadcast, sizeof (aCheckBroadcast)) == 0)
    {
      Gpio_LedBlink(50);
    }
    else if (memcmp(RxBuffer, aFlashBroadcast, sizeof (aFlashBroadcast)) == 0)
    {
      Gpio_FlashBlink();
      SetOffInterval(STD_OFF_INTERVAL_MS);
    }
  }

  /* Restart receive after receive timeout*/
  if (xIrqStatus.IRQ_RX_TIMEOUT)
  {
    rx_timeout = SET;
    SpiritCmdStrobeRx();
  }

  /* Check the SPIRIT RX_DATA_DISC IRQ flag */
  if(xIrqStatus.IRQ_RX_DATA_DISC)
  {
    /* RX command - to ensure the device will be ready for the next reception */
    SpiritCmdStrobeRx();
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
    uint8_t pqi = SpiritQiGetPqi();

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
    xRxDoneFlag = SET;
    uint8_t nLength;
    Spirit1GetRxPacket(RxBuffer, &nLength);

    if (memcmp(RxBuffer, aCheckBroadcast, sizeof (aCheckBroadcast)) == 0)
    {
      Gpio_LedBlink(50);
    }
    else if (memcmp(RxBuffer, aFlashBroadcast, sizeof (aFlashBroadcast)) == 0)
    {
      Gpio_FlashBlink();
      SetOffInterval(STD_OFF_INTERVAL_MS);
    }

    App_PrepareSniffing();
  }

  /* Restart receive after receive timeout*/
  if (xIrqStatus.IRQ_RX_TIMEOUT)
  {
    rx_timeout = SET;
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
