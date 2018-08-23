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


//#define USE_SPIRIT1_868MHz
#define USE_SPIRIT1_915MHz


/*  Radio configuration parameters  */
#define XTAL_OFFSET_PPM             0
#define INFINITE_TIMEOUT            0.0

#ifdef USE_SPIRIT1_868MHz
  #define BASE_FREQUENCY              868.0e6
#endif

#ifdef USE_SPIRIT1_915MHz
  #define BASE_FREQUENCY              915.0e6
#endif

#define CHANNEL_SPACE               20e3
#define CHANNEL_NUMBER              0
#define MODULATION_SELECT           GFSK_BT1
#define DATARATE                    19200
#define FREQ_DEVIATION              20e3
#define BANDWIDTH                   100E3

#define POWER_DBM                   11.6 /* max is 11.6 */
#define POWER_INDEX                 7

#define RECEIVE_TIMEOUT             2200.0 // master vysila CHECK po 2000ms
#define RSSI_THRESHOLD              -120

/*  Packet configuration parameters  */
#define PREAMBLE_LENGTH             PKT_PREAMBLE_LENGTH_04BYTES
#define SYNC_LENGTH                 PKT_SYNC_LENGTH_4BYTES
#define SYNC_WORD                   0x88888888
#define LENGTH_TYPE                 PKT_LENGTH_VAR
#define LENGTH_WIDTH                7
#define CRC_MODE                    PKT_CRC_MODE_8BITS
#define CONTROL_LENGTH              PKT_CONTROL_LENGTH_0BYTES
#define EN_FEC                      S_DISABLE
#define EN_WHITENING                S_ENABLE

/*  Addresses configuration parameters  */
#define EN_ADDRESS                  S_DISABLE
#define EN_FILT_MY_ADDRESS          S_DISABLE
#define EN_FILT_MULTICAST_ADDRESS   S_DISABLE
#define EN_FILT_BROADCAST_ADDRESS   S_DISABLE
#define EN_FILT_SOURCE_ADDRESS      S_DISABLE//S_ENABLE
#define MY_ADDRESS                  0x44
#define DESTINATION_ADDRESS         0x44
#define SOURCE_ADDR_MASK            0xf0
#define SOURCE_ADDR_REF             0x37
#define MULTICAST_ADDRESS           0xEE
#define BROADCAST_ADDRESS           0xFF


#define EN_AUTOACK                    S_DISABLE
#define EN_PIGGYBACKING               S_DISABLE
#define MAX_RETRANSMISSIONS           PKT_DISABLE_RETX


#define PAYLOAD_LEN                     10 //25 /*20 bytes data+tag+cmd_type+cmd+cmdlen+datalen*/
#define APPLI_CMD                       0x11
#define NWK_CMD                         0x22
#define LED_TOGGLE                      0xff
#define ACK_OK                          0x01
#define MAX_BUFFER_LEN                  96
#define TIME_TO_EXIT_RX                 3000
#define DELAY_RX_LED_TOGGLE             100
#define DELAY_TX_LED_GLOW               200
#define LPM_WAKEUP_TIME                 100
#define DATA_SEND_TIME                  50//30


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
void Spirit_BasicProtocolInit(void);
void Spirit_Calibrate(bool bMaster);
void Spirit_SetPowerRegs(void);
void Spirit_ProtocolInitRegs(void);
void Spirit_EnableSQI(void);
void Spirit_SetRssiThreshold(void);

#endif /* SPIRIT_H_ */
