/*
 * main.c
 *
 *  Created on: 10. 8. 2016
 *      Author: priesolv
 */

#include "stm32l0xx.h"
#include "clock.h"
#include "spirit_spi.h"
#include "timer.h"
#include "trigger_app.h"
#include "Gpio_utility.h"

#include "Eeprom.h"


/* --------------------------------------------------------------------------------
 *
 *  Vzdalenost impulsu na fotoaparatu FinePix S6500 je cca 100ms.
 *  1. impuls (predblesk): sirka 500us
 *  2. impuls : sirka 700 us, pak klesa az do 1,5 ms, ve 2ms uz je uroven nula
 *
 *  blesk: plna uroven-sirka 2ms, pak klesa, ve 4ms je uroven nula
 * --------------------------------------------------------------------------------
 *
 * Spotreba:
 *   vypnuto:   MASTER i SLAVE 1,6 uA
 *   po zapnutí:
 *     MASTER         SLAVE
 *      4 mA            14,4 mA
 *
 * --------------------------------------------------------------------------------
 *
 * Intervaly tlacitka pri zapnuti pro MASTER (SLAVE umi jenom zapnout):
 *   < 1000ms: zapnuti, pocet bliknuti indikuje naprogramovany pocet zablesku
 *   > 1000ms: mod rucniho odpalovani slave blesku
 *   > 3000ms: programovani poctu zablesku fotoaparatu
 *
 *
 * v0.1 - prvni verze (1.3.2018)
 */


int main(void)
{
  // pri behu na MSI 2,1 MHz je spotreba 261 uA

  // pri behu na HSI 16MHz je spotreba cca MCU cca 1,5 mA
  SetHSI16();

  SystemCoreClockUpdate();

//  Eeprom_UnlockPELOCK();
//
//  *(uint8_t *)(DATA_E2_ADDR+1) = 0x55; /* (1) */
//  uint8_t v = (*(uint8_t *)(DATA_E2_ADDR+1));
//  Eeprom_LockNVM();

  App_Init();

//  Programming();



//  // po resetu jdeme do Standby
//  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
//  if (!(PWR->CSR & PWR_CSR_SBF))
//  {
//    Gpio_StandbyMode();
//  }

//  uint8_t res = SpiritRadioSetFrequencyBase(915000000);
//  uint32_t freq = SpiritRadioGetFrequencyBase();
  while (1)
  {
    App_Exec();
  }

}
