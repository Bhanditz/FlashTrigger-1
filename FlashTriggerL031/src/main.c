/*
 * main.c
 *
 *  Created on: 10. 8. 2016
 *      Author: Priesol Vladimir
 */

#include "stm32l0xx.h"
#include "clock.h"
#include "trigger_app.h"

/* ------------------------------------------------------------------------------
 * HW usage:
 *
 * ADC   - mereni optodiody na MASTER konfiguraci
 * TIM22 - spousteni ADC prevodu v intervalu 50 us
 * TIM21 - mereni microsec intervalu pro komunikaci SPIRIT1
 * TIM2  - timer pro sniff mod (nepouzito)
 */

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
 *      4 mA            15 mA
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
 * v0.2 - opravena kalibrace (v DEBUG konfiguraci stiha expozicni cas 1/60s)
 * v0.3 - problemy pri zmene teploty -> odstranena manualni kalibrace
 * v0.4(ve vyvoji) - zmena modulace na GFSK_BT1
 *                 - uprava parametru radia (datarate apod.)
 *                 - 3x opakovane vyslani FLASH prikazu
 *                 - wait interval pro komunikaci se SPIRIT1 meren casovacem
 *                 - zkracen timeout vypnuti bez prijmu signalu pro SLAVE
 *
 * Todo: blikani slave podle sily signalu
 *
 */


int main(void)
{
  // pri behu na MSI 2,1 MHz je spotreba 261 uA
  // pri behu na HSI 16MHz je spotreba cca MCU cca 1,5 mA
  SetHSI16();
//  SetMSI(msi_4Mhz);

  SystemCoreClockUpdate();

  App_Init();

//  Programming();


//  // po resetu jdeme do Standby
//  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
//  if (!(PWR->CSR & PWR_CSR_SBF))
//  {
//    Gpio_StandbyMode();
//  }


  while (1)
  {
    App_Exec();
  }

}
