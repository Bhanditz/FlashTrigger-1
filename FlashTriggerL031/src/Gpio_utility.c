/*
 * Gpio_utility.c
 *
 *  Created on: 24. 8. 2016
 *      Author: priesolv
 */

#include "Gpio_utility.h"
#include "timer.h"
#include "spirit.h"

#define LED_PIN                         (1 << 4)  // out
#define LED_GPIO_PORT                   GPIOA

#define BUTTON_PIN                      (1 << 0)  // in
#define BUTTON_GPIO_PORT                GPIOA

#define SLAVE_PIN                       (1 << 9)  // in
#define SLAVE_GPIO_PORT                 GPIOA

#define OPTO_OUTPUT_PIN                 (1 << 3)  // out
#define OPTO_OUTPUT_GPIO_PORT           GPIOA

#define OPTO_INPUT_PIN                  (1 << 1)  // in
#define OPTO_INPUT_GPIO_PORT            GPIOA

#define FLASH_OUTPUT_PIN                (1 << 14) // out
#define FLASH_OUTPUT_GPIO_PORT          GPIOC

#define LED_OFF                         (LED_GPIO_PORT->BRR = LED_PIN)
#define LED_ON                          (LED_GPIO_PORT->BSRR = LED_PIN)

#define FLASH_OFF                       (FLASH_OUTPUT_GPIO_PORT->BRR = FLASH_OUTPUT_PIN)
#define FLASH_ON                        (FLASH_OUTPUT_GPIO_PORT->BSRR = FLASH_OUTPUT_PIN)

#define OPTO_OUTPUT_OFF                 (OPTO_OUTPUT_GPIO_PORT->BRR = OPTO_OUTPUT_PIN)
#define OPTO_OUTPUT_ON                  (OPTO_OUTPUT_GPIO_PORT->BSRR = OPTO_OUTPUT_PIN)

#define ADC_OPTO_INPUT                  ADC_CHSELR_CHSEL1   // vstup ADC

#define FLASH_DURATION_MS               6           // delka impulsu na tyristoru spinani blesku

Ptr_OnAdcConv g_pOnAdcConv = NULL;      // callback od INT prevodniku
Ptr_OnTimer g_pOnTimer = NULL;          // callback od ??? (mod snifferu?)

uint16_t g_nLedInterval;                // citac delky svitu LED
uint16_t g_nFlashInterval_ms;           // citac delky impulsu na tyristoru
uint32_t g_nOffInterval;                // citani intervalu do vypnuti

volatile bool g_bButtonPressed = true;
volatile uint16_t g_nButtonStateDuration;

// inicializace GPIO
void Gpio_Init(void)
{
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOCEN;

  // Configure LED, pin as output
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE4)) | (GPIO_MODER_MODE4_0);

  // Configure BUTTON, SLAVE pins as input
  GPIOA->MODER = (GPIOA->MODER
                 & ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE9));
  GPIOA->PUPDR = (GPIOB->PUPDR  // pull up
                 & ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD9))\
                 | (GPIO_PUPDR_PUPD0_0 | GPIO_PUPDR_PUPD9_0);

  // FLASH OUTPUT pin as output
  GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODE14)) | (GPIO_MODER_MODE14_0);

  Timer_SetSysTickCallback(Gpio_SysTickCallback);
}

// inicializace ADC pro snimani optodiody
void Gpio_OptoInit(Ptr_OnAdcConv pOnAdcConv)
{
  g_pOnAdcConv = pOnAdcConv;

  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

  // Configure OPTO OUTPUT pin as output
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE3)) | (GPIO_MODER_MODE3_0);

  // Configure OPTO INPUT pins as analog input
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE1)) | GPIO_MODER_MODE1;

  // Configure ADC
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;  // clock for ADC

  /* (1) Select HSI16 by writing 00 in CKMODE (reset value) */
  /* (2) Select the external trigger on falling edge and external trigger on TIM22_TRGO
         by selecting TRG4 (EXTSEL = 100)*/
  /* (4) Select a sampling mode of 111 i.e. 239.5 ADC clk to be greater than 5us */
  /* (5) Wake-up the VREFINT (only for VLCD, Temp sensor and VRefInt) */
  ADC1->CFGR2 = (ADC1->CFGR2 & ~ADC_CFGR2_CKMODE) | ADC_CFGR2_CKMODE_0; /* (1) PCLK/2 */
  ADC1->CFGR1 |= ADC_CFGR1_EXTEN_0 | ADC_CFGR1_EXTSEL_2 ; /* (2) */
  ADC1->CHSELR = ADC_OPTO_INPUT; // channel
  ADC1->SMPR = /*ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | */ADC_SMPR_SMP_2; /* (4) */
  ADC1->IER = ADC_IER_EOCIE; // interrupt enable 'end of conversion'  (ADC_IER_EOSEQIE | ADC_IER_OVRIE)

  // Calibrate ADC
  if ((ADC1->CR & ADC_CR_ADEN) != 0) // Ensure that ADEN = 0
  {
    ADC1->CR &= (uint32_t)(~ADC_CR_ADEN);  // Clear ADEN
  }

  ADC1->CR |= ADC_CR_ADCAL; // Set ADCAL=1
  while ((ADC1->ISR & ADC_ISR_EOCAL) == 0) // Wait until EOCAL=1
  {
    /* For robust implementation, add here time-out management */
  }

  ADC1->ISR |= ADC_ISR_EOCAL; // Clear EOCAL

  ADC1->CR |= ADC_CR_ADEN;         // Enable the ADC
  if ((ADC1->CFGR1 &  ADC_CFGR1_AUTOFF) == 0)
  {
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) // Wait until ADC ready if AUTOFF is not set
    {
      /* For robust implementation, add here time-out management */
    }
  }

  NVIC_SetPriority(ADC1_COMP_IRQn, 1);
  NVIC_EnableIRQ(ADC1_COMP_IRQn);

  // Configure TIM 22 for ADC triggering (TIM22_TRGO)
  RCC->APB2ENR |= RCC_APB2ENR_TIM22EN;
  TIM22->CR2 |= TIM_CR2_MMS_1; // Configure MMS=010 to output a rising edge at each update event
  TIM22->PSC = 0;
  TIM22->ARR = (uint16_t) (SystemCoreClock / 20000); // register value = PCLK/20kHz(50us)
  TIM22->CR1 |= TIM_CR1_CEN; // Enable TIM22

  OPTO_OUTPUT_ON;

  ADC1->CR |= ADC_CR_ADSTART; /* Start the ADC conversion */

  // pockat na ustaleni svetelne hodnoty
  Timer_Delay_ms(20);
}

void Gpio_DisableADC()
{
  if ((ADC1->CR & ADC_CR_ADSTART) != 0) // Ensure that no conversion on going
  {
    ADC1->CR |= ADC_CR_ADSTP; // Stop any ongoing conversion
  }

  while ((ADC1->CR & ADC_CR_ADSTP) != 0) // Wait until ADSTP is reset by hardware i.e. conversion is stopped
  {
     /* For robust implementation, add here time-out management */
  }

  ADC1->CR |= ADC_CR_ADDIS; // Disable the ADC
  while ((ADC1->CR & ADC_CR_ADEN) != 0) // Wait until the ADC is fully disabled
  {
    /* For robust implementation, add here time-out management */
  }

  TIM22->CR1 &= ~TIM_CR1_CEN;   // stop TIM22
}

void Gpio_TimoutTimerConfig_ms(uint32_t nTime_ms, Ptr_OnTimer pOnTimer)
{
  g_pOnTimer = pOnTimer;

  // Configure TIM22 for sync timeout by receiving
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  TIM2->PSC = 1000;             // 16Mhz/1000 = 16kHz
  TIM2->ARR = (uint16_t)(16 * nTime_ms); // 16kHz / 1000 = 16ticks/ms

  NVIC_SetPriority(TIM2_IRQn, 2);
  NVIC_EnableIRQ(TIM2_IRQn);

  TIM2->DIER |= TIM_DIER_UIE;   // update interrupt enable
}

void Gpio_TimoutTimerState(uint8_t nEnable)
{
  if (nEnable)
  {
    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN; // Enable TIM2
  }
  else
  {
    TIM2->CR1 &= ~TIM_CR1_CEN;
  }
}

bool Gpio_IsMaster()
{
  return (SLAVE_GPIO_PORT->IDR & SLAVE_PIN) ? true : false;
}

void Gpio_LedBlink(uint16_t nDuration_ms)
{
  g_nLedInterval = nDuration_ms;
  LED_ON;
}

void Gpio_FlashBlink()
{
  g_nFlashInterval_ms = FLASH_DURATION_MS;
  FLASH_ON;
}

// pomale zhasinani LED (dynamicka zmena PWN LED)
void Gpio_LedOffDiming()
{
  uint16_t led_range = 900;  // delka zhasinani
  uint16_t led_rate = led_range;

  while (led_rate)
  {
    for (uint16_t i = 0; i < led_rate; i++)
    {
      LED_ON;
    }

    for (uint16_t i = led_range; i > led_rate; i--)
    {
      LED_OFF;
    }

    led_rate--;
  }
}

// pokud je tlacitko stisknuto, vraci dobu trvani stisknuti v ms, jinak 0
uint16_t Gpio_IsButtonPressed_ms()
{
  if (g_bButtonPressed)
  {
    return 1;
  }

  return 0;
}

// vypnuti
void Gpio_Off(void)
{
  Gpio_LedOffDiming();
  Gpio_DisableADC();
  Gpio_StandbyMode();
}

// prechod Standby modu
void Gpio_StandbyMode(void)
{
  Spirit_EnterShutdown();

  // to standby
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;  // PWR enable
  PWR->CSR |= PWR_CSR_EWUP1;          // Enable WKUP pin 1

  PWR->CR |= PWR_CR_CWUF;  // Clear Wakeup flag
  PWR->CR |= PWR_CR_CSBF;  // clear Standby flag
  PWR->CR |= PWR_CR_PDDS;  // Select STANDBY mode
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; // Set SLEEPDEEP bit of Cortex-M0 System Control Register

//  __WFI;  // Request Wait For Interrupt
  __ASM volatile ("wfi");

  for(;;);
}

void Gpio_SysTickCallback()
{
  // odmereni delky sviceni LED
  if (g_nLedInterval)
  {
    g_nLedInterval--;
    if (g_nLedInterval == 0)
    {
      LED_OFF;
    }
  }

  // odmereni delky impulsu na tyristoru
  if (g_nFlashInterval_ms)
  {
    g_nFlashInterval_ms--;
    if (g_nFlashInterval_ms == 0)
    {
      FLASH_OFF;
    }
  }

  // odmereni OFF intervalu
  if (g_nOffInterval)
  {
    g_nOffInterval--;
  }

  // obsluha tlacitka (odstraneni zakmitu)
#define DBNC_CNT  5   // e.g. 20ms with 5ms tick
  if ((Timer_GetTicks_ms() & 0b11) == 0) // kazde 4 ms
  {

    static uint8_t old_state = 0;
    static uint8_t key_cnt = 0;

//    uint8_t state = (!(BUTTON_GPIO_PORT->IDR & BUTTON_PIN));
    uint8_t state = (BUTTON_GPIO_PORT->IDR & BUTTON_PIN);
    if (state != old_state)
    {
      key_cnt = DBNC_CNT;
      g_nButtonStateDuration = 0;
    }
    else if (key_cnt == 0)
    {
      g_bButtonPressed = state;
      g_nButtonStateDuration += 4;
    }
    else
      key_cnt--;

    old_state = state;
  }
}

void Gpio_SetOffInterval(uint32_t nInterval_ms)
{
  g_nOffInterval = nInterval_ms;
}

uint32_t Gpio_GetOffTime(void)
{
  return g_nOffInterval;
}

void ADC1_COMP_IRQHandler(void)
{
  if (ADC1->ISR & ADC_ISR_EOC)
  {
    if (g_pOnAdcConv)
    {
      g_pOnAdcConv(ADC1->DR);
    }
  }
}
