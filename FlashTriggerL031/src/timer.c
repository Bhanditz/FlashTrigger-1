/*
 * timer.c
 *
 *  Created on: 24. 6. 2016
 *      Author: priesolv
 */

#include "timer.h"

typedef void(*Ptr_OnTxDataPacketResponse)(void);

static volatile uint32_t nDelayTimer;
static volatile uint32_t g_nTicks = 0;

PtrSysTickCallback pSysTickCallback = 0;

void Timer_Init()
{
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    /* Capture error */
    while (1);
  }

  TimerUs_init();
}

void Timer_Delay_ms(uint32_t delay_ms)
{
  nDelayTimer = delay_ms;
  while (nDelayTimer);
}

uint32_t Timer_GetTicks_ms()
{
  return g_nTicks;
}

void Timer_SetSysTickCallback(PtrSysTickCallback pFunction)
{
  pSysTickCallback = pFunction;
}

void SysTick_Handler(void)
{
  g_nTicks++;
  if (nDelayTimer)
  {
    nDelayTimer--;
  }

  if (pSysTickCallback)
  {
    pSysTickCallback();
  }
}


// timer for us counting
void TimerUs_init(void)
{
    // Enable clock for TIM22
    RCC->APB2ENR |= RCC_APB2ENR_TIM22EN;
}

void TimerUs_start(void)
{
    TIM22->PSC = SystemCoreClock / 1000000; // 7 instructions
    TIM22->CNT = 0;
    TIM22->EGR = TIM_EGR_UG;
    TIM22->CR1 |= TIM_CR1_CEN;
}

uint16_t TimerUs_get_microseconds(void)
{
    return TIM22->CNT;
}

void TimerUs_delay(uint16_t microseconds)
{
    uint16_t t = TimerUs_get_microseconds() + microseconds;
    while (TimerUs_get_microseconds() < t)
    {
        continue;
    }
}

void TimerUs_clear(void)
{
    TIM22->CNT = 0;
}

void TimerUs_stop(void)
{
    TIM22->CR1 &= ~TIM_CR1_CEN;
}
