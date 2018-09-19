/*
 * timer.c
 *
 *  Created on: 24. 6. 2016
 *      Author: Priesol Vladimir
 */

#include "timer.h"

#define TIMER_US      TIM21         // prirazeni casovace pro mereni us intervalu
#define TIMER_US_CLK  RCC_APB2ENR_TIM21EN

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
    RCC->APB2ENR |= TIMER_US_CLK;
}

void TimerUs_start(void)
{
  TIMER_US->PSC = SystemCoreClock / 1000000; // 7 instructions
  TIMER_US->CNT = 0;
  TIMER_US->EGR = TIM_EGR_UG;
  TIMER_US->CR1 |= TIM_CR1_CEN;
}

uint16_t TimerUs_get_microseconds(void)
{
    return TIMER_US->CNT;
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
  TIMER_US->CNT = 0;
}

void TimerUs_stop(void)
{
  TIMER_US->CR1 &= ~TIM_CR1_CEN;
}
