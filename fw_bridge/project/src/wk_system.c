/* add user code begin Header */
/**
  **************************************************************************
  * @file     wk_system.c
  * @brief    work bench config program
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
/* add user code end Header */

#include "wk_system.h"

/* global variable */
volatile uint32_t timebase_ticks;

/**
  * @brief  this function is called to increment a global variable "timebase_ticks"
  *         used as application time base.
  * @param  none
  * @retval none
  */
__WEAK void wk_timebase_increase(void)
{
  timebase_ticks ++;
}

/**
  * @brief  provides a tick value in millisecond.
  * @param  none
  * @retval tick value
  */
__WEAK uint32_t wk_timebase_get(void)
{
  return timebase_ticks;
}

/**
  * @brief  this function provides minimum delay (in milliseconds) based
  *         on variable incremented.
  * @param  delay variable specifies the delay time length, in milliseconds.
  * @retval none
  */
__WEAK void wk_delay_ms(uint32_t delay)
{
  uint32_t start_tick = wk_timebase_get();

  if(delay < 0xFFFFFFFFU)
  {
    delay += 1;
  }

  while((wk_timebase_get() - start_tick) < delay)
  {
  }
}

/**
  * @brief  this function configures the source of the time base
  *         the time source is configured to have 1ms time base
  * @param  none
  * @retval none
  */
__WEAK void wk_timebase_init(void)
{
  crm_clocks_freq_type crm_clocks;
  uint32_t frequency = 0;

  /* get crm_clocks */
  crm_clocks_freq_get(&crm_clocks);

  frequency = crm_clocks.ahb_freq / 8;

  /* config systick clock source */
  systick_clock_source_config(SYSTICK_CLOCK_SOURCE_AHBCLK_DIV8);
  /* system tick config */
  SysTick->LOAD  = (uint32_t)((frequency / 1000) - 1UL);
  SysTick->VAL   = 0UL;
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk |
                   SysTick_CTRL_ENABLE_Msk;
}

/**
  * @brief  this function is called at timebase handler, eg: SysTick_Handler
  * @param  none
  * @retval none
  */
__WEAK void wk_timebase_handler(void)
{
  wk_timebase_increase();
}

