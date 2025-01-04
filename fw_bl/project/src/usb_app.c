/* add user code begin Header */
/**
  **************************************************************************
  * @file     usbd_app.c
  * @brief    usb device app
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

#include "usb_conf.h"
#include "usb_core.h"
#include "wk_system.h"

#include "usbd_int.h"
#include "cdc_class.h"
#include "cdc_desc.h"

#include "at32f402_405_wk_config.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */

/* add user code end private includes */

/* private typedef -----------------------------------------------------------*/
/* add user code begin private typedef */

/* add user code end private typedef */

/* private define ------------------------------------------------------------*/
/* add user code begin private define */

/* add user code end private define */

/* private macro -------------------------------------------------------------*/
/* add user code begin private macro */

/* add user code end private macro */

/* private variables ---------------------------------------------------------*/
/* add user code begin private variables */

/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */

/* add user code end function prototypes */

static otg_core_type otg_core_struct_hs;

uint32_t usbd_app_buffer_hs[128];

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/* add user code end 0 */

/**
  * @brief  usb application initialization
  * @param  none
  * @retval none
  */
void wk_usb_app_init(void)
{
  /* add user code begin usb_app_init 0 */

  /* add user code end usb_app_init 0 */

  /*hs device cdc*/
  usbd_init(&otg_core_struct_hs,
            USB_HIGH_SPEED_CORE_ID,
            USB_OTG2_ID,
            &cdc_class_handler,
            &cdc_desc_handler);

  /* add user code begin usb_app_init 1 */

  /* add user code end usb_app_init 1 */
}

/**
  * @brief  usb application task
  * @param  none
  * @retval none
  */
void wk_usb_app_task(void)
{
  /* add user code begin usb_app_task 0 */

  /* add user code end usb_app_task 0 */

  uint32_t length = 0;

  uint32_t timeout = 0;
  static uint8_t send_zero_packet_hs = 0;

  /* add user code begin usb_app_task 1 */

  /* add user code end usb_app_task 1 */

  /* hs device cdc */
  length = usb_vcp_get_rxdata(&otg_core_struct_hs.dev, (uint8_t *)usbd_app_buffer_hs);
  if(length > 0 || send_zero_packet_hs == 1)
  {
    if(length > 0)
      send_zero_packet_hs = 1;
    if(length == 0)
      send_zero_packet_hs = 0;
    timeout = 5000000;
    do
    {
      if(usb_vcp_send_data(&otg_core_struct_hs.dev, (uint8_t *)usbd_app_buffer_hs, length) == SUCCESS)
      {
        break;
      }
    }while(timeout --);
  }

  /* add user code begin usb_app_task 2 */

  /* add user code end usb_app_task 2 */
}

/**
  * @brief  usb interrupt handler
  * @param  none
  * @retval none
  */
void wk_otghs_irq_handler(void)
{
  /* add user code begin otghs_irq_handler 0 */

  /* add user code end otghs_irq_handler 0 */

  usbd_irq_handler(&otg_core_struct_hs);

  /* add user code begin otghs_irq_handler 1 */

  /* add user code end otghs_irq_handler 1 */
}

/**
  * @brief  usb delay function
  * @param  ms: delay number of milliseconds.
  * @retval none
  */
void usb_delay_ms(uint32_t ms)
{
  /* add user code begin delay_ms 0 */

  /* add user code end delay_ms 0 */

  wk_delay_ms(ms);

  /* add user code begin delay_ms 1 */

  /* add user code end delay_ms 1*/
}

/* add user code begin 1 */

/* add user code end 1 */
