/**
  **************************************************************************
  * @file     at32f402_405_pwc.h
  * @brief    at32f402_405 pwr header file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AT32F402_405_PWC_H
#define __AT32F402_405_PWC_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "at32f402_405.h"

/** @addtogroup AT32F402_405_periph_driver
  * @{
  */

/** @addtogroup PWC
  * @{
  */

/** @defgroup PWC_flags_definition
  * @brief pwc flag
  * @{
  */

#define PWC_WAKEUP_FLAG                  ((uint32_t)0x00000001) /*!< wakeup flag */
#define PWC_STANDBY_FLAG                 ((uint32_t)0x00000002) /*!< standby flag */
#define PWC_PVM_OUTPUT_FLAG              ((uint32_t)0x00000004) /*!< pvm output flag */

/**
  * @}
  */

/**
  * @brief pwc wakeup pin num definition
  */
#define PWC_WAKEUP_PIN_1                 ((uint32_t)0x00000100) /*!< standby wake-up pin1(pa0) */
#define PWC_WAKEUP_PIN_2                 ((uint32_t)0x00000200) /*!< standby wake-up pin2(pc13) */
#define PWC_WAKEUP_PIN_6                 ((uint32_t)0x00002000) /*!< standby wake-up pin6(pb5) */

/**
  * @brief  select ldo output voltage.
  * @param  val: set the ldo output voltage.
  *         this parameter can be one of the following values:
  *         - PWC_LDO_OUTPUT_1V1: system clock up to 108MHz.
  *         - PWC_LDO_OUTPUT_1V2: system clock up to 192MHz.
  *         - PWC_LDO_OUTPUT_1V3: system clock up to 216MHz.
  * @note   none.
  */
#define pwc_ldo_output_voltage_set(val)  (PWC->ldoov_bit.ldoovsel = val)

/** @defgroup PWC_exported_types
  * @{
  */

/**
  * @brief pwc pvm voltage type
  */
typedef enum
{
  PWC_PVM_VOLTAGE_2V3                    = 0x01, /*!< power voltage monitoring boundary 2.3v */
  PWC_PVM_VOLTAGE_2V4                    = 0x02, /*!< power voltage monitoring boundary 2.4v */
  PWC_PVM_VOLTAGE_2V5                    = 0x03, /*!< power voltage monitoring boundary 2.5v */
  PWC_PVM_VOLTAGE_2V6                    = 0x04, /*!< power voltage monitoring boundary 2.6v */
  PWC_PVM_VOLTAGE_2V7                    = 0x05, /*!< power voltage monitoring boundary 2.7v */
  PWC_PVM_VOLTAGE_2V8                    = 0x06, /*!< power voltage monitoring boundary 2.8v */
  PWC_PVM_VOLTAGE_2V9                    = 0x07  /*!< power voltage monitoring boundary 2.9v */
} pwc_pvm_voltage_type;

/**
  * @brief pwc ldo output voltage type
  */
typedef enum
{
  PWC_LDO_OUTPUT_1V1                     = 0x01, /*!< ldo output voltage is 1.1v */
  PWC_LDO_OUTPUT_1V2                     = 0x02, /*!< ldo output voltage is 1.2v */
  PWC_LDO_OUTPUT_1V3                     = 0x03  /*!< ldo output voltage is 1.3v */
} pwc_ldo_output_voltage_type;

/**
  * @brief pwc sleep enter type
  */
typedef enum
{
  PWC_SLEEP_ENTER_WFI                    = 0x00, /*!< use wfi enter sleep mode */
  PWC_SLEEP_ENTER_WFE                    = 0x01  /*!< use wfe enter sleep mode */
} pwc_sleep_enter_type ;

/**
  * @brief pwc deep sleep enter type
  */
typedef enum
{
  PWC_DEEP_SLEEP_ENTER_WFI               = 0x00, /*!< use wfi enter deepsleep mode */
  PWC_DEEP_SLEEP_ENTER_WFE               = 0x01  /*!< use wfe enter deepsleep mode */
} pwc_deep_sleep_enter_type ;

/**
  * @brief pwc regulator type
  */
typedef enum
{
  PWC_REGULATOR_ON                       = 0x00, /*!< voltage regulator state on when deepsleep mode */
  PWC_REGULATOR_EXTRA_LOW_POWER          = 0x01  /*!< voltage regulator state extra low power when deepsleep mode */
} pwc_regulator_type ;

/**
  * @brief type define pwc register all
  */
typedef struct
{
  /**
    * @brief pwc ctrl register, offset:0x00
    */
  union
  {
    __IO uint32_t ctrl;
    struct
    {
      __IO uint32_t vrsel                : 1; /* [0] */
      __IO uint32_t lpsel                : 1; /* [1] */
      __IO uint32_t clswef               : 1; /* [2] */
      __IO uint32_t clsef                : 1; /* [3] */
      __IO uint32_t pvmen                : 1; /* [4] */
      __IO uint32_t pvmsel               : 3; /* [7:5] */
      __IO uint32_t bpwen                : 1; /* [8] */
      __IO uint32_t reserved1            : 23;/* [31:9] */
    } ctrl_bit;
  };

  /**
    * @brief pwc ctrlsts register, offset:0x04
    */
  union
  {
    __IO uint32_t ctrlsts;
    struct
    {
      __IO uint32_t swef                 : 1; /* [0] */
      __IO uint32_t sef                  : 1; /* [1] */
      __IO uint32_t pvmof                : 1; /* [2] */
      __IO uint32_t reserved1            : 5; /* [7:3] */
      __IO uint32_t swpen1               : 1; /* [8] */
      __IO uint32_t swpen2               : 1; /* [9] */
      __IO uint32_t reserved2            : 3; /* [12:10] */
      __IO uint32_t swpen6               : 1; /* [9] */
      __IO uint32_t reserved3            : 18; /* [31:14] */
    } ctrlsts_bit;
  };

  __IO uint32_t reserved1[2];

  /**
    * @brief pwc ldoov register, offset:0x10
    */
  union
  {
    __IO uint32_t ldoov;
    struct
    {
      __IO uint32_t ldoovsel             : 2; /* [1:0] */
      __IO uint32_t reserved1            : 2; /* [3:2] */
      __IO uint32_t vrexlpen             : 1; /* [4] */
      __IO uint32_t reserved2            : 27;/* [31:5] */
    } ldoov_bit;
  };

} pwc_type;

/**
  * @}
  */

#define PWC                              ((pwc_type *) PWC_BASE)

/** @defgroup PWC_exported_functions
  * @{
  */

void pwc_reset(void);
void pwc_battery_powered_domain_access(confirm_state new_state);
void pwc_pvm_level_select(pwc_pvm_voltage_type pvm_voltage);
void pwc_power_voltage_monitor_enable(confirm_state new_state);
void pwc_wakeup_pin_enable(uint32_t pin_num, confirm_state new_state);
void pwc_flag_clear(uint32_t pwc_flag);
flag_status pwc_flag_get(uint32_t pwc_flag);
void pwc_sleep_mode_enter(pwc_sleep_enter_type pwc_sleep_enter);
void pwc_deep_sleep_mode_enter(pwc_deep_sleep_enter_type pwc_deep_sleep_enter);
void pwc_voltage_regulate_set(pwc_regulator_type pwc_regulator);
void pwc_standby_mode_enter(void);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif
