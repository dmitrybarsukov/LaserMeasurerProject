/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_ll_adc.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_crs.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define HVFeedback_Pin LL_GPIO_PIN_0
#define HVFeedback_GPIO_Port GPIOA
#define WTF_PWM_Pin LL_GPIO_PIN_1
#define WTF_PWM_GPIO_Port GPIOA
#define BatVoltage_Pin LL_GPIO_PIN_2
#define BatVoltage_GPIO_Port GPIOA
#define Modulation_enable_Pin LL_GPIO_PIN_3
#define Modulation_enable_GPIO_Port GPIOA
#define Measure_In_Pin LL_GPIO_PIN_4
#define Measure_In_GPIO_Port GPIOA
#define LaserChange_Pin LL_GPIO_PIN_6
#define LaserChange_GPIO_Port GPIOA
#define Buzzer_Pin LL_GPIO_PIN_7
#define Buzzer_GPIO_Port GPIOA
#define LaserEnable2_Pin LL_GPIO_PIN_0
#define LaserEnable2_GPIO_Port GPIOB
#define Backlignt_Pin LL_GPIO_PIN_1
#define Backlignt_GPIO_Port GPIOB
#define LCD0_Pin LL_GPIO_PIN_2
#define LCD0_GPIO_Port GPIOB
#define LCD1_Pin LL_GPIO_PIN_10
#define LCD1_GPIO_Port GPIOB
#define LCD2_Pin LL_GPIO_PIN_11
#define LCD2_GPIO_Port GPIOB
#define LCD3_Pin LL_GPIO_PIN_12
#define LCD3_GPIO_Port GPIOB
#define LCD4_Pin LL_GPIO_PIN_13
#define LCD4_GPIO_Port GPIOB
#define CKLIN_Pin LL_GPIO_PIN_9
#define CKLIN_GPIO_Port GPIOA
#define CKLIN_EXTI_IRQn EXTI4_15_IRQn
#define LaserEnable_Pin LL_GPIO_PIN_10
#define LaserEnable_GPIO_Port GPIOA
#define SCL_Pin LL_GPIO_PIN_11
#define SCL_GPIO_Port GPIOA
#define SDA_Pin LL_GPIO_PIN_12
#define SDA_GPIO_Port GPIOA
#define HV_PWM_Pin LL_GPIO_PIN_4
#define HV_PWM_GPIO_Port GPIOB
#define SupplyEnable_Pin LL_GPIO_PIN_5
#define SupplyEnable_GPIO_Port GPIOB
#define Button_Pin LL_GPIO_PIN_8
#define Button_GPIO_Port GPIOB
#define KeyboardInput1_Pin LL_GPIO_PIN_9
#define KeyboardInput1_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
