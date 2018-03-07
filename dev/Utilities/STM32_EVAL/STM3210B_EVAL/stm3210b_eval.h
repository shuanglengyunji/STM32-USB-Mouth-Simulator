/**
  ******************************************************************************
  * @file    stm3210b_eval.h
  * @author  MCD Application Team
  * @version V5.0.1
  * @date    05-March-2012
  * @brief   This file contains definitions for STM3210B_EVAL's Leds, push-buttons
  *          COM ports, SD Card (on SPI), sFLASH (on SPI) and Temperature sensor 
  *          LM75 (on I2C) hardware resources.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM3210B_EVAL_H
#define __STM3210B_EVAL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32_eval_legacy.h"

/** @addtogroup Utilities
  * @{
  */

/** @addtogroup STM32_EVAL
  * @{
  */
    
/** @addtogroup STM3210B_EVAL
  * @{
  */ 

/** @addtogroup STM3210B_EVAL_LOW_LEVEL
  * @{
  */
  
/** @defgroup STM3210B_EVAL_LOW_LEVEL_Exported_Types
  * @{
  */
typedef enum 
{
  LED1 = 0,
  LED2 = 1,
  LED3 = 2,
  LED4 = 3
} Led_TypeDef;

typedef enum 
{  
  BUTTON_WAKEUP = 0,
  BUTTON_TAMPER = 1,
  BUTTON_KEY = 2,
  BUTTON_RIGHT = 3,
  BUTTON_LEFT = 4,
  BUTTON_UP = 5,
  BUTTON_DOWN = 6,
  BUTTON_SEL = 7
} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;

typedef enum 
{ 
  JOY_NONE = 0,
  JOY_SEL = 1,
  JOY_DOWN = 2,
  JOY_LEFT = 3,
  JOY_RIGHT = 4,
  JOY_UP = 5
} JOYState_TypeDef
;

typedef enum 
{
  COM1 = 0,
  COM2 = 1
} COM_TypeDef;

/**
  * @}
  */


/** @defgroup STM3210B_EVAL_LOW_LEVEL_Exported_Constants
  * @{
  */ 

/** 
  * @brief  Define for STM3210B_EVAL board  
  */ 
#if !defined (USE_STM3210B_EVAL)
 #define USE_STM3210B_EVAL
#endif

/** @addtogroup STM3210B_EVAL_LOW_LEVEL_LED
  * @{
  */
#define LEDn                             4

#define LED1_PIN                         GPIO_Pin_6
#define LED1_GPIO_PORT                   GPIOC
#define LED1_GPIO_CLK                    RCC_APB2Periph_GPIOC  
  
#define LED2_PIN                         GPIO_Pin_7
#define LED2_GPIO_PORT                   GPIOC
#define LED2_GPIO_CLK                    RCC_APB2Periph_GPIOC  

#define LED3_PIN                         GPIO_Pin_8  
#define LED3_GPIO_PORT                   GPIOC
#define LED3_GPIO_CLK                    RCC_APB2Periph_GPIOC  
  
#define LED4_PIN                         GPIO_Pin_9
#define LED4_GPIO_PORT                   GPIOC
#define LED4_GPIO_CLK                    RCC_APB2Periph_GPIOC  

/**
  * @}
  */ 
  
/** @addtogroup STM3210B_EVAL_LOW_LEVEL_BUTTON
  * @{
  */  
#define BUTTONn                          8

/**
 * @brief Wakeup push-button
 */
#define WAKEUP_BUTTON_PIN                GPIO_Pin_0
#define WAKEUP_BUTTON_GPIO_PORT          GPIOA
#define WAKEUP_BUTTON_GPIO_CLK           RCC_APB2Periph_GPIOA
#define WAKEUP_BUTTON_EXTI_LINE          EXTI_Line0
#define WAKEUP_BUTTON_EXTI_PORT_SOURCE   GPIO_PortSourceGPIOA
#define WAKEUP_BUTTON_EXTI_PIN_SOURCE    GPIO_PinSource0
#define WAKEUP_BUTTON_EXTI_IRQn          EXTI0_IRQn 
/**
 * @brief Tamper push-button
 */
#define TAMPER_BUTTON_PIN                GPIO_Pin_13
#define TAMPER_BUTTON_GPIO_PORT          GPIOC
#define TAMPER_BUTTON_GPIO_CLK           RCC_APB2Periph_GPIOC
#define TAMPER_BUTTON_EXTI_LINE          EXTI_Line13
#define TAMPER_BUTTON_EXTI_PORT_SOURCE   GPIO_PortSourceGPIOC
#define TAMPER_BUTTON_EXTI_PIN_SOURCE    GPIO_PinSource13
#define TAMPER_BUTTON_EXTI_IRQn          EXTI15_10_IRQn 
/**
 * @brief Key push-button
 */
#define KEY_BUTTON_PIN                   GPIO_Pin_9
#define KEY_BUTTON_GPIO_PORT             GPIOB
#define KEY_BUTTON_GPIO_CLK              RCC_APB2Periph_GPIOB
#define KEY_BUTTON_EXTI_LINE             EXTI_Line9
#define KEY_BUTTON_EXTI_PORT_SOURCE      GPIO_PortSourceGPIOB
#define KEY_BUTTON_EXTI_PIN_SOURCE       GPIO_PinSource9
#define KEY_BUTTON_EXTI_IRQn             EXTI9_5_IRQn
/**
 * @brief Joystick Right push-button
 */
#define RIGHT_BUTTON_PIN                 GPIO_Pin_0
#define RIGHT_BUTTON_GPIO_PORT           GPIOE
#define RIGHT_BUTTON_GPIO_CLK            RCC_APB2Periph_GPIOE
#define RIGHT_BUTTON_EXTI_LINE           EXTI_Line0
#define RIGHT_BUTTON_EXTI_PORT_SOURCE    GPIO_PortSourceGPIOE
#define RIGHT_BUTTON_EXTI_PIN_SOURCE     GPIO_PinSource0
#define RIGHT_BUTTON_EXTI_IRQn           EXTI0_IRQn
/**
 * @brief Joystick Left push-button
 */
#define LEFT_BUTTON_PIN                  GPIO_Pin_1
#define LEFT_BUTTON_GPIO_PORT            GPIOE
#define LEFT_BUTTON_GPIO_CLK             RCC_APB2Periph_GPIOE
#define LEFT_BUTTON_EXTI_LINE            EXTI_Line1
#define LEFT_BUTTON_EXTI_PORT_SOURCE     GPIO_PortSourceGPIOE
#define LEFT_BUTTON_EXTI_PIN_SOURCE      GPIO_PinSource1
#define LEFT_BUTTON_EXTI_IRQn            EXTI1_IRQn  
/**
 * @brief Joystick Up push-button
 */
#define UP_BUTTON_PIN                    GPIO_Pin_8
#define UP_BUTTON_GPIO_PORT              GPIOD
#define UP_BUTTON_GPIO_CLK               RCC_APB2Periph_GPIOD
#define UP_BUTTON_EXTI_LINE              EXTI_Line8
#define UP_BUTTON_EXTI_PORT_SOURCE       GPIO_PortSourceGPIOD
#define UP_BUTTON_EXTI_PIN_SOURCE        GPIO_PinSource8
#define UP_BUTTON_EXTI_IRQn              EXTI9_5_IRQn  
/**
 * @brief Joystick Down push-button
 */  
#define DOWN_BUTTON_PIN                  GPIO_Pin_14
#define DOWN_BUTTON_GPIO_PORT            GPIOD
#define DOWN_BUTTON_GPIO_CLK             RCC_APB2Periph_GPIOD
#define DOWN_BUTTON_EXTI_LINE            EXTI_Line14
#define DOWN_BUTTON_EXTI_PORT_SOURCE     GPIO_PortSourceGPIOD
#define DOWN_BUTTON_EXTI_PIN_SOURCE      GPIO_PinSource14
#define DOWN_BUTTON_EXTI_IRQn            EXTI15_10_IRQn  
/**
 * @brief Joystick Sel push-button
 */
#define SEL_BUTTON_PIN                   GPIO_Pin_12
#define SEL_BUTTON_GPIO_PORT             GPIOD
#define SEL_BUTTON_GPIO_CLK              RCC_APB2Periph_GPIOD
#define SEL_BUTTON_EXTI_LINE             EXTI_Line12
#define SEL_BUTTON_EXTI_PORT_SOURCE      GPIO_PortSourceGPIOD
#define SEL_BUTTON_EXTI_PIN_SOURCE       GPIO_PinSource12
#define SEL_BUTTON_EXTI_IRQn             EXTI15_10_IRQn   
/**
  * @}
  */ 

/**
  * @}
  */
  
/** @defgroup STM3210B_EVAL_LOW_LEVEL_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup STM3210B_EVAL_LOW_LEVEL_Exported_Functions
  * @{
  */ 
void STM_EVAL_LEDInit(Led_TypeDef Led);
void STM_EVAL_LEDOn(Led_TypeDef Led);
void STM_EVAL_LEDOff(Led_TypeDef Led);
void STM_EVAL_LEDToggle(Led_TypeDef Led);

void STM_EVAL_PBInit(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode);
uint32_t STM_EVAL_PBGetState(Button_TypeDef Button);

void STM_EVAL_COM1_Init(void);
 
/**
  * @}
  */ 
    
#ifdef __cplusplus
}
#endif
  
#endif /* __STM3210B_EVAL_H */
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */  

/**
  * @}
  */    

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
