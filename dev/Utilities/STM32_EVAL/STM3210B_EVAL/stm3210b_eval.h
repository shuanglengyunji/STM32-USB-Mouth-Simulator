/**
  ******************************************************************************
  * @file    stm3210b_eval.h
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

typedef enum 
{ 
  JOY_NONE 	= 0,
  JOY_DOWN 	= 1,
  JOY_LEFT 	= 2,
  JOY_RIGHT = 3,
  JOY_UP 	= 4
} JOYState_TypeDef;

/** 
  * @brief  Define for STM3210B_EVAL board  
  */ 
#if !defined (USE_STM3210B_EVAL)
 #define USE_STM3210B_EVAL
#endif

//LED

#define LED1							1
#define LED1_PIN                        GPIO_Pin_13
#define LED1_GPIO_PORT                  GPIOC
#define LED1_GPIO_CLK                   RCC_APB2Periph_GPIOC

#define LED2							2
#define LED2_PIN                        GPIO_Pin_13
#define LED2_GPIO_PORT                  GPIOC
#define LED2_GPIO_CLK                   RCC_APB2Periph_GPIOC

#define LED3							3
#define LED3_PIN                        GPIO_Pin_13
#define LED3_GPIO_PORT                  GPIOC
#define LED3_GPIO_CLK                   RCC_APB2Periph_GPIOC

#define LED4							4
#define LED4_PIN                        GPIO_Pin_13
#define LED4_GPIO_PORT                  GPIOC
#define LED4_GPIO_CLK                   RCC_APB2Periph_GPIOC

//PUSH BUTTON

#define PUSH_BUTTON1					1
#define PUSH_BUTTON1_PIN                GPIO_Pin_14
#define PUSH_BUTTON1_GPIO_PORT          GPIOC
#define PUSH_BUTTON1_GPIO_CLK           RCC_APB2Periph_GPIOC

#define PUSH_BUTTON2					2
#define PUSH_BUTTON2_PIN                GPIO_Pin_15
#define PUSH_BUTTON2_GPIO_PORT          GPIOC
#define PUSH_BUTTON2_GPIO_CLK           RCC_APB2Periph_GPIOC

#define PUSH_BUTTON3					3
#define PUSH_BUTTON3_PIN                GPIO_Pin_1
#define PUSH_BUTTON3_GPIO_PORT          GPIOA
#define PUSH_BUTTON3_GPIO_CLK           RCC_APB2Periph_GPIOA

void STM_EVAL_LED1_Init(void);
void STM_EVAL_LED234_Init(void);
void STM_EVAL_LEDOn(u8 led);
void STM_EVAL_LEDOff(u8 led);
void STM_EVAL_LEDToggle(u8 led);

void STM_EVAL_PBInit(void);
uint32_t STM_EVAL_PBGetState(u8 pushbutton);

void STM_EVAL_USART1_Init(void);
void STM_EVAL_USART3_Init(void);
    
#ifdef __cplusplus
}
#endif
  
#endif /* __STM3210B_EVAL_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
