/**
  ******************************************************************************
  * @file    stm32_eval_legacy.h
  * @author  MCD Application Team
  * @version V5.0.2
  * @date    05-March-2012
  * @brief   This file contains defines legacy for STM32 EVAL drivers.
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
#ifndef __STM32_EVAL_LEGACY_H
#define __STM32_EVAL_LEGACY_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Button Defines Legacy */ 
#define Button_WAKEUP        BUTTON_WAKEUP
#define Button_TAMPER        BUTTON_TAMPER
#define Button_KEY           BUTTON_KEY
#define Button_RIGHT         BUTTON_RIGHT
#define Button_LEFT          BUTTON_LEFT
#define Button_UP            BUTTON_UP
#define Button_DOWN          BUTTON_DOWN
#define Button_SEL           BUTTON_SEL
#define Mode_GPIO            BUTTON_MODE_GPIO
#define Mode_EXTI            BUTTON_MODE_EXTI
#define Button_Mode_TypeDef  ButtonMode_TypeDef
#define JOY_CENTER           JOY_SEL
#define JOY_State_TypeDef    JOYState_TypeDef 

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/ 

#ifdef __cplusplus
}
#endif

#endif /* __STM32_EVAL_LEGACY_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
