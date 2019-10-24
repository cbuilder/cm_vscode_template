/**
  ******************************************************************************
  * @file    stm32f3xx_nucleo.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    25-Aug-2014
  * @brief   This file contains definitions for:
  *          - LEDs and push-button available on STM32F3XX-Nucleo Kit 
  *            from STMicroelectronics
  *          - LCD, joystick and microSD available on Adafruit 1.8" TFT LCD 
  *            shield (reference ID 802)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#ifndef __STM32F3XX_NUCLEO_H
#define __STM32F3XX_NUCLEO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/** @addtogroup BSP
  * @{
  */
  
/** @addtogroup STM32F3XX_NUCLEO
  * @{
  */
      
/** @addtogroup STM32F3XX_NUCLEO_LOW_LEVEL
  * @{
  */ 

/** @defgroup STM32F3XX_NUCLEO_LOW_LEVEL_Exported_Types 
  * @{
  */ 
typedef enum 
{
  LED2 = 0,
  LED_GREEN = LED2
} Led_TypeDef;

typedef enum 
{  
  BUTTON_USER = 0
} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef; 

typedef enum 
{ 
  JOY_NONE  = 0,
  JOY_SEL   = 1,
  JOY_DOWN  = 2,
  JOY_LEFT  = 3,
  JOY_RIGHT = 4,
  JOY_UP    = 5
} JOYState_TypeDef;

/**
  * @}
  */ 

/** @defgroup STM32F3XX_NUCLEO_LOW_LEVEL_Exported_Constants 
  * @{
  */ 

/** 
* @brief	Define for STM32F3XX_NUCLEO board  
*/ 
#if !defined (USE_STM32F3XX_NUCLEO)
 #define USE_STM32F3XX_NUCLEO
#endif

/** @addtogroup STM32F3XX_NUCLEO_LOW_LEVEL_LED
  * @{
  */
#define LEDn                              1

#if defined(STM32F302x8)
#define LED2_PIN                          GPIO_PIN_13
#define LED2_GPIO_PORT                    GPIOB
#define LED2_GPIO_CLK_ENABLE()            __GPIOB_CLK_ENABLE()  
#define LED2_GPIO_CLK_DISABLE()           __GPIOB_CLK_DISABLE()

#elif defined(STM32F334x8) || defined(STM32F303xE)
#define LED2_PIN                          GPIO_PIN_5
#define LED2_GPIO_PORT                    GPIOA
#define LED2_GPIO_CLK_ENABLE()            __GPIOA_CLK_ENABLE()  
#define LED2_GPIO_CLK_DISABLE()           __GPIOA_CLK_DISABLE()

#else
#error "MCU device not yet supported on STM32 F3 Nucleo kit"
#endif /* STM32F302x8 */


#define LEDx_GPIO_CLK_ENABLE(__INDEX__)   (((__INDEX__) == 0) ? LED2_GPIO_CLK_ENABLE() : 0)
#define LEDx_GPIO_CLK_DISABLE(__INDEX__)  (((__INDEX__) == 0) ? LED2_GPIO_CLK_DISABLE() : 0)
/**
  * @}
  */ 
  
/** @addtogroup STM32F3XX_NUCLEO_LOW_LEVEL_BUTTON
  * @{
  */  
#define BUTTONn                           1

/**
 * @brief User button
 */
#define USER_BUTTON_PIN                       GPIO_PIN_13
#define USER_BUTTON_GPIO_PORT                 GPIOC
#define USER_BUTTON_GPIO_CLK_ENABLE()         __GPIOC_CLK_ENABLE()
#define USER_BUTTON_GPIO_CLK_DISABLE()        __GPIOC_CLK_DISABLE()
#define USER_BUTTON_EXTI_IRQn                 EXTI15_10_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    (((__INDEX__) == 0) ? USER_BUTTON_GPIO_CLK_ENABLE() : 0)
#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)   (((__INDEX__) == 0) ? USER_BUTTON_GPIO_CLK_DISABLE() : 0)
/**
  * @}
  */ 

/** @addtogroup STM32F3XX_NUCLEO_LOW_LEVEL_BUS
  * @{
  */ 
#if defined(HAL_SPI_MODULE_ENABLED)
#if defined(STM32F302x8)
/*###################### SPI2 ###################################*/
#define NUCLEO_SPIx                               SPI2
#define NUCLEO_SPIx_CLK_ENABLE()                  __SPI2_CLK_ENABLE()

#define NUCLEO_SPIx_SCK_AF                        GPIO_AF5_SPI2
#define NUCLEO_SPIx_SCK_GPIO_PORT                 GPIOB
#define NUCLEO_SPIx_SCK_PIN                       GPIO_PIN_13
#define NUCLEO_SPIx_SCK_GPIO_CLK_ENABLE()         __GPIOB_CLK_ENABLE()
#define NUCLEO_SPIx_SCK_GPIO_CLK_DISABLE()        __GPIOB_CLK_DISABLE()

#define NUCLEO_SPIx_MISO_MOSI_AF                  GPIO_AF5_SPI2
#define NUCLEO_SPIx_MISO_MOSI_GPIO_PORT           GPIOB
#define NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_ENABLE()   __GPIOB_CLK_ENABLE()
#define NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_DISABLE()  __GPIOB_CLK_DISABLE()
#define NUCLEO_SPIx_MISO_PIN                      GPIO_PIN_14
#define NUCLEO_SPIx_MOSI_PIN                      GPIO_PIN_15

#elif defined(STM32F334x8) || defined(STM32F303xE)
/*###################### SPI1 ###################################*/
#define NUCLEO_SPIx                               SPI1
#define NUCLEO_SPIx_CLK_ENABLE()                  __SPI1_CLK_ENABLE()

#define NUCLEO_SPIx_SCK_AF                        GPIO_AF5_SPI1
#define NUCLEO_SPIx_SCK_GPIO_PORT                 GPIOA
#define NUCLEO_SPIx_SCK_PIN                       GPIO_PIN_5
#define NUCLEO_SPIx_SCK_GPIO_CLK_ENABLE()         __GPIOA_CLK_ENABLE()
#define NUCLEO_SPIx_SCK_GPIO_CLK_DISABLE()        __GPIOA_CLK_DISABLE()

#define NUCLEO_SPIx_MISO_MOSI_AF                  GPIO_AF5_SPI1
#define NUCLEO_SPIx_MISO_MOSI_GPIO_PORT           GPIOA
#define NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_ENABLE()   __GPIOA_CLK_ENABLE()
#define NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_DISABLE()  __GPIOA_CLK_DISABLE()
#define NUCLEO_SPIx_MISO_PIN                      GPIO_PIN_6
#define NUCLEO_SPIx_MOSI_PIN                      GPIO_PIN_7

#else
#error "MCU device not yet supported on STM32 F3 Nucleo kit"
#endif /* STM32F302x8 */

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define NUCLEO_SPIx_TIMEOUT_MAX                   1000
#endif /* HAL_SPI_MODULE_ENABLED */
/**
  * @}
  */

/** @addtogroup STM32F0XX_NUCLEO_LOW_LEVEL_COMPONENT
  * @{
  */

/**
  * @brief  SD Control Lines management
  */
#define SD_CS_LOW()       HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH()      HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET)

/**
  * @brief  LCD Control Lines management
  */
#define LCD_CS_LOW()      HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_RESET)
#define LCD_CS_HIGH()     HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_SET)
#define LCD_DC_LOW()      HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_RESET)
#define LCD_DC_HIGH()     HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_SET)
     
/**
  * @brief  SD Control Interface pins
  */
#define SD_CS_PIN                                 GPIO_PIN_5
#define SD_CS_GPIO_PORT                           GPIOB
#define SD_CS_GPIO_CLK_ENABLE()                 __GPIOB_CLK_ENABLE()
#define SD_CS_GPIO_CLK_DISABLE()                __GPIOB_CLK_DISABLE()

/**
  * @brief  LCD Control Interface pins
  */
#define LCD_CS_PIN                                 GPIO_PIN_6
#define LCD_CS_GPIO_PORT                           GPIOB
#define LCD_CS_GPIO_CLK_ENABLE()                 __GPIOB_CLK_ENABLE()
#define LCD_CS_GPIO_CLK_DISABLE()                __GPIOB_CLK_DISABLE()
    
/**
  * @brief  LCD Data/Command Interface pins
  */
#define LCD_DC_PIN                                 GPIO_PIN_9
#define LCD_DC_GPIO_PORT                           GPIOA
#define LCD_DC_GPIO_CLK_ENABLE()                 __GPIOA_CLK_ENABLE()
#define LCD_DC_GPIO_CLK_DISABLE()                __GPIOA_CLK_DISABLE()

#if defined(HAL_ADC_MODULE_ENABLED)
#if defined(STM32F302x8) || defined(STM32F334x8)
/*##################### ADC1 ###################################*/
/**
  * @brief  ADC Interface pins
  *         used to detect motion of Joystick available on Adafruit 1.8" TFT shield
  */
#define NUCLEO_ADCx                                 ADC1
#define NUCLEO_ADCx_CLK_ENABLE()                  __ADC1_CLK_ENABLE()
#elif defined(STM32F303xE)
/*##################### ADC3 ###################################*/
/**
  * @brief  ADC Interface pins
  *         used to detect motion of Joystick available on Adafruit 1.8" TFT shield
  */
#define NUCLEO_ADCx                                 ADC3
#define NUCLEO_ADCx_CLK_ENABLE()                  __ADC34_CLK_ENABLE()
#else
#error "MCU device not yet supported on STM32 F3 Nucleo kit"
#endif /* STM32F302x8 || STM32F334x8 */

#define NUCLEO_ADCx_GPIO_PORT                       GPIOB
#define NUCLEO_ADCx_GPIO_PIN                        GPIO_PIN_0
#define NUCLEO_ADCx_GPIO_CLK_ENABLE()             __GPIOB_CLK_ENABLE()
#define NUCLEO_ADCx_GPIO_CLK_DISABLE()            __GPIOB_CLK_DISABLE()

#endif /* HAL_ADC_MODULE_ENABLED */

/**
  * @}
  */

/** @defgroup STM32F3XX_NUCLEO_LOW_LEVEL_Exported_Macros 
  * @{
  */  
/**
  * @}
  */ 


/** @defgroup STM32F3XX_NUCLEO_LOW_LEVEL_Exported_Functions 
  * @{
  */
uint32_t  BSP_GetVersion(void);  
void      BSP_LED_Init(Led_TypeDef Led);
void      BSP_LED_On(Led_TypeDef Led);
void      BSP_LED_Off(Led_TypeDef Led);
void      BSP_LED_Toggle(Led_TypeDef Led);
void      BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
uint32_t  BSP_PB_GetState(Button_TypeDef Button);
#if defined(HAL_ADC_MODULE_ENABLED)
uint8_t          BSP_JOY_Init(void);
JOYState_TypeDef BSP_JOY_GetState(void);
#endif /* HAL_ADC_MODULE_ENABLED */
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

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif /* __STM32F3XX_NUCLEO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

