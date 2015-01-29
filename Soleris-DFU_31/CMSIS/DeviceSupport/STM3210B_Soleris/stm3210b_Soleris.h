/**
  ******************************************************************************
  * @file    stm3210b_SOLERIS.h
  * @author  MCD Application Team
  * @version V5.0.1
  * @date    05-March-2012
  * @brief   This file contains definitions for STM3210B_SOLERIS's Leds, push-buttons
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
#ifndef __STM3210B_SOLERIS_H
#define __STM3210B_SOLERIS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "platform_config.h"


/** @addtogroup Utilities
  * @{
  */

/** @addtogroup STM32_SOLERIS
  * @{
  */
    
/** @addtogroup STM3210B_SOLERIS
  * @{
  */ 

/** @addtogroup STM3210B_SOLERIS_LOW_LEVEL
  * @{
  */

/** @defgroup STM3210B_SOLERIS_LOW_LEVEL_Exported_Constants
  * @{
  */ 

/** 
  * @brief  Define for STM3210B_SOLERIS board
  */ 
#if !defined (USE_STM3210B_EVAL)
 #define USE_STM3210B_EVAL
#endif

/** @addtogroup STM3210B_SOLERIS_LOW_LEVEL_LED
  * @{
  */
#define LED_PIN                         GPIO_Pin_12
#define LED_GPIO_PORT                   GPIOC
#define LED_GPIO_CLK                    RCC_APB2Periph_GPIOC
  
/**
  * @}
  */ 
  
/** @addtogroup STM3210B_SOLERIS_SELECTION
  * @{
  */  
/**
 * @brief Drawer Selection Bits
 */
#define DRAWER_SEL_GPIO_PORT            	GPIOC
#define DRAWER_SEL_GPIO_CLK          		RCC_APB2Periph_GPIOC
#define DRAWER_SEL_0_PIN                	GPIO_Pin_8
#define DRAWER_SEL_1_PIN                	GPIO_Pin_9
#define DRAWER_SEL_2_PIN                	GPIO_Pin_10
#define DRAWER_SEL_3_PIN                	GPIO_Pin_11
/**
  * @}
  */ 


/** @addtogroup STM3210B_SOLERIS_LOW_LEVEL_UNIT_SPI
  * @{
  */
/**
  * @brief  unit SPI Interface pins
  */
#define Unit_SPI                     		SPI2
#define Unit_SPI_CLK                   	    RCC_APB2Periph_SPI2
#define Unit_SPI_SCK_PIN                	GPIO_Pin_13                  /* PB.13 */
#define Unit_SPI_SCK_GPIO_PORT           	GPIOB                       /* GPIOB */
#define Unit_SPI_SCK_GPIO_CLK          	    RCC_APB2Periph_GPIOB
#define Unit_SPI_MISO_PIN               	GPIO_Pin_14                  /* PB.14 */
#define Unit_SPI_MISO_GPIO_PORT        	    GPIOB                       /* GPIOB */
#define Unit_SPI_MISO_GPIO_CLK         	    RCC_APB2Periph_GPIOB
#define Unit_SPI_MOSI_PIN              	    GPIO_Pin_15                  /* PB.15 */
#define Unit_SPI_MOSI_GPIO_PORT        	    GPIOB                       /* GPIOB */
#define Unit_SPI_MOSI_GPIO_CLK         	    RCC_APB2Periph_GPIOB
#define Unit_CS_PIN                    	    GPIO_Pin_12                 /* PB.12 */
#define Unit_CS_GPIO_PORT              	    GPIOB                       /* GPIOB */
#define Unit_CS_GPIO_CLK                	RCC_APB2Periph_GPIOB
#define Unit_LATCH_PIN                  	GPIO_Pin_11                  /* PB.11 */
#define Unit_LATCH_GPIO_PORT            	GPIOB                       /* GPIOB */
#define Unit_LATCH_GPIO_CLK             	RCC_APB2Periph_GPIOB
#define Unit_1_WIRE_PIN                  	GPIO_Pin_10                  /* PB.10 */
#define Unit_1_WIRE_GPIO_PORT            	GPIOB                       /* GPIOB */
#define Unit_1_WIRE_GPIO_CLK             	RCC_APB2Periph_GPIOE

/**
  * @}
  */

/** @addtogroup STM3210B_SOLERIS_LOW_LEVEL_M25P_FLASH_SPI
  * @{
  */
/**
  * @brief  M25P FLASH SPI Interface pins
  */  
#define sFLASH_SPI                       SPI1
#define sFLASH_SPI_CLK                   RCC_APB2Periph_SPI1
#define sFLASH_SPI_SCK_PIN               GPIO_Pin_5                  /* PA.05 */
#define sFLASH_SPI_SCK_GPIO_PORT         GPIOA                       /* GPIOA */
#define sFLASH_SPI_SCK_GPIO_CLK          RCC_APB2Periph_GPIOA
#define sFLASH_SPI_MISO_PIN              GPIO_Pin_6                  /* PA.06 */
#define sFLASH_SPI_MISO_GPIO_PORT        GPIOA                       /* GPIOA */
#define sFLASH_SPI_MISO_GPIO_CLK         RCC_APB2Periph_GPIOA
#define sFLASH_SPI_MOSI_PIN              GPIO_Pin_7                  /* PA.07 */
#define sFLASH_SPI_MOSI_GPIO_PORT        GPIOA                       /* GPIOA */
#define sFLASH_SPI_MOSI_GPIO_CLK         RCC_APB2Periph_GPIOA
#define sFLASH_CS_PIN                    GPIO_Pin_4                  /* PA.04 */
#define sFLASH_CS_GPIO_PORT              GPIOA                       /* GPIOA */
#define sFLASH_CS_GPIO_CLK               RCC_APB2Periph_GPIOA 

/**
  * @}
  */


/** @defgroup STM3210B_SOLERIS_LOW_LEVEL_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup STM3210B_SOLERIS_LOW_LEVEL_Exported_Functions
  * @{
  */ 
uint8_t STM_SOLERIS_FindDrawers();
void STM_SOLERIS_LEDInit(void);
void STM_SOLERIS_LEDOn(void);
void STM_SOLERIS_LEDOff(void);
void STM_Soleris_Unit_Init(void);
ErrorStatus tsensor_Reset(uint8_t DrwID);
uint8_t STM_SOLERIS_CheckKeypads();
void STM_SOLERIS_ShowDFUMode(void);
void sFLASH_LowLevel_DeInit(void);
void sFLASH_LowLevel_Init(void); 
 
/**
  * @}
  */ 
    
#ifdef __cplusplus
}
#endif
  
#endif /* __STM3210B_SOLERIS_H */
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
