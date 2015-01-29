/**
  ******************************************************************************
  * @file    stm3210b_SOLERIS.c
  * @author  MCD Application Team
  * @version V5.0.1
  * @date    05-March-2012
  * @brief   This file provides
  *            - set of firmware functions to manage Leds, push-button and COM ports
  *            - low level initialization functions for SD card (on SPI), SPI serial
  *              flash (sFLASH) and temperature sensor (LM75)
  *          available on STM3210B-EVAL evaluation board from STMicroelectronics.    
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
  
/* Includes ------------------------------------------------------------------*/
#include "stm3210b_Soleris.h"
#include "spi_unit.h"
#include "Comm_1_Wire.h"

/** @addtogroup Utilities
  * @{
  */ 

/** @addtogroup STM32_SOLERIS
  * @{
  */ 

extern Unit_Data_TypeDef	Unit_Data[Max_Units];

/** @addtogroup STM3210B_SOLERIS
  * @{
  */ 
      
/** @defgroup STM3210B_SOLERIS_LOW_LEVEL
  * @brief This file provides firmware functions to manage Leds, push-buttons, 
  *        COM ports, SD card on SPI, serial flash (sFLASH), serial EEPROM (sEE) 
  *        and temperature sensor (LM75) available on STM3210B-EVAL evaluation
  *        board from STMicroelectronics.
  * @{
  */ 

/** @defgroup STM3210B_SOLERIS_LOW_LEVEL_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM3210B_SOLERIS_LOW_LEVEL_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM3210B_SOLERIS_LOW_LEVEL_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM3210B_SOLERIS_LOW_LEVEL_Private_Variables
  * @{
  */ 

/**
  * @}
  */ 


/** @defgroup STM3210B_SOLERIS_LOW_LEVEL_Private_FunctionPrototypes
  * @{
  */ 

/**
  * @}
  */ 


/** @defgroup STM3210B_SOLERIS_LOW_LEVEL_Private_Functions
  * @{
  */ 

/*******************************************************************************
* Function Name  : STM_SOLERIS_FindDrawers
* Description    : Looks for temp sensor on each drawer
* Input          : None
* Output         : number found
* Return         : None
*******************************************************************************/
uint8_t STM_SOLERIS_FindDrawers()
{
  uint8_t Ustr[16] = {"       0        "};
  uint8_t n,m;

  //Reset all the units and make their status inactive
  n = Max_Units;
  m = 0;

  do
  {
    n--;

    Ustr[7] = '0' + n;		// Drawer ID

    UNIT_Reset(n);

    Delay(0);  			// 1 mSec Delay

    UNIT_Display_Reset(n);			// And initialize the display
    Unit_Data[n].status = Unit_Active;		//Assume it's there
    if (tsensor_Reset(n) == SUCCESS)
    {// Found one

    	m++;
    	UNIT_Display_String(n, 1, 0, &Ustr[0], 16);
        Unit_Data[n].status = Unit_Active;
        UNIT_Deselect_All();
    }
    else
    {
    	Unit_Data[n].status = Unit_NotActive;
    	UNIT_Deselect_All();
    }

  }
  while (n);

  return m;
}

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *   This parameter can be one of following parameters:
  * @retval None
  */
void STM_SOLERIS_LEDInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_APB2PeriphClockCmd(LED_GPIO_CLK, ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  Turns selected LED On.
  * @param  None
  * @retval None
  */
void STM_SOLERIS_LEDOn(void)
{
	LED_GPIO_PORT->BSRR = GPIO_Pin_12;
}

/**
  * @brief  Turns selected LED Off.
  * @param  None
  * @retval None
  */
void STM_SOLERIS_LEDOff(void)
{
	LED_GPIO_PORT->BRR = GPIO_Pin_12;
}

/**
  * @brief  Configures Unit interface signal
  * @param  None
  * @retval None
  */
void STM_Soleris_Unit_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the USB select Line */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_DISCONNECT, ENABLE);

  /* First Deactivate the USB Output */
  USB_Cable_Config (DISABLE);

  /* Configure the USB Select pin as Output push-pull */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);

  /* Enable the Drawer Select Port Clock */
  RCC_APB2PeriphClockCmd(DRAWER_SEL_GPIO_CLK, ENABLE);

  /* First Deactivate the output pins (Board_Select 0 - 3) and alternate latch clock */
  GPIO_SetBits(DRAWER_SEL_GPIO_PORT, DRAWER_SEL_0_PIN | DRAWER_SEL_1_PIN |
		  DRAWER_SEL_2_PIN | DRAWER_SEL_3_PIN);

  /* Configure the Select pins as Output push-pull */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = DRAWER_SEL_0_PIN | DRAWER_SEL_1_PIN | DRAWER_SEL_2_PIN | DRAWER_SEL_3_PIN;
  GPIO_Init(DRAWER_SEL_GPIO_PORT, &GPIO_InitStructure);

  /* Enable the Unit Port Clock */
  RCC_APB2PeriphClockCmd(Unit_CS_GPIO_CLK| RCC_APB2Periph_AFIO, ENABLE);


  /* Configure Secondary SPI_MISO (PB.14) as input pull-up */
  GPIO_InitStructure.GPIO_Pin = Unit_SPI_MISO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configure the Latch pin as Output push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = Unit_LATCH_PIN;
  GPIO_Init(Unit_LATCH_GPIO_PORT, &GPIO_InitStructure);

  /* Configure the CS pin as Output push-pull */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = Unit_CS_PIN;
  GPIO_Init(Unit_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect the Drawers: Chip Select high */
  UNIT_ENABLE_HIGH();

  /* Now Deactivate the 1-Wire Interface */
  GPIO_SetBits(Unit_1_WIRE_GPIO_PORT, Unit_1_WIRE_PIN);

  /* Configure Sensor 1-Wire Interface (PB.10) as Open Drain */
  GPIO_InitStructure.GPIO_Pin = Unit_1_WIRE_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(Unit_1_WIRE_GPIO_PORT, &GPIO_InitStructure);

  SPI_UNIT_Init();
}

/*******************************************************************************
* Function Name  : tsensor_Reset
* Description    : Initializes the 18B20 sensor.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
ErrorStatus tsensor_Reset(uint8_t DrwID)
{

  UNIT_Send_Command(DrwID, 0, 0, 0, Select_T_Sensor, 1);

 /* Loop while DR register in not emplty */
  while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);

#if defined(USE_SENSOR_TIMER_DMA)

  /* Disable SPI2  */
  SPI_Cmd(SPI2, DISABLE);

#endif

  UNIT_ENABLE_LOW();

  if (Initialize_1_Wire() == ERROR)
  {
    UNIT_ENABLE_HIGH();
    return ERROR;
  }
  else
    UNIT_ENABLE_HIGH();
    return SUCCESS;
}

/**
  * @brief  Checks for a double key press in a 5 second interval.
  * @param  None
  * @retval 0 = no double key presses, 1 = double keypresses.
  */
uint8_t STM_SOLERIS_CheckKeypads(void)
{
	  uint8_t n,m;

	  STM_SOLERIS_LEDOn();
	  for (m=0; m<20; m++)		// 20 250mSec intervals = 5 seconds
	  {
		  //Check all the drawers for a double key press
		  n = Max_Units;
		  do
		  {
		    n--;

		    if (Unit_Data[n].status == Unit_Active)		//Is this drawer found?
		    {
				if (UNIT_Read_Keypad(n) & 0x80)
				{
					STM_SOLERIS_LEDOff();
					return 1;
				}
		    }

		    Delay(0);  			// 1 mSec Delay

		  }
		  while (n);

		  Delay(24);  			// 250 mSec Delay total

	  }

	  STM_SOLERIS_LEDOff();
	  return 0;
}

/*******************************************************************************
* Function Name  : STM_SOLERIS_ShowDFUMode
* Description    : Writes the DFU MODE message to each drawer
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void STM_SOLERIS_ShowDFUMode(void)
{
  uint8_t Dstr[16] = {"    DFU MODE    "};
  uint8_t Cstr[16] = {"                "};
  uint8_t n;

  //Reset all the units and make their status inactive
  n = Max_Units;

  do
  {
    n--;

	UNIT_Display_String(n, 0, 0, &Dstr[0], 16);
	UNIT_Display_String(n, 1, 0, &Cstr[0], 16);
	UNIT_Deselect_All();

  }
  while (n);
}

/**
  * @brief  DeInitializes the peripherals used by the SPI FLASH driver.
  * @param  None
  * @retval None
  */
void sFLASH_LowLevel_DeInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /*!< Disable the sFLASH_SPI  */
  SPI_Cmd(sFLASH_SPI, DISABLE);
  
  /*!< DeInitializes the sFLASH_SPI */
  SPI_I2S_DeInit(sFLASH_SPI);
  
  /*!< sFLASH_SPI Periph clock disable */
  RCC_APB2PeriphClockCmd(sFLASH_SPI_CLK, DISABLE);
  
  /*!< Configure sFLASH_SPI pins: SCK */
  GPIO_InitStructure.GPIO_Pin = sFLASH_SPI_SCK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(sFLASH_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure sFLASH_SPI pins: MISO */
  GPIO_InitStructure.GPIO_Pin = sFLASH_SPI_MISO_PIN;
  GPIO_Init(sFLASH_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure sFLASH_SPI pins: MOSI */
  GPIO_InitStructure.GPIO_Pin = sFLASH_SPI_MOSI_PIN;
  GPIO_Init(sFLASH_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure sFLASH_CS_PIN pin: sFLASH Card CS pin */
  GPIO_InitStructure.GPIO_Pin = sFLASH_CS_PIN;
  GPIO_Init(sFLASH_CS_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  Initializes the peripherals used by the SPI FLASH driver.
  * @param  None
  * @retval None
  */
void sFLASH_LowLevel_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /*!< sFLASH_SPI_CS_GPIO, sFLASH_SPI_MOSI_GPIO, sFLASH_SPI_MISO_GPIO 
       and sFLASH_SPI_SCK_GPIO Periph clock enable */
  RCC_APB2PeriphClockCmd(sFLASH_CS_GPIO_CLK | sFLASH_SPI_MOSI_GPIO_CLK | sFLASH_SPI_MISO_GPIO_CLK |
                         sFLASH_SPI_SCK_GPIO_CLK, ENABLE);

  /*!< sFLASH_SPI Periph clock enable */
  RCC_APB2PeriphClockCmd(sFLASH_SPI_CLK, ENABLE);
  
  /*!< Configure sFLASH_SPI pins: SCK */
  GPIO_InitStructure.GPIO_Pin = sFLASH_SPI_SCK_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(sFLASH_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure sFLASH_SPI pins: MOSI */
  GPIO_InitStructure.GPIO_Pin = sFLASH_SPI_MOSI_PIN;
  GPIO_Init(sFLASH_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure sFLASH_SPI pins: MISO */
  GPIO_InitStructure.GPIO_Pin = sFLASH_SPI_MISO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
  GPIO_Init(sFLASH_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
  
  /*!< Configure sFLASH_CS_PIN pin: sFLASH Card CS pin */
  GPIO_InitStructure.GPIO_Pin = sFLASH_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(sFLASH_CS_GPIO_PORT, &GPIO_InitStructure);
}


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
    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
