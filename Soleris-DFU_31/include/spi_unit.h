/******************** (C) COPYRIGHT 2007 Neogen Corporation ********************
* File Name          : spi_unit.h
* Author             : D.E. OLIN
* Version            : V1.1
* Date               : 11/01/2012
* Description        : Header for spi_unit.c file.
********************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_UNIT_H
#define __SPI_UNIT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_spi.h"
#include "hw_config.h"

/* Exported types ------------------------------------------------------------*/
/* Unit Data structure definition */

typedef struct
{
  uint8_t status;		// Drawer Operating status
  uint8_t KeyInput[3];		// Operator keypad input characters
  uint8_t SavedKeys[4];		// Saved characters
  uint8_t TempDigit[4];		// Converted Temperature digits
}Unit_Data_TypeDef;

/* System Status Definitions */
typedef enum
{ Unit_NotActive = 0x00,
  Unit_Active = 0x01,
  Unit_Door_Open = 0x02,
  Unit_Temp_Heating = 0x04,
  Unit_Error = 0x08,
  Unit_Read_Waiting = 0x10,
  Unit_Temp_Inrange = 0x20,
  Unit_Temp_Entry = 0x40,
  Unit_Solenoid_Active = 0x80
}Unit_Status;

/* Exported constants --------------------------------------------------------*/
/* Number of Units */
#define Max_Units		4

/* Exported macro ------------------------------------------------------------*/

/* Select UNIT_OUTPUT: Enable pin low  */
#define UNIT_ENABLE_LOW()     GPIO_ResetBits(GPIOB, GPIO_Pin_12)
/* Deselect UNIT_OUTPUT: Enable pin high */
#define UNIT_ENABLE_HIGH()    GPIO_SetBits(GPIOB, GPIO_Pin_12)

/* Select UNIT OUTPUT: Latch pin low  */
#define UNIT_LATCH_LOW()     GPIO_ResetBits(GPIOB, GPIO_Pin_11)
/* Deselect UNIT OUTPUT: Latch pin high */
#define UNIT_LATCH_HIGH()    GPIO_SetBits(GPIOB, GPIO_Pin_11)

#define Sensor_Port     GPIOB
#define SENSOR_DATA_OUT	GPIO_Pin_10	/* PB.10 */
#define SENSOR_DATA_IN	GPIO_Pin_14	/* PB.14 */

#define DSelect_Port     GPIOC

#define LED_OUTPUT	GPIO_Pin_12	/* PB.12 */

/* Chip select definitions device selection bytes */
#define Drawer_0		0x100
#define Drawer_1		0x200
#define Drawer_2		0x400
#define Drawer_3		0x800
#define Drawer_None		0x000
#define Drawer_All		0xF00

/* Drawer device selection bytes */
#define Select_Nothing		0x00  /* Disable all devices */ 
#define Select_Column_0		0x01  /* Column selection prefixes */ 
#define Select_Column_1		0x02
#define Select_Column_2		0x04
#define Select_Column_3		0x08
#define Select_T_Sensor		0x10
#define Select_T_Output		0x20
#define Select_Door		0x40
#define Select_Display		0x80
#define Select_All		0xFF /* Select All devices */

/* Heating/Cooling select bits */
#define Heating_Bit		0x80
#define Cooling_Bit		0x40
#define No_HeatorCool		0x00

/* Exported functions ------------------------------------------------------- */
/*----- High layer function -----*/
void SPI_UNIT_Init(void);
void UNIT_Send_Command(uint8_t ID, uint8_t d0, uint8_t d1, uint8_t d2,
                       uint8_t cmd, uint8_t NumByteToWrite);
void UNIT_Reset(uint8_t DrwID);
void UNIT_Send_TDAC(uint8_t DrwID, uint8_t HC, uint16_t DacValue);
uint8_t UNIT_Read_Door_Status(uint8_t DrwID);
uint8_t UNIT_Read_Keypad(uint8_t DrwID);
void UNIT_Display_Full_Bright(uint8_t DrwID);
void UNIT_Display_Enable(uint8_t DrwID);
void UNIT_Display_Reset(uint8_t DrwID);
void UNIT_Display_String(uint8_t DrwID, uint8_t line, uint8_t col,
                        uint8_t* pBuffer, uint8_t NumByteToWrite);

/*----- Low layer function -----*/
void UNIT_Latch_Data(void);
u8 UNIT_SendByte(u8 byte);
u8 UNIT_ReadByte(void);
void UNIT_Deselect_All(void);
void Time_Delay_1_uSec(uint16_t counter);

#endif /* __SPI_UNIT_H */

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
