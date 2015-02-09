/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
* File Name          : spi_unit.c
* Author             : D.E. OLIN
* Version            : V1.1
* Date               : 11/01/2012
* Description        : This file provides the functions needed to interface
*                      with the Soleris drawers.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "spi_unit.h"
#include "stm32f10x_rcc.h"

/* Private typedef -----------------------------------------------------------*/

#define Dummy_Byte 0xA5

/* Private define ------------------------------------------------------------*/

/* Keypad scanning bits */
#define DSP_Write_Cmd		0xF8
#define DSP_Write_Data		0xFA
#define DSP_Clear_Cmd		0x01
#define DSP_On_Cmd		0x0C
#define DSP_NoShift_Cmd		0x06
#define DSP_Full_Cmd		0x38
#define DSP_Line1_Cmd		0x80
#define DSP_Line2_Cmd		0xC0

/* Keypad scanning bits */
#define KP_Row_0		0x01
#define KP_Row_1		0x02
#define KP_Row_2		0x04
#define KP_Row_3		0x08
#define KP_Col_0		0x10
#define KP_Col_1		0x20
#define KP_Col_2		0x40
#define KP_Max_Buttons		13

/* SPI SPE mask */
#define Clk_Alt_F          ((uint32_t)0x00800000)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

SPI_InitTypeDef  SPI_InitStructure;

Unit_Data_TypeDef	Unit_Data[Max_Units];

/* Drawer Selection Bits Look-up Table */
const uint16_t Drawer_Bits[5] = { Drawer_0, Drawer_1, Drawer_2, Drawer_3,
				  Drawer_All};

/* Keypad Selection Bits Look-up Table */
const uint16_t Keypad_Bits[13] = {(KP_Row_0 + KP_Col_0), 
				  (KP_Row_0 + KP_Col_1),
                                  (KP_Row_0 + KP_Col_2),
                                  (KP_Row_1 + KP_Col_0),
                                  (KP_Row_1 + KP_Col_1),
                                  (KP_Row_1 + KP_Col_2),
                                  (KP_Row_2 + KP_Col_0),
                                  (KP_Row_2 + KP_Col_1),
                                  (KP_Row_2 + KP_Col_2),
                                  (KP_Row_3 + KP_Col_0),
                                  (KP_Row_3 + KP_Col_1),
                                  (KP_Row_3 + KP_Col_2),
				  0};

/* Keypad Decode Table */
const uint16_t Keypad_ASCII[13] = {'1', '2', '3', '4', '5', '6', 
				   '7', '8', '9', '*', '0', '#',
				  0};

/* Private function prototypes -----------------------------------------------*/

void UNIT_Display_Send_Command(uint8_t DrwID, uint8_t dcmd);

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : SPI_UNIT_Init
* Description    : Initializes the peripherals used by the SPI_unit driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_UNIT_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
   
  /* SPI2 Periph clock enable */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  /* Configure SPI2 pins: NSS, SCK, MISO and MOSI as alternate open drain
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;*/
  /* Configure SPI2 pins: NSS, SCK, MOSI as alternate open drain*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Deselect the FLASH: Chip Select high */
  UNIT_ENABLE_HIGH();

  /* Now Set the clock level for special latch control (PB.13) */
  GPIO_ResetBits(GPIOB, GPIO_Pin_13);

  /* SPI2 configuration */ 
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);
  
  /* Enable SPI2  */
  SPI_Cmd(SPI2, ENABLE);   

  /* Deselect all units  */
  UNIT_Deselect_All();
}

/*******************************************************************************
* Function Name  : UNIT_Latch_Data
* Description    : Bumps the BRD_LAT and the BRD_CLK lines.  The clock line is
*			toggled by changing the resting state defined by the
*			SPI control register.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UNIT_Latch_Data(void)
{

  uint32_t tmpreg = 0;
  uint8_t pulsecntr;

  /* Get the SPIx CR1 value */
  tmpreg = SPI2->CR1;
  tmpreg &= ~SPI_CPOL_High;

  /* Take the latch low */
  UNIT_LATCH_LOW();
  Time_Delay_1_uSec(2);

  /* Now change the clock to low  using alternate clock polarity */
  SPI2->CR1 = tmpreg;
  Time_Delay_1_uSec(2);

  /* Make the latch go high */
  UNIT_LATCH_HIGH();

  for (pulsecntr = 0; pulsecntr < 5; pulsecntr++);
  
  /* Now change the resting state of clock to high */ 
  tmpreg += SPI_CPOL_High;
  SPI2->CR1 = tmpreg;

  Time_Delay_1_uSec(10);
}

/*******************************************************************************
* Function Name  : UNIT_SendByte
* Description    : Sends a byte through the SPI interface and return the byte 
*                  received from the SPI bus.
* Input          : byte : byte to send.
* Output         : None
* Return         : The value of the received byte.
*******************************************************************************/
uint8_t UNIT_SendByte(uint8_t byte)
{
  uint8_t RByte;
  
  /* Loop while until the SPI is not busy */
  while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI2 peripheral */
  SPI_I2S_SendData(SPI2, byte);

  /* Wait to receive a byte */
  while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

  RByte = SPI_I2S_ReceiveData(SPI2);

  if (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_OVR) == SET)
    RByte = 0;

  Time_Delay_1_uSec(10);
  /* Return the byte read from the SPI bus */
  return RByte;
}

/*******************************************************************************
* Function Name  : UNIT_ReadByte
* Description    : Reads a byte from the drawer.
*                  This function must be used only if the UNIT_Send_Command
*                  function has been previously called.
* Input          : None
* Output         : None
* Return         : Byte Read from the SPI Flash.
*******************************************************************************/
u8 UNIT_ReadByte(void)
{
  return (UNIT_SendByte(Dummy_Byte));
}

/*******************************************************************************
* Function Name  : UNIT_Deselect_All
* Description    : Takes all of the select lines to a high level
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
void UNIT_Deselect_All(void)
{
  GPIO_SetBits(DSelect_Port, Drawer_All);
  Time_Delay_1_uSec(40);		//Give the mux a chance to recover
}

/*******************************************************************************
* Function Name  : UNIT_Send_Command
* Description    : Writes a block of data to the drawer. In this function, the
*                  number of bytes to write are passed, along with the data.
* Input          : - ID : Drawer Selection ( 0 - 3  or 4 = all)
*                  - D0 - D2 : data to be written.
*                  - cmd : Unit selection byte.
*                  - NumByteToWrite : number of bytes to write (including cmd)
* Output         : None
* Return         : None
*******************************************************************************/
void UNIT_Send_Command(uint8_t ID, uint8_t d0, uint8_t d1, uint8_t d2,
                       uint8_t cmd, uint8_t NumByteToWrite)
{

  /* Enable SPI2  */
  SPI_Cmd(SPI2, ENABLE);   

  GPIO_ResetBits(DSelect_Port, Drawer_Bits[ID]);		// Select the device
  
  Time_Delay_1_uSec(12);
  
  switch (NumByteToWrite)			// How many
    {
      case 4 :
        UNIT_SendByte(d0);
      case 3 :
        UNIT_SendByte(d1);
      case 2 :
        UNIT_SendByte(d2);
      case 1 :
      {
        UNIT_SendByte(cmd);
        UNIT_Latch_Data();
        break;
      }
      default:
      {
        break;
      }
    } // end switch/case (NumByteToWrite)}

}

/*******************************************************************************
* Function Name  : UNIT_Reset
* Description    : Deselects all devices in the drawer
* Input          : - ID : Drawer to be reset 
* Output         : None
* Return         : None
*******************************************************************************/
void UNIT_Reset(uint8_t DrwID)
{

  UNIT_Send_Command(DrwID, 0, 0, 0, Select_All, 4);
  UNIT_Send_Command(DrwID, 0, 0, 0, Select_Nothing, 4);
  UNIT_Deselect_All();
}

/*******************************************************************************
* Function Name  : UNIT_Send_TDAC
* Description    : Sends data to the Temperature DAC.
* Input          : - DrwID : Drawer Selection
*                  - HC : Heating/Cooling Select Bits.
*                  - DacValue : DAC Value (12 Bits).
* Output         : None
* Return         : None
*******************************************************************************/
void UNIT_Send_TDAC(uint8_t DrwID, uint8_t HC, uint16_t DacValue)
{
  uint8_t DLow, DHigh;

  /* Seperate the word into bytes*/
  DHigh = DacValue>>8;
  DLow = DacValue & 0xFF;

  GPIO_ResetBits(DSelect_Port, Drawer_Bits[DrwID]);		// Select the device

  Time_Delay_1_uSec(10);

  UNIT_ENABLE_LOW();		// Force the drawer to initialize the shift
  Time_Delay_1_uSec(5);
  UNIT_ENABLE_HIGH();		// register so that the HC bits are latched

  UNIT_Send_Command(DrwID, HC, DHigh, DLow, Select_T_Output, 4);
  UNIT_Deselect_All();
}

/*******************************************************************************
* Function Name  : UNIT_Read_Door_Status
* Description    : Reads the output from the door sensor.
* Input          : - DrwID : Drawer Selection
* Output         : None
* Return         : Byte Read from the SPI Flash (0xFF = closed, 0x00 = open).
*******************************************************************************/
uint8_t UNIT_Read_Door_Status(uint8_t DrwID)
{
  uint8_t Status;

  UNIT_Send_Command(DrwID, 0, 0, 0, Select_Door, 1);
  UNIT_ENABLE_LOW();		// Enable the output
//  Time_Delay_1_uSec(10);
  Status = UNIT_SendByte(Dummy_Byte) & 0xf0;
  UNIT_ENABLE_HIGH();		// Disable the output
  UNIT_Deselect_All();

  return (Status);
}

/*******************************************************************************
* Function Name  : UNIT_Read_Keypad
* Description    : Scans the keypad.  Returns a keycode if only ONE key is down.
* Input          : - DrwID : Drawer Selection
* Output         : None
* Return         : Keycode
*******************************************************************************/
uint8_t UNIT_Read_Keypad(uint8_t DrwID)
{
  uint8_t Keycount = 0, Keycode = 0, bits, Index;

  for (Index=0; Index<12; Index++)	//Do 12 so at the end, one line remains active
  {
    /* Form the keypad scan bits (keep solenoid state)*/
    bits = Keypad_Bits[Index] + 
      (Unit_Data[DrwID].status & Unit_Solenoid_Active);

    UNIT_Send_Command(DrwID, 0, 0, bits, Select_Display, 2);
    UNIT_ENABLE_LOW();			// Enable the output
    bits = UNIT_SendByte(Dummy_Byte) & 0xf0;	// Read the output (0=Found)

    if (bits ==0)
    {
      Keycount++;
      Keycode = Keypad_ASCII[Index];	// Get the ASCII code
    }
    UNIT_ENABLE_HIGH();			// Disable the output
  }

  if (Keycount > 1)
    Keycode += 0x80;			// Set the multiple keys indicator

  UNIT_Deselect_All();

  return Keycode;
}

/*******************************************************************************
* Function Name  : UNIT_Display_Send_Command
* Description    : Selects the display and sends a command to it.
* Input          : - DrwID : Drawer Selection
*                  - dcmd : Display command to output.
* Output         : None
* Return         : None
*******************************************************************************/
void UNIT_Display_Send_Command(uint8_t DrwID, uint8_t dcmd)
{
  uint8_t bits;

  /* Turn off the keypad scan bits (keep solenoid state)*/
  bits = (Unit_Data[DrwID].status & Unit_Solenoid_Active);

  UNIT_Send_Command(DrwID, 0, 0, bits, Select_Display, 2);

  UNIT_ENABLE_LOW();		// Enable the output

  UNIT_SendByte(DSP_Write_Cmd);	// Indicate a command is next
  UNIT_SendByte(dcmd);		// and write it

  UNIT_ENABLE_HIGH();		// Disable the output

}

/*******************************************************************************
* Function Name  : UNIT_Display_Full_Bright
* Description    : Puts the display in full brightness mode.
* Input          : - DrwID : Drawer Selection
* Output         : None
* Return         : None
*******************************************************************************/
void UNIT_Display_Full_Bright(uint8_t DrwID)
{

  UNIT_Display_Send_Command(DrwID, DSP_Full_Cmd);

  /* Turn off selection outputs*/
  UNIT_Deselect_All();

}

/*******************************************************************************
* Function Name  : UNIT_Display_Enable
* Description    : Puts the display in full brightness mode.
* Input          : - DrwID : Drawer Selection
* Output         : None
* Return         : None
*******************************************************************************/
void UNIT_Display_Enable(uint8_t DrwID)
{

  UNIT_Display_Send_Command(DrwID, DSP_On_Cmd);
  UNIT_Display_Send_Command(DrwID, DSP_NoShift_Cmd);
  UNIT_Display_Full_Bright(DrwID);
}

/*******************************************************************************
* Function Name  : UNIT_Display_Reset
* Description    : Clears the display, sets no shift, and full brightness mode.
* Input          : - DrwID : Drawer Selection
* Output         : None
* Return         : None
*******************************************************************************/
void UNIT_Display_Reset(uint8_t DrwID)
{

  UNIT_Display_Send_Command(DrwID, DSP_Clear_Cmd);
  UNIT_Display_Enable(DrwID);
}

/*******************************************************************************
* Function Name  : UNIT_Display_String
* Description    : Writes a string to the display
* Input          : - DrwID : Drawer Selection
*                  - line : Display line on which to put the string.
*                  - col : Display column in which to start.
*                  - pBuffer : Pointer to the ASCII string.
*                  - NumByteToWrite : Number of characters in the string.
* Output         : None
* Return         : None
*******************************************************************************/
void UNIT_Display_String(uint8_t DrwID, uint8_t line, uint8_t col,
                        uint8_t* pBuffer, uint8_t NumByteToWrite)
{

  uint8_t ChrCnt = 0;

  /* Encode the Command to select the line and column */
  line = (line<<6) + DSP_Line1_Cmd + col;
  
  UNIT_Display_Send_Command(DrwID, line);	// Send it

  UNIT_ENABLE_LOW();			// Enable the output

  UNIT_SendByte(DSP_Write_Data);	// Go to write data mode

  while (ChrCnt < NumByteToWrite)
  {
    UNIT_SendByte(pBuffer[ChrCnt]);
    ChrCnt++;
  }

  UNIT_ENABLE_HIGH();		// Disable the output
  UNIT_Deselect_All();
}

/*******************************************************************************
* Function Name  : Time_Delay_1_uSec
* Description    : Approximately 1 uSec per count
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Time_Delay_1_uSec(uint16_t counter)
{
  counter = (counter * 45)/10;
  while (counter)
    counter--;
}

/******************* (C) COPYRIGHT 2012 Neogen Corporation *****END OF FILE****/
