/******************** (C) COPYRIGHT 2012 Neogen Corporation ********************
* File Name          : Comm_1_Wire.c
* Author             : D. E. Olin
* Version            : V1.0
* Date               : 09/17/2008
* Description        : 1 Wire Communications Driver
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "Comm_1_Wire.h"
#include "spi_unit.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#if defined(USE_SENSOR_TIMER_DMA)

#define TIM2_CCR4_Address    0x40000040
#define TIM2_CCR3_Address    0x4000003C

#endif

/* Private macro -------------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t Init_Low_Count, Init_High_Count;
uint16_t Write_1_Low_Count, Write_1_Hi_Count;
uint16_t Write_0_Low_Count, Write_0_Hi_Count;
uint16_t Read_1_Low_Count, Read_1_Delay_Count, Read_1_High_Count;

#if defined(USE_SENSOR_TIMER_DMA)

uint16_t SRC_Buffer[Timing_Buffer_Size];
uint16_t Capture_Buffer[Timing_Buffer_Size];
uint16_t TimerPeriod = 0;

#endif

/* Private function prototypes -----------------------------------------------*/
#if defined(USE_SENSOR_TIMER_DMA)

void Init_Buffers(void);
void T2_Configuration(void);
void IO_Configuration(void);
void DMA_Configuration(void);

#else

void Start_1_Wire(uint16_t counter);
void Slot_Time_Delay(uint16_t counter);

#endif

/* Private functions ---------------------------------------------------------*/

#if defined(USE_SENSOR_TIMER_DMA)

/*******************************************************************************
* Function Name  : Init_1_Wire
* Description    : Uses Timer2, Channel 1 for bit timing, Input is 32MHz
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Init_1_Wire(void)
{
  /* Timer Configuration */
  T2_Configuration();

  /* GPIO Configuration */
  IO_Configuration();

  /* DMA Configuration */
  DMA_Configuration();

  Init_Low_Count = Initialization_Time;
//  Init_High_Count = Init_Sample_Time;
  Write_1_Low_Count = Low_1_Time;
  Write_1_Hi_Count = High_1_Time;
  Write_0_Low_Count = Low_0_Time;
  Write_0_Hi_Count = High_0_Time;
  Read_1_Low_Count = ReadSignal_Time;
  Read_1_Delay_Count = ReadDelay_Time;
  Read_1_High_Count = ReadSlotHi_Time;
  Init_Buffers();
}

/*******************************************************************************
* Function Name  : Init_Buffers
* Description    : Initializes all locations in the buffers to zero
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Init_Buffers(void)
{
  uint8_t n = 0;
  
  while (n < Timing_Buffer_Size)
  {
    SRC_Buffer[n] = Capture_Buffer[n] = 0;
    n++;
  }
}

/*******************************************************************************
* Function Name  : Initialize_1_Wire
* Description    : Sends an initialization signal and waits for a response
* Input          : None
* Output         : Success = Found a Device/Successful Initialization
* Return         : None
*******************************************************************************/
ErrorStatus Initialize_1_Wire(void)
{
uint16_t Value;

//  GPIO_SetBits(Sensor_Port, SENSOR_DATA_OUT);
  SRC_Buffer[0] = Init_Low_Count-1;
  SRC_Buffer[1] = SRC_Buffer[2] = SRC_Buffer[3] = 0;
  Capture_Buffer[0] = Capture_Buffer[1] = 0;

  /* Set the timer reload value */
  TIM_SetAutoreload(TIM2, Initialization_Time);

  /* Read the capture value and clear the DMA request flag */
  TimerPeriod = TIM_GetCapture4(TIM2);
  TIM_ClearFlag(TIM2, (TIM_FLAG_CC3 + TIM_FLAG_CC4 +
                       TIM_FLAG_CC3OF + TIM_FLAG_CC4OF));

  /* Disable the  DMA Channels */
  DMA_Cmd(DMA1_Channel2, DISABLE);
  DMA_Cmd(DMA1_Channel7, DISABLE);

    /* Clear the DMA transfer complete flag */
  DMA_ClearFlag(DMA1_FLAG_TC2 + DMA1_FLAG_TC7);

/* Now Set the DMA Count */
  DMA_SetCurrDataCounter(DMA1_Channel2, 4);
  DMA_SetCurrDataCounter(DMA1_Channel7, 3);

  /* Enable the  DMA Channels */
  DMA_Cmd(DMA1_Channel2, ENABLE);
  DMA_Cmd(DMA1_Channel7, ENABLE);

  /* Start the action */
  TIM_Cmd(TIM2, ENABLE);

  /* Disable SPI2  
  SPI_Cmd(SPI2, DISABLE); */  

  /* Wait to finish the loads */
  while (DMA_GetFlagStatus(DMA1_FLAG_TC2) == RESET);

  /* Stop the action */
  TIM_Cmd(TIM2, DISABLE);

  /* Disable the  DMA Channels 
  DMA_Cmd(DMA1_Channel2, DISABLE);
  DMA_Cmd(DMA1_Channel7, DISABLE);*/

  /* Check the result */

  Value = 3 - DMA_GetCurrDataCounter(DMA1_Channel7);
  if (Value)
    Value--;
  if ((Capture_Buffer[Value] < Init_Max_Rsp_Time) && 
      (Capture_Buffer[Value] > Init_Min_Rsp_Time))
    return SUCCESS;
  else
    return ERROR;

}

/*******************************************************************************
* Function Name  : Write_1_Wire_Byte
* Description    : Writes bytes in the buffer to the 1 wire device
* Input          : bytecount = bytes to be written
* Output         : Data line driven accordingly
* Return         : None
*******************************************************************************/
void Write_1_Wire(uint8_t* buffer, uint8_t bytecount)
{
  uint8_t Value, n, index;
  
  Init_Buffers();
  
  for (n=0; n<bytecount; n++)
  {
    Value = buffer[n];
    index = 0;
    while (index < 8)
    {

      if (Value & 1)
      {
        SRC_Buffer[n*8 + index] = Write_1_Low_Count;
      }
      else
      {
        SRC_Buffer[n*8 + index] = Write_0_Low_Count;
      }

      Value = Value>>1;   // Move to next bit
      index++;
    }// end while
  }// end for

  /* Set the timer reload value */
  TIM_SetAutoreload(TIM2, Time_Slot);

  /* Read the capture value and clear the DMA request flag */
  TimerPeriod = TIM_GetCapture4(TIM2);
  TIM_ClearFlag(TIM2, (TIM_FLAG_CC3 + TIM_FLAG_CC4 +
                       TIM_FLAG_CC3OF + TIM_FLAG_CC4OF));

  /* Disable the  DMA Channels */
  DMA_Cmd(DMA1_Channel2, DISABLE);
  DMA_Cmd(DMA1_Channel7, DISABLE);

  /* Clear the DMA transfer complete flag */
  DMA_ClearFlag(DMA1_FLAG_TC2 + DMA1_FLAG_TC7);

  /* Now Set the DMA Count */
  DMA_SetCurrDataCounter(DMA1_Channel2, bytecount*8 + 2);
  DMA_SetCurrDataCounter(DMA1_Channel7, bytecount*8 + 1);

  /* Enable the  DMA Channels */
  DMA_Cmd(DMA1_Channel2, ENABLE);
  DMA_Cmd(DMA1_Channel7, ENABLE);

  /* Start the action */
  TIM_Cmd(TIM2, ENABLE);

  /* Wait to finish the loads */
  while (DMA_GetFlagStatus(DMA1_FLAG_TC2) == RESET);

  /* Stop the action */
  TIM_Cmd(TIM2, DISABLE);

  /* Disable the  DMA Channels 
  DMA_Cmd(DMA1_Channel2, DISABLE);
  DMA_Cmd(DMA1_Channel7, DISABLE);*/

}

/*******************************************************************************
* Function Name  : Read_1_Wire_Byte
* Description    : Reads a byte from the 1 wire device
* Input          : None
* Output         : Data line driven accordingly
* Return         : Byte Read
*******************************************************************************/
ErrorStatus Read_1_Wire_Byte(uint8_t* buffer, uint8_t bytecount)
{
  uint8_t Value, n, index;
  uint16_t temp;
  
  Init_Buffers();
  
  for (n=0; n<(bytecount*8); n++)
  {
     SRC_Buffer[n] = Read_1_Low_Count;
  }// end for

  /* Set the timer reload value */
  TIM_SetAutoreload(TIM2, Time_Slot);

  /* Read the capture value and clear the DMA request flag */
  TimerPeriod = TIM_GetCapture4(TIM2);
  TIM_ClearFlag(TIM2, (TIM_FLAG_CC3 + TIM_FLAG_CC4 +
                       TIM_FLAG_CC3OF + TIM_FLAG_CC4OF));

  /* Disable the  DMA Channels */
  DMA_Cmd(DMA1_Channel2, DISABLE);
  DMA_Cmd(DMA1_Channel7, DISABLE);

  /* Clear the DMA transfer complete flag */
  DMA_ClearFlag(DMA1_FLAG_TC2 + DMA1_FLAG_TC7);

  /* Now Set the DMA Count */
  DMA_SetCurrDataCounter(DMA1_Channel2, bytecount*8 + 2);
  DMA_SetCurrDataCounter(DMA1_Channel7, bytecount*8 + 1);

  /* Enable the  DMA Channels */
  DMA_Cmd(DMA1_Channel2, ENABLE);
  DMA_Cmd(DMA1_Channel7, ENABLE);

  /* Start the action */
  TIM_Cmd(TIM2, ENABLE);

  /* Wait to finish the loads */
  while (DMA_GetFlagStatus(DMA1_FLAG_TC2) == RESET);

  /* Stop the action */
  TIM_Cmd(TIM2, DISABLE);

  /* Disable the  DMA Channels 
  DMA_Cmd(DMA1_Channel2, DISABLE);
  DMA_Cmd(DMA1_Channel7, DISABLE);*/

  for (n=0; n<bytecount; n++)
  {
    Value = 0;
    index = 0;
    while (index < 8)
    {
      Value = Value>>1;   // Move to next bit
      
      temp = Capture_Buffer[n*8 + index];

      if ((temp > Read_1_Time_Min) && (temp < Read_1_Time_Max))
      {
        Value |= 0x80;
      }
      else if ((temp < Read_0_Time_Min) || (temp > Read_0_Time_Max))
      {
        return ERROR;
      }

      index++;
    }// end while
    
   buffer[n]  = Value;
  }// end for

  return SUCCESS;
}

/*******************************************************************************
* Function Name  : IO_Configuration
* Description    : Sets Tim2 Channel 3 as open drain and channel 4 as input
* Input          : None
* Output         : None
* Return         : Byte Read
*******************************************************************************/
void IO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  /* GPIOA Configuration: Channel 3 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* GPIOB Remap Timer2 Channel 3 to PB10*/ 
 // GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);

  /* TIM2 channel 4 pin (PA.03) configuration */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name  : DMA_Configuration
* Description    : Configures the DMA.
* Input          : None
* Output         : Data line driven accordingly
* Return         : Byte Read
*******************************************************************************/
void DMA_Configuration(void)
{
  DMA_InitTypeDef DMA_InitStructure;

  /* DMA clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* DMA1 Channel2 Config */
  DMA_DeInit(DMA1_Channel2);

  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)TIM2_CCR3_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)SRC_Buffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 5;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

  DMA_Init(DMA1_Channel2, &DMA_InitStructure);

  /* DMA1 Channel2 enable */
  DMA_Cmd(DMA1_Channel2, ENABLE);

  /* DMA1 Channel7 Config */
  DMA_DeInit(DMA1_Channel7);

  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)TIM2_CCR4_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Capture_Buffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 16;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

  DMA_Init(DMA1_Channel7, &DMA_InitStructure);

}

/**
  * @brief  Configures the TIM2 Timers.
  * @param  None
  * @retval None
  */
  /* TIM2 DMA Transfer example -------------------------------------------------
  TIM2CLK = SystemCoreClock, Prescaler = 72, TIM2 counter clock = 1 MHz
  SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density

  The objective is to configure TIM2 channel 3 to generate the required timing
  while using channel 4 to time the input signal.

  Both channel use DMA to load and acquire the timing values.
  The output timing is in the SRC_Buffer queue and the result of the input 
  capture is stored in Capture_Buffer. 
  -----------------------------------------------------------------------------*/
void T2_Configuration(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;

  /* TIM2 clock enable */
  // Enables or disables the Low Speed APB (APB1) peripheral clock.
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* Compute the value to be set in ARR register clock at 1 uSec */
  TimerPeriod = (SystemCoreClock / 1000000 ) - 1;

  /* TIM2 Peripheral Configuration --------------------------------------------*/
  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = TimerPeriod;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = Time_Slot;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 2;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* Channel 3 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = SRC_Buffer[0];
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OC3Init(TIM2, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
  
  /* TIM2 Update DMA Request enable */
  TIM_DMACmd(TIM2, TIM_DMA_Update, ENABLE);

  /* TIM2 CCR4 configuration: Input Capture mode --------------
     The external signal is connected to TIM2 CH4 pin (PA.03), 
     The Rising edge is used as active edge,
     The TIM2 CCR4 is used to time the input period 
  ------------------------------------------------------------ */

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_ICInit(TIM2, &TIM_ICInitStructure);

  /* TIM2 CC4 Update DMA Request enable */
  TIM_DMACmd(TIM2, TIM_DMA_CC4, ENABLE);

  /* Main Output Enable */
  TIM_CtrlPWMOutputs(TIM2, ENABLE);
}

#else

/*******************************************************************************
* Function Name  : Init_1_Wire
* Description    : Uses Timer2, Channel 1 for bit timing, Input is 32MHz
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Init_1_Wire(void)
{
  GPIO_SetBits(Sensor_Port, SENSOR_DATA_OUT);
//  GPIO_SetBits(Sensor_Port, SENSOR_CLK);
  Init_Low_Count = Initialization_Time;
  Init_High_Count = Init_Sample_Time;
  Write_1_Low_Count = Low_1_Time;
  Write_1_Hi_Count = High_1_Time;
  Write_0_Low_Count = Low_0_Time;
  Write_0_Hi_Count = High_0_Time;
  Read_1_Low_Count = ReadSignal_Time;
  Read_1_Delay_Count = ReadDelay_Time;
  Read_1_High_Count = ReadSlotHi_Time;
}

/*******************************************************************************
* Function Name  : Slot_Time_Delay_2_uSec
* Description    : Uses Timer2, Channel 1 for bit timing, Input is 32MHz
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Slot_Time_Delay(uint16_t counter)
{
  while (counter)
    counter--;
}

/*******************************************************************************
* Function Name  : Start_1_Wire
* Description    : Starts the timer and waits for 1 Update
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Start_1_Wire(uint16_t counter)
{
  GPIO_ResetBits(Sensor_Port, SENSOR_DATA_OUT);
  /* Disable SPI2  */
  SPI_Cmd(SPI2, DISABLE);   

  while (counter)
    counter--;
}

/*******************************************************************************
* Function Name  : Initialize_1_Wire
* Description    : Sends an initialization signal and waits for a response
* Input          : None
* Output         : Success = Found a Device/Successful Initialization
* Return         : None
*******************************************************************************/
ErrorStatus Initialize_1_Wire(void)
{
  uint16_t counter,subcount;
  uint16_t dcounter = 0;        // Bit sample counter

  Start_1_Wire(Init_Low_Count);

  /* Turn off global interrupts*/
  __disable_irq();

  /* Now bring the line high */
  GPIO_SetBits(Sensor_Port, SENSOR_DATA_OUT);

  counter = Init_High_Count;

  while (counter >= 5)
  {
    subcount = 5;
      while(subcount)
        subcount--;
    counter-= 5;
    if (GPIO_ReadInputDataBit(Sensor_Port, SENSOR_DATA_IN) == Bit_RESET) dcounter++; 
  }

  /* Turn ON global interrupts*/
  __enable_irq();

  /* And then delay another 450 uSec */
  counter = Init_High_Count*3;

  while (counter >= 5)
  {
    subcount = 5;
      while(subcount)
        subcount--;
    counter-= 5;
  }

  /* Now make sure the line has returned high */
  while (GPIO_ReadInputDataBit(Sensor_Port, SENSOR_DATA_IN) == Bit_RESET); 

  subcount = 200;
  counter = 60;
  if ((dcounter < subcount) && (dcounter > counter))
    return SUCCESS;
  else
    return ERROR;

}

/*******************************************************************************
* Function Name  : Write_1_Wire_Byte
* Description    : Writes a byte to the 1 wire device
* Input          : Value = byte to be written
* Output         : Data line driven accordingly
* Return         : None
*******************************************************************************/
void Write_1_Wire_Byte(uint8_t Value)
{
  uint8_t index = 8;
  
  while (index)
  {
    /* Turn off global interrupts */
    __disable_irq();

    if (Value & 1)
    {
      Start_1_Wire(Write_1_Low_Count);
      /* Now bring the line high */
      GPIO_SetBits(Sensor_Port, SENSOR_DATA_OUT);
      /* Turn ON global interrupts */
      __enable_irq();
      Slot_Time_Delay(Write_1_Hi_Count);
    }
    else
    {
      Start_1_Wire(Write_0_Low_Count);
     /* Now bring the line high */
     GPIO_SetBits(Sensor_Port, SENSOR_DATA_OUT);
      /* Turn ON global interrupts */
      __enable_irq();
     Slot_Time_Delay(Write_0_Hi_Count);
    }

    Value = Value>>1;   // Move to next bit
    index--;
  }
}

/*******************************************************************************
* Function Name  : Read_1_Wire_Byte
* Description    : Reads a byte from the 1 wire device
* Input          : None
* Output         : Data line driven accordingly
* Return         : Byte Read
*******************************************************************************/
uint8_t Read_1_Wire_Byte(void)
{
  uint8_t index = 8;
  uint8_t data = 0;

  while (index)
  {
    data = data>>1;   // make room for next bit

    /* Turn off global interrupts */
    __disable_irq();

    Start_1_Wire(Read_1_Low_Count);

    /* Now bring the line high */
    GPIO_SetBits(Sensor_Port, SENSOR_DATA_OUT);

    Slot_Time_Delay(Read_1_Delay_Count);
    if (GPIO_ReadInputDataBit(Sensor_Port, SENSOR_DATA_IN) == Bit_SET) data |= 0x80; 
  
    /* Turn ON global interrupts */
    __enable_irq();

    Slot_Time_Delay(Read_1_High_Count);

    index--;
  }
  return data;
}

#endif

/******************* (C) COPYRIGHT 2012 Neogen Corporation *****END OF FILE****/
