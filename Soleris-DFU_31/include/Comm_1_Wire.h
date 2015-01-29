/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
* File Name          : Comm_1_Wire.h
* Author             : D.E. Olin
* Version            : V1.1
* Date               : 01/26/2012
* Description        : This file contains all the functions prototypes for the
*                      1 Wire Communications driver.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __1_WIRE_COMM
#define __1_WIRE_COMM

/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"

/* Private define ------------------------------------------------------------*/

#if defined(USE_SENSOR_TIMER_DMA)
 #define Time_Slot           100  /* 100 uSeconds */
 #define Low_1_Time          10   /* 10 uSeconds */
 #define High_1_Time         90   /* (100-10) uSeconds */
 #define Low_0_Time          70   /* 70 uSeconds */
 #define High_0_Time         30   /* (100-70) uSeconds */
 #define ReadSignal_Time     6    /* 6 uSeconds */
 #define ReadDelay_Time      8    /*  8 uSeconds */
 #define ReadSlotHi_Time     86   /* (100-6-8)uSeconds */
 #define Initialization_Time 500  /* 500 uSeconds */
 #define Init_Min_Rsp_Time   70   /* 45 uSeconds */
 #define Init_Max_Rsp_Time   300  /* 175 uSeconds */
 #define Read_1_Time_Min     5    /* 5 uSeconds */
 #define Read_1_Time_Max     14   /* 14 uSeconds */
 #define Read_0_Time_Min     18   /* 18 uSeconds */
 #define Read_0_Time_Max     50   /* 50 uSeconds */

 #define Timing_Buffer_Size   82  /* allows 80 bytes max */

#else
 #define Time_Slot           1000   /* 100 uSeconds * 10 */
 #define Low_1_Time          100    /* 10 uSeconds * 10 */
 #define High_1_Time         900   /* (100-10) uSeconds * 10 */
 #define Low_0_Time          700   /* 70 uSeconds * 10 */
 #define High_0_Time         300   /* (100-70) uSeconds * 10 */
 #define ReadSignal_Time     60    /* 6 uSeconds * 10 */
 #define ReadDelay_Time      80    /*  8 uSeconds * 10 */
 #define ReadSlotHi_Time     860   /* (100-6-8)uSeconds * 10 */
 #define Initialization_Time 5000  /* 500 uSeconds * 10 */
 #define Init_Sample_Time    600  /* 150 uSeconds * 4 */
#endif

/* Exported types ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#if defined(USE_SENSOR_TIMER_DMA)

void Init_1_Wire(void);
ErrorStatus Initialize_1_Wire(void);
ErrorStatus Read_1_Wire_Byte(uint8_t* buffer, uint8_t bytecount);
void Write_1_Wire(uint8_t* buffer, uint8_t bytecount);

#else

void Init_1_Wire(void);
ErrorStatus Initialize_1_Wire(void);
uint8_t Read_1_Wire_Byte(void);
void Write_1_Wire_Byte(uint8_t Value);

#endif

#endif /* __1_WIRE_COMM */

/******************* (C) COPYRIGHT 2012 Neogen Corp *****END OF FILE****/
