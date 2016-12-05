/*********************************************************************
*          Portions COPYRIGHT 2016 STMicroelectronics                *
*          Portions SEGGER Microcontroller GmbH & Co. KG             *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2015  SEGGER Microcontroller GmbH & Co. KG       *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.32 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with the following terms:

The  software has  been licensed  to STMicroelectronics International
N.V. a Dutch company with a Swiss branch and its headquarters in Plan-
les-Ouates, Geneva, 39 Chemin du Champ des Filles, Switzerland for the
purposes of creating libraries for ARM Cortex-M-based 32-bit microcon_
troller products commercialized by Licensee only, sublicensed and dis_
tributed under the terms and conditions of the End User License Agree_
ment supplied by STMicroelectronics International N.V.
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
File        : LCDConf.c
Purpose     : Display controller configuration (single layer)
---------------------------END-OF-HEADER------------------------------
*/

/**
  ******************************************************************************
  * @file    LCDConf_stm3210c_eval.c
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    22-September-2016
  * @brief   Driver for STM3210C-EVAL board LCD
  ******************************************************************************
  * @attention
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

#include "GUI.h"
#include "GUIDRV_FlexColor.h"
#include "LCDConf_F103_24.h"
#include "stm32f1xx_hal.h"

/*********************************************************************
*
*       Layer configuration (to be modified)
*
**********************************************************************
*/

//
// Physical display size
//
#define XSIZE_PHYS  240
#define YSIZE_PHYS  320

/*********************************************************************
*
*       Configuration checking
*
**********************************************************************
*/
#ifndef   VXSIZE_PHYS
  #define VXSIZE_PHYS XSIZE_PHYS
#endif
#ifndef   VYSIZE_PHYS
  #define VYSIZE_PHYS YSIZE_PHYS
#endif
#ifndef   XSIZE_PHYS
  #error Physical X size of display is not defined!
#endif
#ifndef   YSIZE_PHYS
  #error Physical Y size of display is not defined!
#endif
#ifndef   GUICC_565
  #error Color conversion not defined!
#endif
#ifndef   GUIDRV_FLEXCOLOR
  #error No display driver defined!
#endif

/*********************************************************************
*
*       Defines, sfrs
*
**********************************************************************
*/
#define START_BYTE         0x70
#define SET_INDEX          0x00
#define READ_STATUS        0x01
#define LCD_WRITE_REG      0x02
#define LCD_READ_REG       0x03

/*********************************************************************
*
*       Local functions
*
**********************************************************************
*/
/* connection: 
8 bit Data:  PortA, 
CS: PB_7, chip select
reset: PB_8, 
RS RD: PB_6,      read strobe
WR: PB_5,		write strobe
RD  DC: PB_4     register or data  data/command
*/
#define _DC(val) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,(val?GPIO_PIN_SET:GPIO_PIN_RESET))
#define _WR(val) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,(val?GPIO_PIN_SET:GPIO_PIN_RESET))
#define _RD(val) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,(val?GPIO_PIN_SET:GPIO_PIN_RESET))
#define _CS(val) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,(val?GPIO_PIN_SET:GPIO_PIN_RESET))
#define _RESET(val) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,(val?GPIO_PIN_SET:GPIO_PIN_RESET))

void delay_us_DWT(int uSec)
{
	volatile uint32_t cycles = (SystemCoreClock / 1000000L)*uSec;
	volatile uint32_t start = DWT->CYCCNT;
	do {
	} while (DWT->CYCCNT - start < cycles);
}

void Board_LCD_Init(void)
{
	_CS(1);
	_RESET(0);  // reset is active
	delay_us_DWT(10);  // 10us min
	_RESET(1);                       // end reset
	delay_us_DWT(120000);				
}
/********************************************************************
*
*       LcdWriteReg
*
* Function description:
*   Sets display register
*/
static void LcdWriteReg(U8 Data) {
	_CS(1); // cancel previous command
	_CS(0);
	_DC(0); // 0=cmd
	_WR(0);
	 GPIOA->ODR = Data;     // write 8bit
	_WR(1);					// should allow 10ns min settling time
	_DC(1); // 1=data next

}

/********************************************************************
*
*       LcdWriteData
*
* Function description:
*   Writes a value to a display register
*/
static void LcdWriteData(U8 Data) {
	_DC(1); // 1=data just to make sure
	_WR(0);
	GPIOA->ODR = Data;     // write 8bit
	_WR(1);					// should allow 10ns min settling time
}

/********************************************************************
*
*       LcdWriteDataMultiple
*
* Function description:
*   Writes multiple values to a display register.
*/
static void LcdWriteDataMultiple(U8 * pData, int NumItems) {
  
  while (NumItems--) {
	  LcdWriteData(*pData++);
  } 
  
}

/********************************************************************
*
*       LcdReadDataMultiple
*
* Function description:
*   Reads multiple values from a display register.
*/
static U8 LcdReadData(void){
	U8 Data;
	_RD(0);
	Data = GPIOA->IDR;     // fake read is necessary
	_RD(1);	
	_RD(0);
	Data= GPIOA->IDR;     // read 8bit should allow 45ns settling time
	_RD(1);	
	return Data;
}
static void LcdReadDataMultiple(U8 * pData, int NumItems) {
	U8 Data;
	_RD(0);
	Data = GPIOA->IDR;     // fake read is necessary
	_RD(1);	
	while (NumItems --) {
		_RD(0);
		*pData++ = GPIOA->IDR;     // read 8bit should allow 45ns settling time
		_RD(1);	
	}
}

/*********************************************************************
*
*       Public functions
*
**********************************************************************
*/
/*********************************************************************
*
*       LCD_X_Config
*
* Function description:
*   Called during the initialization process in order to set up the
*   display driver configuration.
*
*/
void LCD_X_Config(void) {
  GUI_DEVICE * pDevice;
  CONFIG_FLEXCOLOR Config = {0};
  GUI_PORT_API PortAPI = {0};
  //
  // Set display driver and color conversion
  //
  pDevice = GUI_DEVICE_CreateAndLink(GUIDRV_FLEXCOLOR, GUICC_565, 0, 0);
  //
  // Display driver configuration, required for Lin-driver
  //
  LCD_SetSizeEx (0, XSIZE_PHYS , YSIZE_PHYS);
  LCD_SetVSizeEx(0, VXSIZE_PHYS, VYSIZE_PHYS);
  //
  // Orientation
  //
  Config.Orientation = GUI_SWAP_XY | GUI_MIRROR_Y;
  GUIDRV_FlexColor_Config(pDevice, &Config);
  //
  // Set controller and operation mode
  //
  PortAPI.pfWrite8_A0  = LcdWriteReg;
  PortAPI.pfWrite8_A1  = LcdWriteData;
  PortAPI.pfWriteM8_A1 = LcdWriteDataMultiple;
  PortAPI.pfRead8_A1 = LcdReadData;
  PortAPI.pfReadM8_A1  = LcdReadDataMultiple;
	GUIDRV_FlexColor_SetFunc(pDevice, &PortAPI, GUIDRV_FLEXCOLOR_F66709, GUIDRV_FLEXCOLOR_M16C0B8);
}

/*********************************************************************
*
*       LCD_X_DisplayDriver
*
* Function description:
*   This function is called by the display driver for several purposes.
*   To support the according task the routine needs to be adapted to
*   the display controller. Please note that the commands marked with
*   'optional' are not cogently required and should only be adapted if
*   the display controller supports these features.
*
* Parameter:
*   LayerIndex - Index of layer to be configured
*   Cmd        - Please refer to the details in the switch statement below
*   pData      - Pointer to a LCD_X_DATA structure
*
* Return Value:
*   < -1 - Error
*     -1 - Command not handled
*      0 - Ok
*/
int LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void * pData) {
  int r;
  (void) LayerIndex;
  (void) pData;
  
  switch (Cmd) {
  case LCD_X_INITCONTROLLER: {
    Board_LCD_Init();
    return 0;
  }
  default:
    r = -1;
  }
  return r;
}

/*************************** End of file ****************************/

