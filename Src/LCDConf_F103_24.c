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
#define LCD_MIRROR_X 1
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
void MX_GPIO_Init(void);

#define SWEEPER 1
#ifdef SWEEPER
/* connection: 
8 bit Data:  PortA, 
RD: PC_10,      read strobe
WR: PC_11,		write strobe
RS  DC: PB_1     register or data  data/command
CS: PB_0, chip select
reset: PB_15, 
*/
#define _LE(val) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,(val?GPIO_PIN_SET:GPIO_PIN_RESET))
#define _RD(val) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,(val?GPIO_PIN_SET:GPIO_PIN_RESET))
#define _WR(val) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,(val?GPIO_PIN_SET:GPIO_PIN_RESET))
#define _DC(val) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,(val?GPIO_PIN_SET:GPIO_PIN_RESET))
#define _CS(val) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,(val?GPIO_PIN_SET:GPIO_PIN_RESET))
#define _RESET(val) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,(val?GPIO_PIN_SET:GPIO_PIN_RESET))
#else
#ifdef NUCLEO
//   D0 connects to digital pin 8  PA9	(Notice these are
//   D1 connects to digital pin 9  PC7  NOT in order!)
//   D2 connects to digital pin 2  PA10
//   D3 connects to digital pin 3  PB3
//   D4 connects to digital pin 4  PB5
//   D5 connects to digital pin 5  PB4
//   D6 connects to digital pin 6  PB10
//   D7 connects to digital pin 7  PA8
#define YP A3  PB0 // must be an analog pin, use "An" notation!
#define XM A2  PA4 // must be an analog pin, use "An" notation!
#define YM 9   PC7 // can be a digital pin
#define XP 8   PA8 // can be a digital pin
#define LCD_CS A3  PB0
#define LCD_CD A2  PA4
#define LCD_WR A1  PA1
#define LCD_RD A0  PA0
#define LCD_RESET A4  PC1

#else
/* connection: 
8 bit Data:  PortA, 
RD: PB_11,      read strobe
WR: PB_10,		write strobe
RS  DC: PB_1     register or data  data/command
CS: PB_0, chip select
reset: PB_15, 
*/
#define _RD(val) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,(val?GPIO_PIN_SET:GPIO_PIN_RESET))
#define _WR(val) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,(val?GPIO_PIN_SET:GPIO_PIN_RESET))
#define _DC(val) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,(val?GPIO_PIN_SET:GPIO_PIN_RESET))
#define _CS(val) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,(val?GPIO_PIN_SET:GPIO_PIN_RESET))
#define _RESET(val) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,(val?GPIO_PIN_SET:GPIO_PIN_RESET))
#define _LE(val)
#endif
#endif

/* compatibility macros */
#define wr_cmd8 LcdWriteReg8
#define wr_data8 LcdWriteData8
#define reg_write(cmd,data) {LcdWriteReg16(cmd);LcdWriteData16(data);}

void init_9320(void);
void init_9341(void);

void delay_us_DWT(int uSec)
{
	volatile uint32_t cycles = (SystemCoreClock / 1000000L)*uSec;
	volatile uint32_t start = DWT->CYCCNT;
	do {
	} while (DWT->CYCCNT - start < cycles);
}

/********************************************************************
*
*       LcdWriteReg
*
* Function description:
*   Sets display register
*/

static void LcdWriteReg16(U16 Cmd)
{
	_CS(1); // cancel previous command
	_CS(0);
	_DC(0); // 0=cmd
	_LE(1);
	GPIOA->ODR = Cmd & 0xFF; //write LSB
	_LE(0);
	_WR(0);
	GPIOA->ODR = (Cmd >> 8);     // write MSB
	_WR(1);					// should allow 10ns min settling time
	_DC(1); // 1=data next
}

static void LcdWriteReg8(U8 Cmd) {
	_CS(1); // cancel previous command
	_CS(0);
	_DC(0); // 0=cmd
	_WR(0);
	GPIOA->ODR &= 0xFF00;
	GPIOA->ODR |= Cmd;     // write 8bit
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
static void LcdWriteData16(U16 Data)
{
	_DC(1); // 1=data just to make sure
	_WR(0);
	_LE(1);
	GPIOA->ODR = Data&0xFF;     // write LSB
	_LE(0);
	GPIOA->ODR = (Data>>8);     // write MSB
	_WR(1);					// should allow 10ns min settling time
}
static void LcdWriteData8(U8 Data) {
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
#ifdef SWEEPER
static void LcdWriteDataMultiple16(U16 * pData, int NumItems)
{
	while (NumItems--) {
		LcdWriteData16(*pData++);
	} 
}
#else
static void LcdWriteDataMultiple8(U8 * pData, int NumItems) {
  while (NumItems--) {
	  LcdWriteData8(*pData++);
  } 
}
#endif
/********************************************************************
*
*       LcdReadDataMultiple
*
* Function description:
*   Reads multiple values from a display register.
*/
static void PortA_input()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA4 PA5 PA6 PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 
	                        | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void PortA_output()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA4 PA5 PA6 PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 
	                        | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static U8 LcdReadData8(void){
	U8 Data;
	PortA_input();
	_RD(0);
	Data = GPIOA->IDR;     // fake read is necessary
	_RD(1);	
	_RD(0);
	Data= GPIOA->IDR;     // read 8bit should allow 45ns settling time
	_RD(1);	
	PortA_output();
	return Data;
}

static void LcdReadDataMultiple8(U8 * pData, int NumItems) {
	U8 Data;
	PortA_input();
	_RD(0);
	Data = GPIOA->IDR;     // fake read is necessary
	_RD(1);	
	while (NumItems --) {
		_RD(0);
		*pData++ = GPIOA->IDR;     // read 8bit should allow 45ns settling time
		_RD(1);	
	}
	PortA_output();
}

unsigned int rd_reg_data32(unsigned char reg)
{
	wr_cmd8(reg);
	unsigned int r = 0;
	PortA_input();
   
	U8 rb[4];
	LcdReadDataMultiple8(&rb[0], 4);
	r = rb[0] << 24 | rb[1] << 16 | rb[2] << 8 | rb[3];

	_CS(1); // force CS HIG to interupt the cmd in case was not supported
	_CS(0);
	PortA_output();
	return r;
}


void Board_LCD_Init(void) {
	unsigned int tftID;
	MX_GPIO_Init();
	_RD(1);
	_WR(1);
	_DC(0);
	_CS(0);
	_RESET(1);
#ifdef SWEEPER
	_LE(1);
#endif
	delay_us_DWT(15);  // 10us min
	_RESET(0);  // reset is active
	delay_us_DWT(15); // 10us min
	_RESET(1);          // end reset
	delay_us_DWT(150000);		// 120 ms min		


#if SWEEPER
	init_9320();
#else
	tftID = rd_reg_data32(0xBF);
	tftID = rd_reg_data32(0x0);
	init_9341();
#endif
}


void init_9320(){

//flipped = FLIP_X; // FLIP_NONE, FLIP_X, FLIP_Y, FLIP_X|FLIP_Y
 
reg_write(0x0001, 0x0100); 
reg_write(0x0002, 0x0700); 
reg_write(0x0003, 0x1030); 
reg_write(0x0004, 0x0000); 
reg_write(0x0008, 0x0202);  
reg_write(0x0009, 0x0000);
reg_write(0x000A, 0x0000); 
reg_write(0x000C, 0x0000); 
reg_write(0x000D, 0x0000);
reg_write(0x000F, 0x0000);
//power on sequence
reg_write(0x0010, 0x0000);   
reg_write(0x0011, 0x0007);  
reg_write(0x0012, 0x0000);  
reg_write(0x0013, 0x0000); 
reg_write(0x0007, 0x0001);
	HAL_Delay(200); 

reg_write(0x0010, 0x10C0);   
reg_write(0x0011, 0x0007);
	HAL_Delay(50); 

reg_write(0x0012, 0x0110);
	HAL_Delay(50); 

reg_write(0x0013, 0x0b00);
	HAL_Delay(50); 

reg_write(0x0029, 0x0000); 
reg_write(0x002B, 0x4010); // bit 14???
	HAL_Delay(50); 
//gamma
/*
 reg_write(0x0030,0x0004);
 reg_write(0x0031,0x0307);
 reg_write(0x0032,0x0002);// 0006
 reg_write(0x0035,0x0206);
 reg_write(0x0036,0x0408);
 reg_write(0x0037,0x0507); 
 reg_write(0x0038,0x0204);//0200
 reg_write(0x0039,0x0707); 
 reg_write(0x003C,0x0405);// 0504
 reg_write(0x003D,0x0F02);
 */
 //ram
reg_write(0x0050, 0x0000); 
reg_write(0x0051, 0x00EF);
reg_write(0x0052, 0x0000); 
reg_write(0x0053, 0x013F);  
reg_write(0x0060, 0x2700); 
reg_write(0x0061, 0x0001); 
reg_write(0x006A, 0x0000); 
//
reg_write(0x0080, 0x0000); 
reg_write(0x0081, 0x0000); 
reg_write(0x0082, 0x0000); 
reg_write(0x0083, 0x0000); 
reg_write(0x0084, 0x0000); 
reg_write(0x0085, 0x0000); 
//
reg_write(0x0090, 0x0000); 
reg_write(0x0092, 0x0000); 
reg_write(0x0093, 0x0001); 
reg_write(0x0095, 0x0110); 
reg_write(0x0097, 0x0000); 
reg_write(0x0098, 0x0000);
 
reg_write(0x0007, 0x0133); // display on
}

void init_9341(void)
{


	/* Start Initial Sequence ----------------------------------------------------*/
    
	wr_cmd8(0xCB);  // POWER_ON_SEQ_CONTROL             
	wr_data8(0x39);
	wr_data8(0x2C);
	wr_data8(0x00);
	wr_data8(0x34);
	wr_data8(0x02);
     
	wr_cmd8(0xCF);  // POWER_CONTROL_B              
	wr_data8(0x00);
	wr_data8(0xC1);  // Applic Notes 81, was 83, C1 enables PCEQ: PC and EQ operation for power saving
	wr_data8(0x30);
     
	wr_cmd8(0xE8);  // DRIVER_TIMING_CONTROL_A               
	wr_data8(0x85);
	wr_data8(0x00);  // AN 10, was 01
	wr_data8(0x78);  // AN 7A, was 79
     
	wr_cmd8(0xEA);  // DRIVER_TIMING_CONTROL_B                    
	wr_data8(0x00);
	wr_data8(0x00);
     
	wr_cmd8(0xED);                     
	wr_data8(0x64);
	wr_data8(0x03);
	wr_data8(0x12);
	wr_data8(0x81);
     
	wr_cmd8(0xF7);  // PUMP_RATIO_CONTROL                   
	wr_data8(0x20);
     
	wr_cmd8(0xC0);                     // POWER_CONTROL_1
	wr_data8(0x23);  // AN 21, was 26
     
	wr_cmd8(0xC1);                     // POWER_CONTROL_2
	wr_data8(0x10);  // AN 11, was 11
     
	wr_cmd8(0xC5);                     // VCOM_CONTROL_1
	wr_data8(0x3E);  // AN 3F, was 35
	wr_data8(0x28);  // AN 3C, was 3E
     
	wr_cmd8(0xC7);                     // VCOM_CONTROL_2
	wr_data8(0x86);  // AN A7, was BE
     
     
     
	wr_cmd8(0xB1);                     // Frame Rate
	wr_data8(0x00);
	wr_data8(0x18);  // AN 1B, was 1B  1B=70hz             
     
	wr_cmd8(0xB6);                       // display function control, INTERESTING
	wr_data8(0x08);  // AN 0A, was 0A
	wr_data8(0x82);  // AN A2
	wr_data8(0x27);  // AN not present
	//   wr_data8(0x00);  // was present
     
	wr_cmd8(0xF2);                     // Gamma Function Disable
	wr_data8(0x00);  // AN 00, was 08
     
	wr_cmd8(0x26);                     
	wr_data8(0x01);                 // gamma set for curve 01/2/04/08
     
	wr_cmd8(0xE0);                     // positive gamma correction
	wr_data8(0x0F); 
	wr_data8(0x31); 
	wr_data8(0x2B); 
	wr_data8(0x0C); 
	wr_data8(0x0E); 
	wr_data8(0x08); 
	wr_data8(0x4E); 
	wr_data8(0xF1); 
	wr_data8(0x37); 
	wr_data8(0x07); 
	wr_data8(0x10); 
	wr_data8(0x03); 
	wr_data8(0x0E);
	wr_data8(0x09); 
	wr_data8(0x00);
     
	wr_cmd8(0xE1);                     // negativ gamma correction
	wr_data8(0x00); 
	wr_data8(0x0E); 
	wr_data8(0x14); 
	wr_data8(0x03); 
	wr_data8(0x11); 
	wr_data8(0x07); 
	wr_data8(0x31); 
	wr_data8(0xC1); 
	wr_data8(0x48); 
	wr_data8(0x08); 
	wr_data8(0x0F); 
	wr_data8(0x0C); 
	wr_data8(0x31);
	wr_data8(0x36); 
	wr_data8(0x0F);
     
	//wr_cmd8(0x34);                     // tearing effect off
     
	//wr_cmd8(0x35);                     // tearing effect on
      
	//   wr_cmd8(0xB7);                       // ENTRY_MODE_SET
	//   wr_data8(0x07);
  
	wr_cmd8(0x36);      // MEMORY_ACCESS_CONTROL (orientation stuff)
	wr_data8(0x48);
     
	wr_cmd8(0x3A);      // COLMOD_PIXEL_FORMAT_SET
	wr_data8(0x55);     // 16 bit pixel 

	wr_cmd8(0x13); // Normal Displaymode
    
	wr_cmd8(0x11);                     // sleep out
	HAL_Delay(150); 
     
	wr_cmd8(0x29);                     // display on
	HAL_Delay(150); 
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
	Config.Orientation = GUI_SWAP_XY;
  GUIDRV_FlexColor_Config(pDevice, &Config);
  //
  // Set controller and operation mode
  //
#ifdef SWEEPER
	PortAPI.pfWrite16_A0  = LcdWriteReg16;
	PortAPI.pfWrite16_A1  = LcdWriteData16;
	PortAPI.pfWriteM16_A1 = LcdWriteDataMultiple16;
	PortAPI.pfRead8_A1 = LcdReadData8;
	PortAPI.pfReadM8_A1  = LcdReadDataMultiple8;
#else
  PortAPI.pfWrite8_A0  = LcdWriteReg8;
  PortAPI.pfWrite8_A1  = LcdWriteData8;
  PortAPI.pfWriteM8_A1 = LcdWriteDataMultiple8;
  PortAPI.pfRead8_A1 = LcdReadData8;
  PortAPI.pfReadM8_A1  = LcdReadDataMultiple8;
#endif
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

void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	  /* GPIO Ports Clock Enable */
	__GPIOC_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();

	  /*Configure GPIO pins : PAPin PAPin */
	GPIO_InitStruct.Pin = (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

#ifdef SWEEPER
	  /*Configure GPIO pin : PB */
	GPIO_InitStruct.Pin = (GPIO_PIN_14);
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIOB->ODR = 0x1F0; // avoid spikes on control lines
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIOB->ODR = 0x1F0;

	  /*Configure GPIO pins : PC13: LED */
	GPIO_InitStruct.Pin = (GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#else
	  /*Configure GPIO pin : PB */
	GPIO_InitStruct.Pin = (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_15);
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIOB->ODR = 0x1F0; // avoid spikes on control lines
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIOB->ODR = 0x1F0;

	  /*Configure GPIO pins : PC13: LED */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#endif

}

/*************************** End of file ****************************/

