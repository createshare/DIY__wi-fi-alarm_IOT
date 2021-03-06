//*****************************************************************************
// pinmux.c
//
// configure the device pins for different peripheral signals
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

// This file was automatically generated by the CC31xx PinMux Utility
// Version: 1.0.2
//
//*****************************************************************************

#include "pinmux.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include "pin.h"
#include "rom.h"
#include "rom_map.h"
#include "gpio.h"
#include "prcm.h"

/****************************************************************************
*??????-Function:		void Uart_PinMuxConfig(void)
*????- Description:		????????????(????UART ????????????)
*????????-Input:	None
*????????-output:	None
*????????-Note??	??01)    ??02)    ??03)    ??04)  
*****************************************************************************/
void Uart_PinMuxConfig(void)   // ????????????(????UART ????????????)
{
    // Enable Peripheral Clocks 
    //??????????PRCMPeripheralClkEnable????????????????????????????????????
    // ???? UARTA0 ????????????
    MAP_PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);

    //
    // Configure PIN_55 for UART0 UART0_TX
    //??????????PinTypeUART????????????????????????????????????
    //???????? PIN_55 ?? UART0_TX ????????  
    MAP_PinTypeUART(PIN_55, PIN_MODE_3);

    //
    // Configure PIN_57 for UART0 UART0_RX
    //??????????PinTypeUART????????????????????????????????????
    //???????? PIN_57 ?? UART0_RX ????????  
    MAP_PinTypeUART(PIN_57, PIN_MODE_3);
}


/****************************************************************************
*??????-Function:		void Led_PinMuxConfig(void)
*????- Description:		????????????(????LED  GPIO????????????)
*????????-Input:	None
*????????-output:	None
*????????-Note??	??01)    ??02)    ??03)    ??04)  
*****************************************************************************/
void Led_PinMuxConfig(void)  //????????????(????LED  GPIO????????????)
{
    //??????????PRCMPeripheralClkEnable????????????????????????????????????
    //????GPIOA1???????????? 
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK);

    //??????????PinTypeGPIO????????????????????????????????????
    //????????PIN_64??GPIO ????????  
    MAP_PinTypeGPIO(PIN_64, PIN_MODE_0, false);
    
    //??????????GPIODirModeSet????????????????????????????????????
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x2, GPIO_DIR_MODE_OUT);


    // ????????PIN_01??GPIO ???????? 
    MAP_PinTypeGPIO(PIN_01, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x4, GPIO_DIR_MODE_OUT);


    // ????????PIN_02??GPIO ???????? 
    MAP_PinTypeGPIO(PIN_02, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x8, GPIO_DIR_MODE_OUT);

}



/****************************************************************************
*??????-Function:		void Button_PinMuxConfig(void)
*????- Description:		????????????(????SW2(Pin15--GPIO22)??SW3(Pin4--GPIO13)????????????????)
*????????-Input:	None
*????????-output:	None
*????????-Note??	??01)    ??02)    ??03)    ??04)  
*****************************************************************************/
void Button_PinMuxConfig(void)  //????????????(????SW2(Pin15--GPIO22)??SW3(Pin4--GPIO13)????????????????)
{
	//????GPIOA1??GPIOA2???????????? 
	MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK);
	MAP_PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_RUN_MODE_CLK);

	// ????????PIN_04??GPIO ???????? 
	MAP_PinTypeGPIO(PIN_04, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA1_BASE, 0x20, GPIO_DIR_MODE_IN);

	// ????????PIN_15??GPIO ???????? 
	MAP_PinTypeGPIO(PIN_15, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA2_BASE, 0x40, GPIO_DIR_MODE_IN);
			
}


/****************************************************************************
*??????-Function:		void IIC_PinMuxConfig(void)
*????- Description:		 ????????????(????IIC ????????????)
*????????-Input:	None
*????????-output:	None
*????????-Note??	??01)    ??02)    ??03)    ??04)  
*****************************************************************************/
void IIC_PinMuxConfig(void) // ????????????(????IIC ????????????)
{
	// ???? I2CA0 ????????????
	MAP_PRCMPeripheralClkEnable(PRCM_I2CA0, PRCM_RUN_MODE_CLK);

	// ???????? PIN_01 ?? I2C_SCL ????????  
	MAP_PinTypeI2C(PIN_01, PIN_MODE_1);
	// ???????? PIN_02 ?? I2C_SDA ????????
	MAP_PinTypeI2C(PIN_02, PIN_MODE_1);
}




