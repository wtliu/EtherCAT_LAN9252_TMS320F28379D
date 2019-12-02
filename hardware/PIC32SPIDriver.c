/*******************************************************************************
 PIC32 SPI Interface Driver

  Company:
    Microchip Technology Inc.

  File Name:
    SPIDriver.c

  Summary:
    Contains the functional implementation of PIC32 SPI Interface Driver

  Description:
    This file contains the functional implementation of PIC32 SPI Interface Driver
	
  Change History:
    Version		Changes
	1.3			Initial version.
*******************************************************************************/

/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
 

#include "PIC32SPIDriver.h"
#include "9252_HW.h"

/*******************************************************************************
  Function:
   void SPIWrite(UINT8 data)
  Summary:
    This function write one byte.
        
  Description:
    This function write one byte.

*****************************************************************************/ 
void SPIWrite(UINT8 data)
{
   //写入数据
   SpiaRegs.SPITXBUF = data << 8;
   //等待中断标志
   while(SpiaRegs.SPISTS.bit.INT_FLAG == 0);
   //读取数据清除中断标志
   SpiaRegs.SPIRXBUF;
}
/*******************************************************************************
  Function:
   void SPIRead(void)
  Summary:
    This function read one byte.
        
  Description:
    This function read one byte.

*****************************************************************************/  
UINT8 SPIRead()
{
   UINT8 rxdata;
   //写入数据
   SpiaRegs.SPITXBUF = 0;
   //等待中断标志
   while(SpiaRegs.SPISTS.bit.INT_FLAG == 0);
   //读取数据清除中断标志
   rxdata = SpiaRegs.SPIRXBUF;
   //返回数据
   return rxdata;
}
/*******************************************************************************
  Function:
   void ESC_initSPIFIFO(void)
  Summary:
    This function init MCU SPI point speed at 50MHz
        
  Description:
    This function init MCU SPI point

*****************************************************************************/
void ESC_initSPIFIFO(void)
{
    EALLOW;
    //复位SPI模块
    SpiaRegs.SPICCR.bit.SPISWRESET=0;
    /*设置SPI为高速模式*/
    SpiaRegs.SPICCR.bit.HS_MODE = 0x1;
    /*设置数据位数为8*/
    SpiaRegs.SPICCR.bit.SPICHAR = 0x7;
    /*时钟相位控制*/
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 1;   // Rising edge
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0;     // Add 1/2-cycle delay of Clk wrt SPISTEA
    /*传输启用*/
    SpiaRegs.SPICTL.bit.TALK = 1;          //
    /*主站模式*/
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;  // Master mode
    /*清除中断标志*/
    SpiaRegs.SPISTS.all=0x0000;        // Clear Status bits (TxBufFull,INT, Overrun)
    //配置低速外设时钟
    ClkCfgRegs.LOSPCP.all = 0x0; // 0:LSPCLK = SYSCLK/1 = 200M; 1: = sysclk/2 = 100M
    //配置SPI模块时钟：
     /*配置时钟分频为4*/
    SpiaRegs.SPIBRR.all=0x003;     // Baud Rate = LSPCLK / (SPIBRR+1) [LSPCLK=SysClk/4 by]
     /*仿真软运行*/
    SpiaRegs.SPIPRI.all=0x0020;    // Stop after transaction complete on EmuStop
    //使能SPI模块
    SpiaRegs.SPICCR.bit.SPISWRESET=1;  // Enable SPI

   EDIS;
}
/*******************************************************************************
  Function:
   void ESC_initSPIAGpio(void)
  Summary:
    This function init MCU SPI GPIO

  Description:
    This function init MCU SPI GPIO

*****************************************************************************/
void ESC_initSPIAGpio(void)
{
    EALLOW;

 /* Enable internal pull-up for the selected pins */
 // Pull-ups can be enabled or disabled by the user.
 // This will enable the pullups for the specified pins.
 // Comment out other unwanted lines.

     GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0;   // Enable pull-up on GPIO16 (SPISIMOA)
 //  GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;    // Enable pull-up on GPIO5 (SPISIMOA)
     GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0;   // Enable pull-up on GPIO17 (SPISOMIA)
 //  GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;    // Enable pull-up on GPIO3 (SPISOMIA)
     GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0;   // Enable pull-up on GPIO18 (SPICLKA)
     //GpioCtrlRegs.GPBPUD.bit.GPIO61 = 0;   // Enable pull-up on GPIO19 (SPISTEA)

 /* Set qualification for selected pins to asynch only */
 // This will select asynch (no qualification) for the selected pins.
 // Comment out other unwanted lines.

     GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = 3; // Asynch input GPIO16 (SPISIMOA)
 //  GpioCtrlRegs.GPAQSEL1.bit.GPIO5 = 3;  // Asynch input GPIO5 (SPISIMOA)
     GpioCtrlRegs.GPBQSEL2.bit.GPIO59 = 3; // Asynch input GPIO17 (SPISOMIA)
 //  GpioCtrlRegs.GPAQSEL1.bit.GPIO3 = 3;  // Asynch input GPIO3 (SPISOMIA)
     GpioCtrlRegs.GPBQSEL2.bit.GPIO60 = 3; // Asynch input GPIO18 (SPICLKA)
     //GpioCtrlRegs.GPBQSEL2.bit.GPIO61 = 3; // Asynch input GPIO19 (SPISTEA)

 /* Configure SPI-A pins using GPIO regs*/
 // This specifies which of the possible GPIO pins will be SPI functional pins.
 // Comment out other unwanted lines.
     GpioCtrlRegs.GPBGMUX2.bit.GPIO58 = 3;
     GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 3; // Configure GPIO58 as SPISIMOA

     GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 3; // Configure GPIO59 as SPISOMIA
     GpioCtrlRegs.GPBGMUX2.bit.GPIO59 = 3;

     GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 3; // Configure GPIO60 as SPICLKA
     GpioCtrlRegs.GPBGMUX2.bit.GPIO60 = 3;

     //GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 3; // Configure GPIO61 as SPISTEA
     //GpioCtrlRegs.GPBGMUX2.bit.GPIO61 = 3;
     EDIS;

     //cs片选信号作为独立引脚
     //GPIO61 set output
     GPIO_SetupPinOptions(61,GPIO_OUTPUT, GPIO_PULLUP);
     GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
}
/*******************************************************************************
  Function:
   void ESC_initHW()
  Summary:
    This function init MCU

  Description:
    This function init MCU

*****************************************************************************/
void ESC_initHW()
{
    // Initialize system clock
    InitSysCtrl();
    // Disable global interrupt
    DINT;
    //  Initialize the PIE control registers to their default state.
    //  The default state is all PIE interrupts disabled and flags
    //  are cleared.
    //  This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();
    // Disable CPU interrupts and clear all CPU interrupt flags:
    EALLOW;
    IER = 0x0000;
    IFR = 0x0000;
    EDIS;
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // GService Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();
    InitGpio(); // Default setup for all pins
    InitCpuTimers();
//------------------------------------------------------------------------------
   //配置SPI
   ESC_initSPIAGpio();
   ESC_initSPIFIFO();

   //配置time1
   //ESC_configureTime1();

   EINT;                             // Enable Global Interrupts
}
