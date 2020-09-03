/*******************************************************************************
 PIC32 SPI Interface Driver

  Company:
    Microchip Technology Inc.

  File Name:
    SPIDriver.h

  Summary:
    Contains the Header File of PIC32 SPI Interface Driver

  Description:
    This file contains the Header File of PIC32 SPI Interface Driver
	
  Change History:
    Version		Changes
	1.3			Initial version.

*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
 Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

 Microchip licenses to you the right to use, modify, copy and distribute
 Software only when embedded on a Microchip microcontroller or digital signal
 controller that is integrated into your product or third party product
 (pursuant to the sublicense terms in the accompanying license agreement).

 You should refer to the license agreement accompanying this Software for
 additional information regarding your rights and obligations.

 SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
 MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
 IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
 CONTRACT, NEGLiPMPCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
 OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
 CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
 SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

//#include <plib.h>
#include "../EtherCAT/ethercat/ecat_def.h"
#include "../hardware/SPIDriver.h"

#ifndef PIC32SPIDRIVER_H
#define	PIC32SPIDRIVER_H

#ifdef	__cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------------------------------------
------
------    SPI defines/macros
------
-----------------------------------------------------------------------------------------*/
#define    SPI1_SEL_EN                        GpioDataRegs.GPBCLEAR.bit.GPIO61
#define    SPI1_SEL_DE                        GpioDataRegs.GPBSET.bit.GPIO61
#define    SELECT_SPI                         {(SPI1_SEL_EN) = 1;}
#define    DESELECT_SPI                       {(SPI1_SEL_DE) = 1;}


// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Data Types
// *****************************************************************************
// *****************************************************************************
	
#define CSLOW()      SELECT_SPI
#define CSHIGH()     DESELECT_SPI


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Functions
// *****************************************************************************
// *****************************************************************************

    /*SPI基本的读写字节函数*/
	extern UINT8 SPIRead();
	extern void SPIWrite(UINT8 data);
   /*MCU初始化:
    * F28379D初始化
    * SPI初始化
    * time0初始化
    * SYNC0,SYNC1,SPI_IRQ初始化
    *
    */
    extern void ESC_initHW();

#ifdef	__cplusplus
}
#endif

#endif	/* PMPDRIVER_H */

