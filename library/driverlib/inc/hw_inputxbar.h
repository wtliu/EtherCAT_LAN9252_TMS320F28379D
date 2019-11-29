//###########################################################################
//
// FILE:    hw_inputxbar.h
//
// TITLE:   Definitions for the XBAR registers.
//
//###########################################################################
// $TI Release: F2837xD Support Library v3.06.00.00 $
// $Release Date: Mon May 27 06:48:24 CDT 2019 $
// $Copyright:
// Copyright (C) 2013-2019 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

#ifndef HW_INPUTXBAR_H
#define HW_INPUTXBAR_H

//*****************************************************************************
//
// The following are defines for the XBAR register offsets
//
//*****************************************************************************
#define XBAR_O_INPUT1SELECT       0x0U         // INPUT1 Input Select Register
                                               // (GPIO0 to x)
#define XBAR_O_INPUT2SELECT       0x1U         // INPUT2 Input Select Register
                                               // (GPIO0 to x)
#define XBAR_O_INPUT3SELECT       0x2U         // INPUT3 Input Select Register
                                               // (GPIO0 to x)
#define XBAR_O_INPUT4SELECT       0x3U         // INPUT4 Input Select Register
                                               // (GPIO0 to x)
#define XBAR_O_INPUT5SELECT       0x4U         // INPUT5 Input Select Register
                                               // (GPIO0 to x)
#define XBAR_O_INPUT6SELECT       0x5U         // INPUT6 Input Select Register
                                               // (GPIO0 to x)
#define XBAR_O_INPUT7SELECT       0x6U         // INPUT7 Input Select Register
                                               // (GPIO0 to x)
#define XBAR_O_INPUT8SELECT       0x7U         // INPUT8 Input Select Register
                                               // (GPIO0 to x)
#define XBAR_O_INPUT9SELECT       0x8U         // INPUT9 Input Select Register
                                               // (GPIO0 to x)
#define XBAR_O_INPUT10SELECT      0x9U         // INPUT10 Input Select Register
                                               // (GPIO0 to x)
#define XBAR_O_INPUT11SELECT      0xAU         // INPUT11 Input Select Register
                                               // (GPIO0 to x)
#define XBAR_O_INPUT12SELECT      0xBU         // INPUT12 Input Select Register
                                               // (GPIO0 to x)
#define XBAR_O_INPUT13SELECT      0xCU         // INPUT13 Input Select Register
                                               // (GPIO0 to x)
#define XBAR_O_INPUT14SELECT      0xDU         // INPUT14 Input Select Register
                                               // (GPIO0 to x)
#define XBAR_O_INPUTSELECTLOCK    0x1EU        // Input Select Lock Register

//*****************************************************************************
//
// The following are defines for the bit fields in the INPUTSELECTLOCK register
//
//*****************************************************************************
#define XBAR_INPUTSELECTLOCK_INPUT1SELECT  0x1U         // Lock bit for INPUT1SEL
                                               // Register
#define XBAR_INPUTSELECTLOCK_INPUT2SELECT  0x2U         // Lock bit for INPUT2SEL
                                               // Register
#define XBAR_INPUTSELECTLOCK_INPUT3SELECT  0x4U         // Lock bit for INPUT3SEL
                                               // Register
#define XBAR_INPUTSELECTLOCK_INPUT4SELECT  0x8U         // Lock bit for INPUT4SEL
                                               // Register
#define XBAR_INPUTSELECTLOCK_INPUT5SELECT  0x10U        // Lock bit for INPUT5SEL
                                               // Register
#define XBAR_INPUTSELECTLOCK_INPUT6SELECT  0x20U        // Lock bit for INPUT6SEL
                                               // Register
#define XBAR_INPUTSELECTLOCK_INPUT7SELECT  0x40U        // Lock bit for INPUT7SEL
                                               // Register
#define XBAR_INPUTSELECTLOCK_INPUT8SELECT  0x80U        // Lock bit for INPUT8SEL
                                               // Register
#define XBAR_INPUTSELECTLOCK_INPUT9SELECT  0x100U       // Lock bit for INPUT9SEL
                                               // Register
#define XBAR_INPUTSELECTLOCK_INPUT10SELECT  0x200U       // Lock bit for INPUT10SEL
                                               // Register
#define XBAR_INPUTSELECTLOCK_INPUT11SELECT  0x400U       // Lock bit for INPUT11SEL
                                               // Register
#define XBAR_INPUTSELECTLOCK_INPUT12SELECT  0x800U       // Lock bit for INPUT12SEL
                                               // Register
#define XBAR_INPUTSELECTLOCK_INPUT13SELECT  0x1000U      // Lock bit for INPUT13SEL
                                               // Register
#define XBAR_INPUTSELECTLOCK_INPUT14SELECT  0x2000U      // Lock bit for INPUT14SEL
                                               // Register
#define XBAR_INPUTSELECTLOCK_INPUT15SELECT  0x4000U      // Lock bit for INPUT15SEL
                                               // Register
#define XBAR_INPUTSELECTLOCK_INPUT16SELECT  0x8000U      // Lock bit for INPUT16SEL
                                               // Register
#endif
