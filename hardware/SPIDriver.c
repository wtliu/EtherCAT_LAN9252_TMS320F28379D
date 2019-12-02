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
	0.1			Initial version.
	1.3			Re-arranged the function. Moved the other functions to PIC32SPIDriver.C
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

struct UINT16_BYTE {
	unsigned short LB:8;
	unsigned short HB:8;
};

struct UINT32_BYTE {
	unsigned short LB:8;
	unsigned short HB:8;
	unsigned short UB:8;
	unsigned short MB:8;
};

typedef union
{
    unsigned short    Val;
    struct UINT16_BYTE    byte;
}UINT16_VAL;

typedef union
{
    unsigned long    Val;
    unsigned short    w[2];
    struct UINT32_BYTE    byte;
}UINT32_VAL;

typedef union
{
    unsigned short    w[4];
    UINT32_VAL    dw;		
}UINT64_VAL;

/*******************************************************************************
  Function:
	UINT32 SPIReadDWord (UINT16 Address)
  Summary:
    This function reads the LAN9252 CSR registers.        
  
*****************************************************************************/
UINT32 SPIReadDWord (UINT16 Address)
{
    UINT32_VAL dwResult;
    UINT16_VAL wAddr;

    wAddr.Val  = Address;
    //Assert CS line
    CSLOW();
    //Write Command
    SPIWriteByte(CMD_FAST_READ);
    //Write Address
    SPIWriteByte(wAddr.byte.HB);
    SPIWriteByte(wAddr.byte.LB);
    
    //Dummy Byte
    SPIWriteByte(CMD_FAST_READ_DUMMY);
    //Read Bytes
    dwResult.byte.LB = SPIReadByte();
    dwResult.byte.HB = SPIReadByte();
    dwResult.byte.UB = SPIReadByte();
    dwResult.byte.MB = SPIReadByte();
    //De-Assert CS line
    CSHIGH();
   
    return dwResult.Val;
}

/*******************************************************************************
  Function:
	void SPISendAddr (UINT16 Address)
  Summary:
    This function write address to SPI data bus.        
  
*****************************************************************************/
void SPISendAddr (UINT16 Address)
{
    UINT16_VAL wAddr;

    wAddr.Val  = Address;
    //Write Address
    SPIWriteByte(wAddr.byte.HB);
    SPIWriteByte(wAddr.byte.LB);
}

/*******************************************************************************
  Function:
	UINT32 SPIReadBurstMode ()
  Summary:
    This function read 4 bytes continuosly.        
  
*****************************************************************************/
UINT32 SPIReadBurstMode ()
{
    UINT32_VAL dwResult;
    //Read Bytes
    dwResult.byte.LB = SPIReadByte();
    dwResult.byte.HB = SPIReadByte();
    dwResult.byte.UB = SPIReadByte();
    dwResult.byte.MB = SPIReadByte();
    
    return dwResult.Val;
}

/*******************************************************************************
  Function:
	void SPIWriteBurstMode (UINT32 Val)
  Summary:
    This function writes 4 bytes continuosly.        
  
*****************************************************************************/
void SPIWriteBurstMode (UINT32 Val)
{
    UINT32_VAL dwData;
    dwData.Val = Val;
    
    //Write Bytes
    SPIWriteByte(dwData.byte.LB);
    SPIWriteByte(dwData.byte.HB);
    SPIWriteByte(dwData.byte.UB);
    SPIWriteByte(dwData.byte.MB);
}

#define ADDRESS_AUTO_INCREMENT 0x40
/*******************************************************************************
  Function:
	void SPIWriteBytes(UINT16 Address, UINT8 *Val, UINT8 nLenght)
  Summary:
    This function writes the LAN9252 CSR registers.        
  
*****************************************************************************/
void SPIWriteBytes(UINT16 Address, UINT8 *Val, UINT8 nLenght)
{
    //UINT8 *dwData;
	UINT16_VAL *dwData;
    UINT16_VAL wAddr;
	UINT8 i=0;

    wAddr.Val  = Address;
    dwData = (UINT16_VAL *)Val;
    //Assert CS line
    CSLOW();
    //Write Command
    SPIWriteByte(CMD_SERIAL_WRITE);
    //Write Address
    SPIWriteByte(wAddr.byte.HB|ADDRESS_AUTO_INCREMENT);
    SPIWriteByte(wAddr.byte.LB);
    //Write Bytes
    //while(nLenght--)
	while(i++<nLenght)
    {
     //SPIWriteByte(*(dwData++));
	 if(i&0x1==1)SPIWriteByte(dwData->byte.LB);
	 else SPIWriteByte(dwData++->byte.HB);
    }
        
    //De-Assert CS line
    CSHIGH();
}

/*******************************************************************************
  Function:
	void SPIWriteDWord (UINT16 Address, UINT32 Val)
  Summary:
    This function writes the LAN9252 CSR registers.        
  
*****************************************************************************/
void SPIWriteDWord (UINT16 Address, UINT32 Val)
{
    UINT32_VAL dwData;
    UINT16_VAL wAddr;

    wAddr.Val  = Address;
    dwData.Val = Val;
    //Assert CS line
    CSLOW();
    //Write Command
    SPIWriteByte(CMD_SERIAL_WRITE);
    //Write Address
    SPIWriteByte(wAddr.byte.HB);
    SPIWriteByte(wAddr.byte.LB);
    //Write Bytes
    SPIWriteByte(dwData.byte.LB);
    SPIWriteByte(dwData.byte.HB);
    SPIWriteByte(dwData.byte.UB);
    SPIWriteByte(dwData.byte.MB);

    //De-Assert CS line
    CSHIGH();
}

/*******************************************************************************
  Function:
   void SPIReadRegUsingCSR(UINT8 *ReadBuffer, UINT16 Address, UINT8 Count)
  Summary:
    This function reads the EtherCAT core registers using LAN9252 CSR registers.        
  
*****************************************************************************/
void SPIReadRegUsingCSR(UINT8 *ReadBuffer, UINT16 Address, UINT8 Count)
{
    UINT32_VAL param32_1 = {0};
    //UINT8 i = 0;
    UINT8 i = Count;
    UINT16_VAL wAddr;
    wAddr.Val = Address;
	UINT32_VAL *tmpReadBuffer = (UINT32_VAL *)ReadBuffer;

    //param32_1.v[0] = wAddr.byte.LB;
    //param32_1.v[1] = wAddr.byte.HB;
    //param32_1.v[2] = Count;
    //param32_1.v[3] = ESC_READ_BYTE;
	param32_1.byte.LB = wAddr.byte.LB;
    param32_1.byte.HB = wAddr.byte.HB;
    param32_1.byte.UB = Count;
    param32_1.byte.MB = ESC_READ_BYTE;

    SPIWriteDWord (ESC_CSR_CMD_REG, param32_1.Val);

//    do
//    {
//        param32_1.Val = SPIReadDWord (ESC_CSR_CMD_REG);
//
//    //}while(param32_1.v[3] & ESC_CSR_BUSY);
//	}while(param32_1.byte.MB & ESC_CSR_BUSY);

    param32_1.Val = SPIReadDWord (ESC_CSR_DATA_REG);

    
    //for(i=0;i<Count;i++)
    //     ReadBuffer[i] = param32_1.v[i];
	if(i-->0)tmpReadBuffer->byte.LB=param32_1.byte.LB;else return;
	if(i-->0)tmpReadBuffer->byte.HB=param32_1.byte.HB;else return;
	if(i-->0)tmpReadBuffer->byte.UB=param32_1.byte.UB;else return;
	if(i-->0)tmpReadBuffer->byte.MB=param32_1.byte.MB;
   
    return;
}

/*******************************************************************************
  Function:
   void SPIWriteRegUsingCSR( UINT8 *WriteBuffer, UINT16 Address, UINT8 Count)
  Summary:
    This function writes the EtherCAT core registers using LAN9252 CSR registers.        
  
*****************************************************************************/
void SPIWriteRegUsingCSR( UINT8 *WriteBuffer, UINT16 Address, UINT8 Count)
{
    UINT32_VAL param32_1 = {0};
    //UINT8 i = 0;
    UINT16_VAL wAddr;
	UINT32_VAL *tmpWriteBuffer = (UINT32_VAL *)WriteBuffer;

    //for(i=0;i<Count;i++)
    //     param32_1.v[i] = WriteBuffer[i];
	param32_1.w[0]=tmpWriteBuffer->w[0];
	param32_1.w[1]=tmpWriteBuffer->w[1];

    SPIWriteDWord (ESC_CSR_DATA_REG, param32_1.Val);


    wAddr.Val = Address;

    //param32_1.v[0] = wAddr.byte.LB;
    //param32_1.v[1] = wAddr.byte.HB;
    //param32_1.v[2] = Count;
    //param32_1.v[3] = ESC_WRITE_BYTE;
	param32_1.byte.LB = wAddr.byte.LB;
    param32_1.byte.HB = wAddr.byte.HB;
    param32_1.byte.UB = Count;
    param32_1.byte.MB = ESC_WRITE_BYTE;

    SPIWriteDWord (0x304, param32_1.Val);
//    do
//    {
//        param32_1.Val = SPIReadDWord (0x304);
//
//    //}while(param32_1.v[3] & ESC_CSR_BUSY);
//	}while(param32_1.byte.MB & ESC_CSR_BUSY);

    return;
}

/*******************************************************************************
  Function:
   void SPIReadPDRamRegister(UINT8 *ReadBuffer, UINT16 Address, UINT16 Count)
  Summary:
    This function reads the PDRAM using LAN9252 FIFO.        
  
*****************************************************************************/
void SPIReadPDRamRegister(UINT8 *ReadBuffer, UINT16 Address, UINT16 Count)
{
    UINT64_VAL param32_1 = {0};
    UINT8 i = 0,nlength, nBytePosition;
    //UINT8 nReadSpaceAvblCount;
    //UINT16 RefAddr = Address;
	UINT32_VAL *tmpReadBuffer = (UINT32_VAL *)ReadBuffer;
	UINT8 j=0;

//	/*Reset/Abort any previous commands.*/
//    //param32_1.Val = PRAM_RW_ABORT_MASK;
//	param32_1.dw.w[0] = 0;
//	param32_1.dw.w[1] = 0x4000;
//
//    //SPIWriteDWord (PRAM_READ_CMD_REG, param32_1.Val);
//	SPIWriteDWord (PRAM_READ_CMD_REG, param32_1.dw.Val);

//    /*The host should not modify this field unless the PRAM Read Busy
//    (PRAM_READ_BUSY) bit is a 0.*/
//	do
//    {
//        //param32_1.Val = SPIReadDWord (PRAM_READ_CMD_REG);
//		param32_1.dw.Val = SPIReadDWord (PRAM_READ_CMD_REG);
//
//    //}while((param32_1.v[3] & PRAM_RW_BUSY_8B));
//	}while((param32_1.dw.byte.MB & PRAM_RW_BUSY_8B));
    
    /*Write Address and Length Register (PRAM_READ_ADDR_LEN) with the
    starting UINT8 address and length) and Set PRAM Read Busy (PRAM_READ_BUSY) bit(-EtherCAT Process RAM Read Command Register)
    to start read operatrion*/
	param32_1.w[0] = Address;
    param32_1.w[1] = Count;
    param32_1.w[2] = 0x0;
    param32_1.w[3] = 0x8000;
    
	//SPIWriteBytes (PRAM_READ_ADDR_LEN_REG, (UINT8*)&param32_1.Val,8);   
	SPIWriteBytes (PRAM_READ_ADDR_LEN_REG, (UINT8*)&param32_1.w,8);   

//    /*Read PRAM Read Data Available (PRAM_READ_AVAIL) bit is set*/
//    do
//    {
//        //param32_1.Val = SPIReadDWord (PRAM_READ_CMD_REG);
//		param32_1.dw.Val = SPIReadDWord (PRAM_READ_CMD_REG);
//
//    //}while(!(param32_1.v[0] & IS_PRAM_SPACE_AVBL_MASK));
//	}while(!(param32_1.dw.byte.LB & IS_PRAM_SPACE_AVBL_MASK));
//
//    //nReadSpaceAvblCount = param32_1.v[1] & PRAM_SPACE_AVBL_COUNT_MASK;
//	nReadSpaceAvblCount = param32_1.dw.byte.HB & PRAM_SPACE_AVBL_COUNT_MASK;

    /*Fifo registers are aliased address. In indexed it will read indexed data reg 0x04, but it will point to reg 0
     In other modes read 0x04 FIFO register since all registers are aliased*/

    /*get the UINT8 lenth for first read*/
    //Auto increment is supported in SPIO
    //param32_1.Val = SPIReadDWord (PRAM_READ_FIFO_REG);
	param32_1.dw.Val = SPIReadDWord (PRAM_READ_FIFO_REG);
//    nReadSpaceAvblCount--;
    nBytePosition = (Address & 0x03);
    nlength = (4-nBytePosition) > Count ? Count:(4-nBytePosition);
    //memcpy(ReadBuffer+i ,&param32_1.v[nBytePosition],nlength);
	j=nlength;
	switch(nBytePosition)
	{
	case 0:
		if(j-->0)tmpReadBuffer->byte.LB=param32_1.dw.byte.LB;else break;
		if(j-->0)tmpReadBuffer->byte.HB=param32_1.dw.byte.HB;else break;
		if(j-->0)tmpReadBuffer->byte.UB=param32_1.dw.byte.UB;else break;
		if(j-->0)tmpReadBuffer->byte.MB=param32_1.dw.byte.MB;
		break;
	case 1:
		if(j-->0)tmpReadBuffer->byte.LB=param32_1.dw.byte.HB;else break;
		if(j-->0)tmpReadBuffer->byte.HB=param32_1.dw.byte.UB;else break;
		if(j-->0)tmpReadBuffer->byte.UB=param32_1.dw.byte.MB;
		break;
	case 2:
		if(j-->0)tmpReadBuffer->byte.LB=param32_1.dw.byte.UB;else break;
		if(j-->0)tmpReadBuffer->byte.HB=param32_1.dw.byte.MB;
		break;
	case 3:
		tmpReadBuffer->byte.LB=param32_1.dw.byte.MB;
		break;
	default:
		break;
	}
    Count-=nlength;
    i+=nlength;

    //Lets do it in auto increment mode
    CSLOW();

    //Write Command
    SPIWriteByte(CMD_FAST_READ);

    SPISendAddr(PRAM_READ_FIFO_REG);
    
    //Dummy Byte
    SPIWriteByte(CMD_FAST_READ_DUMMY);

    while(Count)
    {
        //param32_1.Val = SPIReadBurstMode ();
		param32_1.dw.Val = SPIReadBurstMode ();

        nlength = Count > 4 ? 4 : Count;
        //memcpy((ReadBuffer+i) ,&param32_1,nlength);
		j=nlength;
		switch(i&0x3)
		{
		case 0:
			tmpReadBuffer+=1;
			if(j-->0)tmpReadBuffer->byte.LB=param32_1.dw.byte.LB;else break;
			if(j-->0)tmpReadBuffer->byte.HB=param32_1.dw.byte.HB;else break;
			if(j-->0)tmpReadBuffer->byte.UB=param32_1.dw.byte.UB;else break;
			if(j-->0)tmpReadBuffer->byte.MB=param32_1.dw.byte.MB;
			break;
		case 1:
			if(j-->0)tmpReadBuffer->byte.HB=param32_1.dw.byte.LB;else break;
			if(j-->0)tmpReadBuffer->byte.UB=param32_1.dw.byte.HB;else break;
			if(j-->0)tmpReadBuffer->byte.MB=param32_1.dw.byte.UB;else break;
			tmpReadBuffer+=1;
			if(j-->0)tmpReadBuffer->byte.LB=param32_1.dw.byte.MB;
			break;
		case 2:
			if(j-->0)tmpReadBuffer->byte.UB=param32_1.dw.byte.LB;else break;
			if(j-->0)tmpReadBuffer->byte.MB=param32_1.dw.byte.HB;else break;
			tmpReadBuffer+=1;
			if(j-->0)tmpReadBuffer->byte.LB=param32_1.dw.byte.UB;else break;
			if(j-->0)tmpReadBuffer->byte.HB=param32_1.dw.byte.MB;
			break;
		case 3:
			if(j-->0)tmpReadBuffer->byte.MB=param32_1.dw.byte.LB;else break;
			tmpReadBuffer+=1;
			if(j-->0)tmpReadBuffer->byte.LB=param32_1.dw.byte.HB;else break;
			if(j-->0)tmpReadBuffer->byte.HB=param32_1.dw.byte.UB;else break;
			if(j-->0)tmpReadBuffer->byte.UB=param32_1.dw.byte.MB;
			break;
		default:
			break;
		}

        i+=nlength;
        Count-=nlength;
//        nReadSpaceAvblCount --;
    }

    CSHIGH();

    return;
}
        
/*******************************************************************************
  Function:
   void SPIWritePDRamRegister(UINT8 *WriteBuffer, UINT16 Address, UINT16 Count)
  Summary:
    This function writes the PDRAM using LAN9252 FIFO.        
  
*****************************************************************************/
void SPIWritePDRamRegister(UINT8 *WriteBuffer, UINT16 Address, UINT16 Count)
{
    UINT64_VAL param32_1 = {0};
    UINT8 i = 0,nlength, nBytePosition;
    //nWrtSpcAvlCount;
	UINT32_VAL *tmpWriteBuffer = (UINT32_VAL *)WriteBuffer;

//    /*Reset or Abort any previous commands.*/
//    //param32_1.Val = PRAM_RW_ABORT_MASK;
//	param32_1.dw.w[0] = 0;
//	param32_1.dw.w[1] = 0x4000;
//
//    //SPIWriteDWord (PRAM_WRITE_CMD_REG, param32_1.Val);
//	SPIWriteDWord (PRAM_WRITE_CMD_REG, param32_1.dw.Val);

//    /*Make sure there is no previous write is pending
//    (PRAM Write Busy) bit is a 0 */
//    do
//    {
//        //param32_1.Val = SPIReadDWord (PRAM_WRITE_CMD_REG);
//		param32_1.dw.Val = SPIReadDWord (PRAM_WRITE_CMD_REG);
//
//    //}while((param32_1.v[3] & PRAM_RW_BUSY_8B));
//    }while((param32_1.dw.byte.MB & PRAM_RW_BUSY_8B));

    /*Write Address and Length Register (ECAT_PRAM_WR_ADDR_LEN) with the
    starting UINT8 address and length) and write to the EtherCAT Process RAM Write Command Register (ECAT_PRAM_WR_CMD) with the  PRAM Write Busy
    (PRAM_WRITE_BUSY) bit set*/
	param32_1.w[0] = Address;
    param32_1.w[1] = Count;
    param32_1.w[2] = 0x0;
    param32_1.w[3] = 0x8000;
    
   //SPIWriteBytes (PRAM_WRITE_ADDR_LEN_REG, (UINT8*)&param32_1.Val,8);
   SPIWriteBytes (PRAM_WRITE_ADDR_LEN_REG, (UINT8*)param32_1.w,8);

//   /*Read PRAM write Data Available (PRAM_READ_AVAIL) bit is set*/
//	do
//    {
//       //param32_1.Val = SPIReadDWord (PRAM_WRITE_CMD_REG);
//	   param32_1.dw.Val = SPIReadDWord (PRAM_WRITE_CMD_REG);
//
//    //}while(!(param32_1.v[0] & IS_PRAM_SPACE_AVBL_MASK));
//	}while(!(param32_1.dw.byte.LB & IS_PRAM_SPACE_AVBL_MASK));
//
//    /*Check write data available count*/
//    //nWrtSpcAvlCount = param32_1.v[1] & PRAM_SPACE_AVBL_COUNT_MASK;
//	nWrtSpcAvlCount = param32_1.dw.byte.HB & PRAM_SPACE_AVBL_COUNT_MASK;

    /*Write data to Write FIFO) */ 
    /*get the byte lenth for first read*/
    nBytePosition = (Address & 0x03);

    nlength = (4-nBytePosition) > Count ? Count:(4-nBytePosition);

    //param32_1.Val = 0;
    param32_1.w[0]=0;
    param32_1.w[1]=0;
    param32_1.w[2]=0;
    param32_1.w[3]=0;
    //memcpy(&param32_1.v[nBytePosition],WriteBuffer+i, nlength);
	switch(nBytePosition)
	{
	case 0:
		param32_1.dw.w[0]=tmpWriteBuffer->w[0];
		param32_1.dw.w[1]=tmpWriteBuffer->w[1];
		break;
	case 1:
		param32_1.dw.byte.HB=tmpWriteBuffer->byte.LB;
		param32_1.dw.byte.UB=tmpWriteBuffer->byte.HB;
		param32_1.dw.byte.MB=tmpWriteBuffer->byte.UB;
		break;
	case 2:
		param32_1.dw.byte.UB=tmpWriteBuffer->byte.LB;
		param32_1.dw.byte.MB=tmpWriteBuffer->byte.HB;
		break;
	case 3:
		param32_1.dw.byte.MB=tmpWriteBuffer->byte.LB;
		break;
	default:
		break;
	}

    //SPIWriteDWord (PRAM_WRITE_FIFO_REG,param32_1.Val);
	SPIWriteDWord (PRAM_WRITE_FIFO_REG,param32_1.dw.Val);

//    nWrtSpcAvlCount--;
    Count-=nlength;
    i+=nlength;

    //Auto increment mode
    CSLOW();

    //Write Command
    SPIWriteByte(CMD_SERIAL_WRITE);

    SPISendAddr(PRAM_WRITE_FIFO_REG);

    while(Count)
    {
        nlength = Count > 4 ? 4 : Count;
        //param32_1.Val = 0;
        param32_1.w[0]=0;
        param32_1.w[1]=0;
        param32_1.w[2]=0;
        param32_1.w[3]=0;
        //memcpy(&param32_1, (WriteBuffer+i), nlength);
		switch(i&0x3)
		{
		case 0:
			tmpWriteBuffer+=1;
			param32_1.dw.w[0]=tmpWriteBuffer->w[0];
			param32_1.dw.w[1]=tmpWriteBuffer->w[1];
			break;
		case 1:
			param32_1.dw.byte.LB=tmpWriteBuffer->byte.HB;
			param32_1.dw.byte.HB=tmpWriteBuffer->byte.UB;
			param32_1.dw.byte.UB=tmpWriteBuffer->byte.MB;
			tmpWriteBuffer+=1;
			param32_1.dw.byte.MB=tmpWriteBuffer->byte.LB;
			break;
		case 2:
			param32_1.dw.byte.LB=tmpWriteBuffer->byte.UB;
			param32_1.dw.byte.HB=tmpWriteBuffer->byte.MB;
			tmpWriteBuffer+=1;
			param32_1.dw.byte.UB=tmpWriteBuffer->byte.LB;
			param32_1.dw.byte.MB=tmpWriteBuffer->byte.HB;
			break;
		case 3:
			param32_1.dw.byte.LB=tmpWriteBuffer->byte.MB;
			tmpWriteBuffer+=1;
			param32_1.dw.byte.HB=tmpWriteBuffer->byte.LB;
			param32_1.dw.byte.UB=tmpWriteBuffer->byte.HB;
			param32_1.dw.byte.MB=tmpWriteBuffer->byte.UB;
			break;
		default:
			break;
		}

        //SPIWriteBurstMode (param32_1.Val);
		SPIWriteBurstMode (param32_1.dw.Val);
        i+=nlength;
        Count-=nlength;
//        nWrtSpcAvlCount--;
    }

    CSHIGH();
    return;
}

/*******************************************************************************
  Function:
   void PDIReadReg(UINT8 *ReadBuffer, UINT16 Address, UINT16 Count)
  Summary:
    This function reads the ESC registers using LAN9252 CSR or FIFO.         
  
*****************************************************************************/
void PDIReadReg(UINT8 *ReadBuffer, UINT16 Address, UINT16 Count)
{
    if (Address >= 0x1000)
    {
         SPIReadPDRamRegister(ReadBuffer, Address,Count);
    }
    else
    {
         SPIReadRegUsingCSR(ReadBuffer, Address,Count);
    }
}
/*******************************************************************************
  Function:
   void PDIWriteReg( UINT8 *WriteBuffer, UINT16 Address, UINT16 Count)
  Summary:
    This function writes the ESC registers using LAN9252 CSR or FIFO.        
  
*****************************************************************************/
void PDIWriteReg( UINT8 *WriteBuffer, UINT16 Address, UINT16 Count)
{
   
   if (Address >= 0x1000)
   {
		SPIWritePDRamRegister(WriteBuffer, Address,Count);
   }
   else
   {
		SPIWriteRegUsingCSR(WriteBuffer, Address,Count);
   }
    
}

/*******************************************************************************
  Function:
	UINT32 PDIReadLAN9252DirectReg( UINT16 Address)
  Summary:
    This function reads the LAN9252 CSR registers(Not ESC registers).        
  
*****************************************************************************/
UINT32 PDIReadLAN9252DirectReg( UINT16 Address)
{   
    UINT32 data;
    data = SPIReadDWord (Address);
    return data;
}

/*******************************************************************************
  Function:
	void PDIWriteLAN9252DirectReg( UINT32 Val, UINT16 Address)
  Summary:
    This function writes the LAN9252 CSR registers(Not ESC registers).        
  
*****************************************************************************/
void PDIWriteLAN9252DirectReg( UINT32 Val, UINT16 Address)
{
    SPIWriteDWord (Address, Val);
}

