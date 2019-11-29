/*******************************************************************************
  DSP_C28x SPI Interface Driver

  Company:
    Microchip Technology Inc.

  File Name:
    SPIDriver.h

  Summary:
    Contains the Header File of F28379D SPI Interface Driver

  Description:
    This file contains the Header File of F28379D SPI Interface Driver
	
  Change History:
    Version		Changes
	0.1			Initial version.
	0.2			-
	0.3			-	
	0.4 		-
*******************************************************************************/
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "F28x_Project.h"
#include "F2837xD_spi.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifndef SPIDRIVER_H
#define	SPIDRIVER_H

#ifdef	__cplusplus
extern "C" {
#endif

//数据类型定义
#define UINT8   unsigned char
#define UINT16  uint16_t
#define UINT32  uint32_t

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
    struct UINT16_BYTE   byte;
} UINT16_VAL;
typedef union
{
    unsigned long    Val;
    unsigned short    w[2];
    struct UINT32_BYTE   byte;
} UINT32_VAL;

typedef union
{
    unsigned short   w[4];
    UINT32_VAL   dw;
} UINT64_VAL;

    // *****************************************************************************
	// *****************************************************************************
	// Section: File Scope or Global Data Types
	// *****************************************************************************
	// *****************************************************************************
	#define CMD_SERIAL_READ              0x03
	#define CMD_FAST_READ                0x0B
	#define CMD_DUAL_OP_READ             0x3B
	#define CMD_DUAL_IO_READ             0xBB
	#define CMD_QUAD_OP_READ             0x6B
	#define CMD_QUAD_IO_READ             0xEB
	#define CMD_SERIAL_WRITE             0x02
	#define CMD_DUAL_DATA_WRITE          0x32
	#define CMD_DUAL_ADDR_DATA_WRITE     0xB2
	#define CMD_QUAD_DATA_WRITE          0x62
	#define CMD_QUAD_ADDR_DARA_WRITE     0xE2

	#define CMD_SERIAL_READ_DUMMY            0
	#define CMD_FAST_READ_DUMMY              1
	#define CMD_DUAL_OP_READ_DUMMY           1
	#define CMD_DUAL_IO_READ_DUMMY           2
	#define CMD_QUAD_OP_READ_DUMMY           1
	#define CMD_QUAD_IO_READ_DUMMY           4
	#define CMD_SERIAL_WRITE_DUMMY           0
	#define CMD_DUAL_DATA_WRITE_DUMMY        0
	#define CMD_DUAL_ADDR_DATA_WRITE_DUMMY   0
	#define CMD_QUAD_DATA_WRITE_DUMMY        0
	#define CMD_QUAD_ADDR_DARA_WRITE_DUMMY   0

	#define ESC_CSR_CMD_REG		0x304
	#define ESC_CSR_DATA_REG	0x300
	#define ESC_WRITE_BYTE 		0x80
	#define ESC_READ_BYTE 		0xC0
	#define ESC_CSR_BUSY		0x80


    /*配置SPI时钟速率*/
	#define SPI_CLK_DIV(MHz)      ((SYS_FREQ_MHZ/(2*MHz))-1)
    //SPI器件片选CS信号
	#define CSLOW()               GPIO_WritePin(61,0)
	#define CSHIGH()              GPIO_WritePin(61,1)
    //SPI读写单字节接口
	#define SPIWriteByte(UINT8)   SPIWrite(UINT8)
	#define SPIReadByte()         SPIRead()


    //9252 CSR命令
    #define ECAT_REG_BASE_ADDR              0x0300
    #define CSR_DATA_REG_OFFSET             0x00
    #define CSR_CMD_REG_OFFSET              0x04
    #define PRAM_READ_ADDR_LEN_OFFSET       0x08
    #define PRAM_READ_CMD_OFFSET            0x0c
    #define PRAM_WRITE_ADDR_LEN_OFFSET      0x10
    #define PRAM_WRITE_CMD_OFFSET           0x14

    #define PRAM_SPACE_AVBL_COUNT_MASK      0x1f
    #define IS_PRAM_SPACE_AVBL_MASK         0x01


    #define CSR_DATA_REG                    ECAT_REG_BASE_ADDR+CSR_DATA_REG_OFFSET
    #define CSR_CMD_REG                     ECAT_REG_BASE_ADDR+CSR_CMD_REG_OFFSET
    #define PRAM_READ_ADDR_LEN_REG          ECAT_REG_BASE_ADDR+PRAM_READ_ADDR_LEN_OFFSET
    #define PRAM_READ_CMD_REG               ECAT_REG_BASE_ADDR+PRAM_READ_CMD_OFFSET
    #define PRAM_WRITE_ADDR_LEN_REG         ECAT_REG_BASE_ADDR+PRAM_WRITE_ADDR_LEN_OFFSET
    #define PRAM_WRITE_CMD_REG              ECAT_REG_BASE_ADDR+PRAM_WRITE_CMD_OFFSET

    #define PRAM_READ_FIFO_REG              0x04
    #define PRAM_WRITE_FIFO_REG             0x20

    #define HBI_INDEXED_DATA0_REG           0x04
    #define HBI_INDEXED_DATA1_REG           0x0c
    #define HBI_INDEXED_DATA2_REG           0x14

    #define HBI_INDEXED_INDEX0_REG          0x00
    #define HBI_INDEXED_INDEX1_REG          0x08
    #define HBI_INDEXED_INDEX2_REG          0x10

    #define HBI_INDEXED_PRAM_READ_WRITE_FIFO    0x18

    #define PRAM_RW_ABORT_MASK      (1 << 30)
    #define PRAM_RW_BUSY_32B        (1 << 31)
    #define PRAM_RW_BUSY_8B         (1 << 7)
    #define PRAM_SET_READ           (1 << 6)
    #define PRAM_SET_WRITE          0


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Functions
// *****************************************************************************
// *****************************************************************************

void SPIWritePDRamRegister(UINT8 *WriteBuffer, UINT16 Address, UINT16 Count);
void SPIReadPDRamRegister(UINT8 *ReadBuffer, UINT16 Address, UINT16 Count);
void SPIReadRegUsingCSR(UINT8 *ReadBuffer, UINT16 Address, UINT8 Count);
void SPIWriteRegUsingCSR( UINT8 *WriteBuffer, UINT16 Address, UINT8 Count);
void SPIWriteDWord (UINT16 Address, UINT32 Val);
UINT32 SPIReadDWord (UINT16 Address);
void SPIWriteBurstMode (UINT32 Val);
UINT32 SPIReadBurstMode ();
void SPISendAddr (UINT16 Address);

void SPIWriteBytes(UINT16 Address, UINT8 *Val, UINT8 nLenght);
UINT32 PDIReadLAN9252DirectReg( UINT16 Address);
void PDIWriteLAN9252DirectReg( UINT32 Val, UINT16 Address);
void SPIReadDRegister(UINT8 *ReadBuffer, UINT16 Address, UINT16 Count);
void SPIWriteRegister( UINT8 *WriteBuffer, UINT16 Address, UINT16 Count);
//void PDI_Init();

extern  void ESC_initHW();
#ifdef	__cplusplus
}
#endif

#endif	/* PMPDRIVER_H */

