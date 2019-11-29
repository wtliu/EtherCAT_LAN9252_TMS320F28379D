/*******************************************************************************
  DSP_C28x SPI Interface Driver

  Company:
    Microchip Technology Inc.

  File Name:
    SPIDriver.c

  Summary:
    Contains the functional implementation of C28x SPI Interface Driver

  Description:
    This file contains the functional implementation of DSP F28379D SPI Interface Driver
	
  Change History:
    Version		Changes
	0.1			Initial version.
	0.2			-
	0.3			-	
	0.4 		-
*******************************************************************************/
#include <SPIDriver.h>
/*******************************************************************************
         函数名称：  SPIWrite
         函数描述：  SPI发送单字节数据
         输入参数:
            data  字节数据
         返回值:
********************************************************************************/
void SPIWrite(UINT8 data)
{
  //发送数据左对齐
  SpiaRegs.SPITXBUF = data << 8;
  //延时
  DELAY_US(2);
  //等待发送FIFO发送数据缓存区为零
  while(SpiaRegs.SPIFFTX.bit.TXFFST != 0);
  //复位FIFO
  SpiaRegs.SPIFFTX.bit.TXFIFO=0;
  SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;
  DELAY_US(2);
  SpiaRegs.SPIFFTX.bit.TXFIFO=1;
  SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;
}
/******************************************************************************
          函数名称：  SPIRead
          函数描述：  SPI读取单字节数据
          输入参数:
              返回值: 读取的字节数据
*******************************************************************************/
UINT8 SPIRead()
{
  UINT8 readval;
  //发送无效字节
  SpiaRegs.SPITXBUF = 0;
  //等待接收FIFO接收到数据
  while(SpiaRegs.SPIFFRX.bit.RXFFST < 1);
  //读取接收到的数据
  readval = SpiaRegs.SPIRXBUF;
  //复位FIFO
  DELAY_US(5);
  SpiaRegs.SPIFFTX.bit.TXFIFO = 0;
  SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;
  DELAY_US(2);
  SpiaRegs.SPIFFTX.bit.TXFIFO = 1;
  SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;
  //返回读取的数据
  return readval;
}
/******************************************************************************
          函数名称：  ESC_initSPIFIFO
          函数描述：  初始化SPI接口
          输入参数:
              返回值:
*******************************************************************************/
void ESC_initSPIFIFO(void)
{
    uint16_t m;
    EALLOW;
    // FIFO configuration
    SpiaRegs.SPIFFCT.all=0x0;              // place SPI in reset
    for(m=0;m<3;m++);
    SpiaRegs.SPIFFRX.all=0x2040;           // RX FIFO enabled, clear FIFO int
    SpiaRegs.SPIFFRX.bit.RXFFIL = 16;      // Set RX FIFO level
    SpiaRegs.SPIFFTX.all=0xE040;           // FIFOs enabled, TX FIFO released,

    // SPI configuration
    SpiaRegs.SPIFFTX.bit.TXFFIL = 16;  // Set TX FIFO level
    SpiaRegs.SPICCR.bit.SPICHAR = 0x7;//0xF; // Character Length  = 8
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 1; // Rising edge
    SpiaRegs.SPICTL.bit.SPIINTENA = 1;     // Enabled
    SpiaRegs.SPICTL.bit.TALK = 1;          //
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;  // Master mode
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0;     // Add 1/2-cycle delay of Clk wrt SPISTEA
    SpiaRegs.SPICTL.bit.OVERRUNINTENA = 1; // Overrun Interrupt enabled
    SpiaRegs.SPISTS.all=0x0000;        // Clear Status bits (TxBufFull,INT, Overrun)

    // SpixRegs->SPIBRR.all = 0x63;            // LSPCLK/100
    ClkCfgRegs.LOSPCP.all = 0x2; // 0 = sysclk/1 = 200M; 1 = sysclk/2 = 100M   //ʱ��200M��4���50M
    SpiaRegs.SPIBRR.all=0x004;     // Baud Rate = LSPCLK / (SPIBRR+1) [LSPCLK=SysClk/4 by default=50M]
    SpiaRegs.SPIFFCT.all=0x00;                //50M��5����10M
    SpiaRegs.SPIPRI.all=0x0020;            // Stop after transaction complete on EmuStop

    SpiaRegs.SPIFFTX.bit.TXFFIENA = 0; // Disable TXFF INT
    SpiaRegs.SPIFFRX.bit.RXFFIENA = 0; // disable RXFF INT


    SpiaRegs.SPICCR.bit.SPISWRESET=1;  // Enable SPI

    EDIS;
}
/******************************************************************************
          函数名称：  ESC_initSPIAGpio
          函数描述：  初始化SPI接口GPIO
          输入参数:
              返回值:
                      注: SPIA:GPIO58, GPIO59, GPIO60, GPIO61
*******************************************************************************/
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

    //GPIO61 set output
     GPIO_SetupPinOptions(61,GPIO_OUTPUT, GPIO_PULLUP);
     GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
}
/******************************************************************************
          函数名称：  ESC_initHW()
          函数描述：  初始化硬件接口和MCU
          输入参数:
              返回值:
*******************************************************************************/
void ESC_initHW()
{
    //Init sysclk
    InitSysCtrl();
    //Disable  global  interupt
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

    EALLOW;
    InitGpio(); // Default setup for all pins
    InitCpuTimers();

//------------------------------------------------------------------------------
   //init spi
   ESC_initSPIAGpio();
   ESC_initSPIFIFO();

   EINT;                                // Enable Global Interrupts

}
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
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
void SPISendAddr (UINT16 Address)
{
    UINT16_VAL wAddr;

    wAddr.Val  = Address;
    //Write Address
    SPIWriteByte(wAddr.byte.HB);
    SPIWriteByte(wAddr.byte.LB);
}
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
void SPIReadRegUsingCSR(UINT8 *ReadBuffer, UINT16 Address, UINT8 Count)
{
    UINT32_VAL param32_1 = {0};
    UINT8 i = Count;
    UINT16_VAL wAddr;
    wAddr.Val = Address;
    UINT32_VAL *tmpReadBuffer = (UINT32_VAL *)ReadBuffer;
//    param32_1.v[0] = wAddr.byte.LB;
//    param32_1.v[1] = wAddr.byte.HB;
//    param32_1.v[2] = Count;
//    param32_1.v[3] = ESC_READ_BYTE;
    param32_1.byte.LB = wAddr.byte.LB;
    param32_1.byte.HB = wAddr.byte.HB;
    param32_1.byte.UB = Count;
    param32_1.byte.MB = ESC_READ_BYTE;
    SPIWriteDWord (ESC_CSR_CMD_REG, param32_1.Val);

//    do
//    {
//        param32_1.Val = SPIReadDWord (ESC_CSR_CMD_REG);
//
//    }while(param32_1.v[3] & ESC_CSR_BUSY);

    param32_1.Val = SPIReadDWord (ESC_CSR_DATA_REG);


//    for(i=0;i<Count;i++)
//         ReadBuffer[i] = param32_1.v[i];
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

//    for(i=0;i<Count;i++)
//         param32_1.v[i] = WriteBuffer[i];

    param32_1.w[0]=tmpWriteBuffer->w[0];
    param32_1.w[1]=tmpWriteBuffer->w[1];

    SPIWriteDWord (ESC_CSR_DATA_REG, param32_1.Val);


    wAddr.Val = Address;

//    param32_1.v[0] = wAddr.byte.LB;
//    param32_1.v[1] = wAddr.byte.HB;
//    param32_1.v[2] = Count;
//    param32_1.v[3] = ESC_WRITE_BYTE;
    param32_1.byte.LB = wAddr.byte.LB;
    param32_1.byte.HB = wAddr.byte.HB;
    param32_1.byte.UB = Count;
    param32_1.byte.MB = ESC_WRITE_BYTE;

    SPIWriteDWord (0x304, param32_1.Val);
//    do
//    {
//        param32_1.Val = SPIReadDWord (0x304);
//
//    }while(param32_1.v[3] & ESC_CSR_BUSY);

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

//  /*Reset/Abort any previous commands.*/
//    //param32_1.Val = PRAM_RW_ABORT_MASK;
//  param32_1.dw.w[0] = 0;
//  param32_1.dw.w[1] = 0x4000;
//
//    //SPIWriteDWord (PRAM_READ_CMD_REG, param32_1.Val);
//  SPIWriteDWord (PRAM_READ_CMD_REG, param32_1.dw.Val);

//    /*The host should not modify this field unless the PRAM Read Busy
//    (PRAM_READ_BUSY) bit is a 0.*/
//  do
//    {
//        //param32_1.Val = SPIReadDWord (PRAM_READ_CMD_REG);
//      param32_1.dw.Val = SPIReadDWord (PRAM_READ_CMD_REG);
//
//    //}while((param32_1.v[3] & PRAM_RW_BUSY_8B));
//  }while((param32_1.dw.byte.MB & PRAM_RW_BUSY_8B));

    /*Write Address and Length Register (PRAM_READ_ADDR_LEN) with the
    starting UINT8 address and length) and Set PRAM Read Busy (PRAM_READ_BUSY) bit(-EtherCAT Process RAM Read Command Register)
    to start read operatrion*/
    param32_1.w[0] = Address;
    param32_1.w[1] = Count;
    param32_1.w[2] = 0x0;
    param32_1.w[3] = 0x8000;

    //SPIWriteBytes (PRAM_READ_ADDR_LEN_REG, (UINT8*)&param32_1.Val,8);
    SPIWriteBytes(PRAM_READ_ADDR_LEN_REG, (UINT8*)&param32_1.w,8);

//    /*Read PRAM Read Data Available (PRAM_READ_AVAIL) bit is set*/
//    do
//    {
//        //param32_1.Val = SPIReadDWord (PRAM_READ_CMD_REG);
//      param32_1.dw.Val = SPIReadDWord (PRAM_READ_CMD_REG);
//
//    //}while(!(param32_1.v[0] & IS_PRAM_SPACE_AVBL_MASK));
//  }while(!(param32_1.dw.byte.LB & IS_PRAM_SPACE_AVBL_MASK));
//
//    //nReadSpaceAvblCount = param32_1.v[1] & PRAM_SPACE_AVBL_COUNT_MASK;
//  nReadSpaceAvblCount = param32_1.dw.byte.HB & PRAM_SPACE_AVBL_COUNT_MASK;

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
//  param32_1.dw.w[0] = 0;
//  param32_1.dw.w[1] = 0x4000;
//
//    //SPIWriteDWord (PRAM_WRITE_CMD_REG, param32_1.Val);
//  SPIWriteDWord (PRAM_WRITE_CMD_REG, param32_1.dw.Val);

//    /*Make sure there is no previous write is pending
//    (PRAM Write Busy) bit is a 0 */
//    do
//    {
//        //param32_1.Val = SPIReadDWord (PRAM_WRITE_CMD_REG);
//      param32_1.dw.Val = SPIReadDWord (PRAM_WRITE_CMD_REG);
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
//  do
//    {
//       //param32_1.Val = SPIReadDWord (PRAM_WRITE_CMD_REG);
//     param32_1.dw.Val = SPIReadDWord (PRAM_WRITE_CMD_REG);
//
//    //}while(!(param32_1.v[0] & IS_PRAM_SPACE_AVBL_MASK));
//  }while(!(param32_1.dw.byte.LB & IS_PRAM_SPACE_AVBL_MASK));
//
//    /*Check write data available count*/
//    //nWrtSpcAvlCount = param32_1.v[1] & PRAM_SPACE_AVBL_COUNT_MASK;
//  nWrtSpcAvlCount = param32_1.dw.byte.HB & PRAM_SPACE_AVBL_COUNT_MASK;

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
void SPIReadDRegister(UINT8 *ReadBuffer, UINT16 Address, UINT16 Count)
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
void SPIWriteRegister( UINT8 *WriteBuffer, UINT16 Address, UINT16 Count)
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
/*******************************************************************************
  Function:
    void PDI_Init()
  Summary:
    This function initialize the PDI(SPI).

*****************************************************************************/
//void PDI_Init()
//{
//    SPIOpen();
//}
