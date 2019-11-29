//###########################################################################
//
// FILE:   pdi_test_appl.c
//
// TITLE:  EtherCAT(ET1200/ET1100) PDI (Processor Data Interface) example for F2837x
//         devices
//###########################################################################״
#include "F28x_Project.h"
int16 read_adder = 0x0000;
Uint32 read_data  = 0x0000;
Uint16 write_data = 0x0000;
unsigned char readbuffer[10];
unsigned char writebuffer[10];
void main()
{
   //ESC初始化
   ESC_initHW();
   while(1)
    {
     //CSR寄存器读写测试
     SPIWriteDWord(read_adder,write_data);
     DELAY_US(1000);
     read_data = SPIReadDWord(read_adder);
     DELAY_US(1000);
     //测试读写LAN9252内存0x1000以内数据
     SPIWriteRegUsingCSR(writebuffer,read_adder,4);
     DELAY_US(1000);
     SPIReadRegUsingCSR(readbuffer,read_adder,4);
     DELAY_US(1000);
     //测试读写LAN9252全部内存地址数据
     SPIWriteRegister(writebuffer,read_adder,10);
     DELAY_US(1000);
     SPIReadDRegister(readbuffer,read_adder,10);
     DELAY_US(1000);
    }
}
//
// End of File
//
