/*******************************************************************************
 LAN9252 Hardware Abtraction Layer - Implementation file

  Company:
    Microchip Technology Inc.

  File Name:
    PICHW.c

  Description:
    This file  cContains the functional implementation of PIC32 Hardware Abtraction Layer
	
  Change History:
    Version		Changes
	1.3			Initial version.
*******************************************************************************/

// Included files

#include <ecatappl.h>
#include "9252_HW.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

/*-----------------------------------------------------------------------------------------
------
------    Global Interrupt setting
------
-----------------------------------------------------------------------------------------*/

#define   DISABLE_GLOBAL_INT            DINT
#define   ENABLE_GLOBAL_INT             EINT

#define   DISABLE_AL_EVENT_INT        DISABLE_GLOBAL_INT
#define   ENABLE_AL_EVENT_INT         ENABLE_GLOBAL_INT


/*-----------------------------------------------------------------------------------------
------
------    ESC Interrupt
------
-----------------------------------------------------------------------------------------*/

#define    ESC_SPI_INT_GPIO            55U

#define    ACK_ESC_INT                 {(PieCtrlRegs.PIEACK.all)=PIEACK_GROUP1;}

#define    INIT_ESC_INT                ESC_configureIRQGPIO()
/*-----------------------------------------------------------------------------------------
------
------    SYNC0 Interrupt
------
-----------------------------------------------------------------------------------------*/
/*sync0引脚定义*/
#define    ESC_SYNC0_GPIO                   56U
/*sync0中断标志应答*/
#define    SYNC0_INT_REQ                    (PieCtrlRegs.PIEACK.all)
#define    ACK_SYNC0_INT                    {(SYNC0_INT_REQ)=PIEACK_GROUP12;}
/*sync0引脚配置*/
#define    INIT_SYNC0_INT                   ESC_configureSync0GPIO()
/*sync0中断失能*/
#define    DISABLE_SYNC0_INT                {(PieCtrlRegs.PIEIER12.bit.INTx3)=0;}
/*sync0中断使能*/
#define    ENABLE_SYNC0_INT                 {(PieCtrlRegs.PIEIER12.bit.INTx3)=1;}

/*-----------------------------------------------------------------------------------------
------
------    SYNC1 Interrupt
------
-----------------------------------------------------------------------------------------*/
/*sync1引脚定义*/
#define    ESC_SYNC1_GPIO                   57U
/*sync1中断标志应答*/
#define    SYNC1_INT_REQ                    (PieCtrlRegs.PIEACK.all)
#define    ACK_SYNC1_INT                    {(SYNC0_INT_REQ)=PIEACK_GROUP12;}
/*sync1引脚配置*/
#define    INIT_SYNC1_INT                    ESC_configureSync1GPIO()
/*sync1中断失能*/
#define    DISABLE_SYNC1_INT                {(PieCtrlRegs.PIEIER12.bit.INTx2)=0;}
/*sync1中断使能*/
#define    ENABLE_SYNC1_INT                 {(PieCtrlRegs.PIEIER12.bit.INTx2)=1;}


/*-----------------------------------------------------------------------------------------
------
------    Hardware timer
------
-----------------------------------------------------------------------------------------*/

#define    ECAT_TIMER_ACK_INT            {(PieCtrlRegs.PIEACK.all) = PIEACK_GROUP1;}

#define    ENABLE_ECAT_TIMER_INT         {(PieCtrlRegs.PIEIER1.bit.INTx7) = 1;}
#define    DISABLE_ECAT_TIMER_INT        {(PieCtrlRegs.PIEIER1.bit.INTx7) = 0;}



#define    INIT_ECAT_TIMER                 ESC_configureTime()



#define    STOP_ECAT_TIMER                {DISABLE_ECAT_TIMER_INT;/*disable timer interrupt*/ \
                                       (CpuTimer0Regs.TCR.bit.TSS) = 1; /*disable timer*/}

#define    START_ECAT_TIMER               {ENABLE_ECAT_TIMER_INT; /*enable timer interrupt*/ \
                                       (CpuTimer0Regs.TCR.bit.TSS) = 0; /*enable timer*/}


/*-----------------------------------------------------------------------------------------
------
------    LED defines
------
-----------------------------------------------------------------------------------------*/
// EtherCAT Status LEDs -> StateMachine
#ifdef _2806x
#define LED_ECATGREEN                  GpioDataRegs.GPBDAT.bit.GPIO39
#define LED_ECATRED                    GpioDataRegs.GPBDAT.bit.GPIO34
#endif
#ifdef _2837xD
#define LED_ECATGREEN                  GpioDataRegs.GPBDAT.bit.GPIO34
#define LED_ECATRED                    GpioDataRegs.GPADAT.bit.GPIO31
#endif

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */
/*******************************************************************************
  Function:
   void PDI_Restore_Global_Interrupt()
  Summary:
    Restore enable global interrupt

  Description:
    Restore enable global interrupt

*****************************************************************************/
void PDI_Restore_Global_Interrupt()
{
	ENABLE_GLOBAL_INT;
}

/*******************************************************************************
  Function:
     void PDI_Enable_Global_interrupt()

  Summary:
    Enables the platform to handle interrupts.

  Description:
    This routine enables the core to handle any pending interrupt requests.

  Precondition:

    Need to configure system using interrupt (IRQ, SYNC0 and SYNC1- if they are using)
 *****************************************************************************/
void PDI_Enable_Global_interrupt()
{
    ENABLE_GLOBAL_INT;
}

/*******************************************************************************
  Function:
    void PDI_Disable_Global_Interrupt()

  Summary:
    Disables the platform from handling interrupts.

  Description:
    This routine disables the core from handling any pending interrupt requests.

  Returns:
    The previous state of the interrupt Status.  The PDI_Restore_Global_Interrupt
    function can be used in other routines to restore the system interrupt state.

*****************************************************************************/
void PDI_Disable_Global_Interrupt()
{
	DISABLE_GLOBAL_INT;
}

/*******************************************************************************
  Function:
    UINT16 PDI_GetTimer(void)

  Summary:
    Get the 1ms current timer value
  Description:
    This routine gets the 1ms current timer value.
*****************************************************************************/
UINT32 PDI_GetTimer()
{
	return(CpuTimer0Regs.TIM.all);
}

/*******************************************************************************
  Function:
    void PDI_ClearTimer(void)

  Summary:
    Clear the 1ms current timer value
  Description:
    This routine clears the 1ms current timer value.
*****************************************************************************/
void PDI_ClearTimer()
{
	CpuTimer0Regs.TIM.all = 0;
}

/*******************************************************************************
  Function:
  void PDI_Timer_Interrupt(void)

  Summary:
   This function configure and enable the TIMER interrupt for 1ms

  Description:
  This function configure and enable the TIMER interrupt for 1ms
*****************************************************************************/
void PDI_Timer_Interrupt()
{
    INIT_ECAT_TIMER;
    START_ECAT_TIMER;
}
/*******************************************************************************
  Function:
  void PDI_IRQ_Interrupt(void)

  Summary:
   This function configure and enable the interrupt for IRQ
        
  Description:
  This function configure and enable the interrupt for IRQ
*****************************************************************************/
void PDI_IRQ_Interrupt()
{
    INIT_ESC_INT;
    ENABLE_ESC_INT();
}
/*******************************************************************************
  Function:
    void PDI_Init_SYNC_Interrupts(void)

  Summary:
    The function configure and enable SYNC0 and SYNC1 interrupt.
        
  Description:
	The function configure and enable SYNC0 and SYNC1 interrupt. It is platform dependent. INIT_SYNCx_INT is a macro which will configure the external interrupt and ENABLE_SYNCx_INT macro for enabling interrupt for the particular lines.
  *****************************************************************************/

void PDI_Init_SYNC_Interrupts()
{
    //SYNC0配置
    INIT_SYNC0_INT;
    ENABLE_SYNC0_INT;
    //SYNC1配置
    INIT_SYNC1_INT;
    ENABLE_SYNC1_INT;
}
/*******************************************************************************
  Function:
  void HW_SetLed(UINT8 RunLed,UINT8 ErrLed)

  Summary:
   This function set the Error LED if it is required.
        
  Description:
  LAN9252 does not support error LED. So this feature has to be enabled by PDI SOC if it is needed.

 Parameters:
 RunLed  - It is not used. This will be set by LAN9252.    
 ErrLed      -  1- ON, 0-0FF.
  *****************************************************************************/
void HW_SetLed(UINT8 RunLed,UINT8 ErrLed)
{
  /* Here RunLed is not used. Because on chip supported RUN Led is available*/
	LED_ECATRED   = ~ErrLed;
}


/*******************************************************************************
  Function:
    __interrupt void ESC_applicationLayerISR(void)

  Summary:
    Interrupt service routine for the PDI interrupt from the EtherCAT Slave Controller

  *****************************************************************************/
Uint16 ESC_applicationLayerISRnumber = 0;
__interrupt void ESC_applicationLayerISR(void)
{
   DISABLE_ESC_INT();

   /* reset the interrupt flag */
   ACK_ESC_INT;

   PDI_Isr();

   ESC_applicationLayerISRnumber++;

   ENABLE_ESC_INT();
}


#ifdef DC_SUPPORTED
/*******************************************************************************
  Function:
    __interrupt void ESC_applicationSync0ISR(void)

  Summary:
    Interrupt service routine for the interrupts from SYNC0
  *****************************************************************************/
Uint16 ESC_applicationSync0ISRnumber = 0;
__interrupt void ESC_applicationSync0ISR(void)
{
    
   DISABLE_ESC_INT();
   /* reset the interrupt flag */

   Sync0_Isr();

   ACK_SYNC0_INT;

   ESC_applicationSync0ISRnumber++;

   ENABLE_ESC_INT();
}


/*******************************************************************************
  Function:
    void __ISR(_EXTERNAL_2_VECTOR, ipl4) Sync1Isr(void)


  Summary:

    Interrupt service routine for the interrupts from SYNC1
  *****************************************************************************/
Uint16 ESC_applicationSync1ISRnumber = 0;
__interrupt void ESC_applicationSync1ISR(void)
{

    DISABLE_ESC_INT();

    Sync1_Isr();

    /* reset the interrupt flag */
    ACK_SYNC1_INT;

    ESC_applicationSync1ISRnumber++;

    ENABLE_ESC_INT();
}
#endif //DC_SUPPORTED

/*******************************************************************************
  Function:
    __interrupt void TimerIsr(void)

  Summary:
    Interrupt service routine for the interrupts from TIME0
  *****************************************************************************/
Uint16 TimerIsrnumber = 0;
__interrupt void TimerIsr(void)
{
	ECAT_TIMER_ACK_INT;

	TimerIsrnumber++;

	ECAT_CheckTimer();
}
/*******************************************************************************
  Function:
    __interrupt void Timer1Isr(void)

  Summary:
    Interrupt service routine for the interrupts from TIME1
  *****************************************************************************/
Uint16 Timer1Isrnumber = 0;
__interrupt void Timer1Isr(void)
{
    Timer1Isrnumber++;
}
//-----------------------------------------------------------------------------------
// Function to configure SYNC1 signal
// 用于配置SYNC0信号的功能
//-----------------------------------------------------------------------------------
void ESC_configureSync0GPIO()
{
    //设置SYNC0引脚为输入引脚
    GPIO_SetupPinOptions(ESC_SYNC0_GPIO, GPIO_INPUT, GPIO_PULLUP| GPIO_ASYNC);
    GPIO_SetupPinMux(ESC_SYNC0_GPIO, GPIO_MUX_CPU1, 0);
    //设置引脚触发外部中断
    EALLOW;

    InputXbarRegs.INPUT14SELECT = ESC_SYNC0_GPIO; //input14 is tied to XINT5
    PieVectTable.XINT5_INT = &ESC_applicationSync0ISR;

    XintRegs.XINT5CR.bit.POLARITY = 1;      //下降沿中断
    XintRegs.XINT5CR.bit.ENABLE = 1;

    PieCtrlRegs.PIEIER12.bit.INTx3 = 1;     // 使能 Group 12, INT3 (XINT5)

    IER |= 0x0800; //Enable Group 12 interrupt

    EDIS;

}

//-----------------------------------------------------------------------------------
// Function to configure SYNC1 signal
// 用于配置SYNC1信号的功能
//-----------------------------------------------------------------------------------
void ESC_configureSync1GPIO()
{
    //配置引脚为输入引脚
    GPIO_SetupPinOptions(ESC_SYNC1_GPIO, GPIO_INPUT, GPIO_PULLUP| GPIO_ASYNC);
    GPIO_SetupPinMux(ESC_SYNC1_GPIO, GPIO_MUX_CPU1, 0);

    EALLOW;
    //SYNC1引脚链接到XBAR模块的INPUT13,INPUT13链接CPU中断XINT4
    InputXbarRegs.INPUT13SELECT = ESC_SYNC1_GPIO;
    //PIE中断XINT4中断服务函数
    PieVectTable.XINT4_INT  =  &ESC_applicationSync1ISR;
    //设置外部中XINT4为下降沿触发
    XintRegs.XINT4CR.bit.POLARITY = 1;      // Falling edge interrupt
    XintRegs.XINT4CR.bit.ENABLE = 1;

    PieCtrlRegs.PIEIER12.bit.INTx2 = 1;     // Enable Group 12, INT2 (XINT4)

    IER |= 0x0800;  //Enable Group 12 interrupt

    EDIS;

}

//-----------------------------------------------------------------------------------
// Function to configure IRQ signal
// 用于配置IRQ信号的功能
//-----------------------------------------------------------------------------------
void ESC_configureIRQGPIO()
{
    //配置引脚为输入引脚
    GPIO_SetupPinOptions(ESC_SPI_INT_GPIO, GPIO_INPUT, GPIO_PULLUP| GPIO_ASYNC);
    GPIO_SetupPinMux(ESC_SPI_INT_GPIO, GPIO_MUX_CPU1, 0);

    EALLOW;
    //SPI_INT引脚链接到XBAR模块的INPUT4,INPUT4链接CPU中断XINT1
    InputXbarRegs.INPUT4SELECT = ESC_SPI_INT_GPIO;
    //PIE中断XINT4中断服务函数
    PieVectTable.XINT1_INT  =  &ESC_applicationLayerISR;
    //设置外部中XINT4为下降沿触发
    XintRegs.XINT1CR.bit.POLARITY = 0;      // Falling edge interrupt
    XintRegs.XINT1CR.bit.ENABLE = 1;

    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;     // Enable Group 1, INT4 (XINT1)

    IER |= 0x0001;  //Enable Group 1 interrupt

    EDIS;

}
//-----------------------------------------------------------------------------------
// Function to configure time to 1ms
// 配置定时器产生1ms中断
//-----------------------------------------------------------------------------------
void ESC_configureTime()
{
//配置定时器
#if CPU_FRQ_200MHZ
ConfigCpuTimer(&CpuTimer0, 200, 1000);
#endif
#if CPU_FRQ_150MHZ
ConfigCpuTimer(&CpuTimer0, 150, 1000);
#endif
#if CPU_FRQ_120MHZ
ConfigCpuTimer(&CpuTimer0, 120, 1000);
#endif

EALLOW;

//清空定时器计数器
CpuTimer0Regs.TIM.all = 0;
//设置中断服务函数
PieVectTable.TIMER0_INT  = &TimerIsr;
//开启中断
PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

IER |= 0x0001;  //Enable Group 1 interrupt

EDIS;
}
//-----------------------------------------------------------------------------------
// Function to configure time1 to 1ms
// 配置定时器1产生1ms中断
//-----------------------------------------------------------------------------------
void ESC_configureTime1()
{
//配置定时器
ConfigCpuTimer(&CpuTimer1, 200, 1000);

EALLOW;
//清空定时器计数器
CpuTimer1Regs.TIM.all = 0;
//设置中断服务函数
PieVectTable.TIMER1_INT  = &Timer1Isr;
//开启中断
IER |= 1<<12;  //Enable Group 13 interrupt
//开启定时器
CpuTimer1Regs.TCR.bit.TSS = 0;
EDIS;
}
/* *****************************************************************************
 End of File
 */
