/**
  ******************************************************************************
  * 文件名程: Uart_echo.c
  * 作    者: By Sw Young
  * 版    本: V1.0
  * 功    能:
  * 编写日期: 2018.3.29
  ******************************************************************************
  * 说明：
  * 硬件平台：
  *   MCUc:TM4C123、2相四线步进电机、DRV8825电机驱动、WiFi
  * 软件设计说明：
  *   通过无线精确控制小车的前进、后退距离；左转右转角度。
  * Github：https://github.com/youngsw/Remote_Control_Car_PointRace_3_Car
  ******************************************************************************
**/
//*****************************************************************************
//
// uart_echo.c - Example for reading data from and writing data to the UART in
//               an interrupt driven fashion.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "uart.h"
#include "Timer/Timer.h"
#include "head.h"


//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>UART Echo (uart_echo)</h1>
//!
//! This example application utilizes the UART to echo text.  The first UART
//! (connected to the USB debug virtual serial port on the evaluation board)
//! will be configured in 115,200 baud, 8-n-1 mode.  All characters received on
//! the UART are transmitted back to the UART.
//
//*****************************************************************************

/*---------------------------------------------------------------Beautiful Line---------------------------------------------------------*/
//UART0
uint8_t ReciveData_UART0[16];
uint8_t ReciveData_i_UART0 = 0;


//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UARTIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART0_BASE, ui32Status);

    ReciveData_i_UART0 = 0;
    //
    // Loop while there are characters in the receive FIFO.
    //
    while(UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
//        UARTCharPutNonBlocking(UART0_BASE,
//                                   UARTCharGetNonBlocking(UART0_BASE));

        ReciveData_UART0[ReciveData_i_UART0] = UARTCharGetNonBlocking(UART0_BASE);
        ReciveData_i_UART0++;
        //
        // Blink the LED to show a character transfer is occuring.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        SysCtlDelay(SysCtlClockGet() / (1000 * 3));

        //
        // Turn off the LED
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

    }

 //   UARTSend()
    UART0Send(ReciveData_UART0, ReciveData_i_UART0);
}


//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UART0Send(uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
    }
}



//*****************************************************************************
//
// uart0 init
//
//*****************************************************************************
void Uart0Iint(void)
{
    //
     // Enable the peripherals used by this example.
     //
     SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

     //
     // Enable processor interrupts.
     //
     IntMasterEnable();

     //
     // Set GPIO A0 and A1 as UART pins.
     //
     GPIOPinConfigure(GPIO_PA0_U0RX);
     GPIOPinConfigure(GPIO_PA1_U0TX);
     GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

     //
     // Configure the UART for 115,200, 8-N-1 operation.
     //
     UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                             (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                              UART_CONFIG_PAR_NONE));

     //
     // Enable the UART interrupt.
     //
     IntEnable(INT_UART0);
     UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

     //
     // Prompt for text to be entered.
     //
     UART0Send((uint8_t *)"\n UART0 Is OK!!\n\n ", 17);
}



/*---------------------------------------------------------------Beautiful Line---------------------------------------------------------*/

//UART1
uint8_t ReciveData_UART1[16];
uint8_t ReciveData_i_UART1 = 0;
uint8_t MotorOrderDirection = 5;        //前：0  后：1  左：2  右： 3
uint8_t MotorOrderDisplacement = 0;     //前后表示距离，左右表示转向角
extern uint32_t Counter,CountBan;                 //Counter最大值65535，计数一圈6400故，有效计数为10圈，即200cm
extern uint8_t FlagSend;
extern uint8_t Beep_Flag;
extern uint32_t Beep_Counter;
extern uint32_t Beep_Fre;
extern float parameter_Ang;
extern float parameter_Dis;
extern uint8_t Flag_Stop;
//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UART1IntHandler(void)
{
    uint32_t ui32Status;
    //
    // Get the interrrupt status.
    //
    ui32Status = UARTIntStatus(UART1_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART1_BASE, ui32Status);

    ReciveData_i_UART1 = 0;
    //
    // Loop while there are characters in the receive FIFO.
    //
    while(UARTCharsAvail(UART1_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
//        UARTCharPutNonBlocking(UART1_BASE,
//                                   UARTCharGetNonBlocking(UART1_BASE));

        ReciveData_UART1[ReciveData_i_UART1] = UARTCharGetNonBlocking(UART1_BASE);
        ReciveData_i_UART1++;
        //
        // Blink the LED to show a character transfer is occuring.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        SysCtlDelay(SysCtlClockGet() / (1000 * 3));

        //
        // Turn off the LED
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

    }

 //   UARTSend()
    UART1Send("Received: ",10);
    UART1Send(ReciveData_UART1, ReciveData_i_UART1);
    if(ReciveData_UART1[0]=='F')
    {
        Counter = 0; //计数清零
        TimerEnable(TIMER0_BASE, TIMER_A);
        FlagSend = 1;
        MotorOrderDirection = 0;//前：0  后：1  左：2  右： 3

        Beep_Flag = 1;
        Beep_Counter = 0;
        Beep_Fre = 50;

    }
    else if (ReciveData_UART1[0]=='B')
    {
        Counter = 0; //计数清零
        TimerEnable(TIMER0_BASE, TIMER_A);
        FlagSend = 1;
        MotorOrderDirection = 1;//前：0  后：1  左：2  右： 3

        Beep_Flag = 1;
        Beep_Counter = 0;
        Beep_Fre = 33;

    }
    else if (ReciveData_UART1[0]=='L')
    {
        Counter = 0; //计数清零
        TimerEnable(TIMER0_BASE, TIMER_A);
        FlagSend =1;
        MotorOrderDirection = 2;//前：0  后：1  左：2  右： 3

        Beep_Flag = 1;
        Beep_Counter = 0;
        Beep_Fre = 20;

    }
    else if (ReciveData_UART1[0]=='R')
    {
        Counter = 0; //计数清零
        TimerEnable(TIMER0_BASE, TIMER_A);
        FlagSend = 1;
        MotorOrderDirection = 3;//前：0  后：1  左：2  右： 3

        Beep_Flag = 1;
        Beep_Counter = 0;
        Beep_Fre = 40;

    }
    else if (ReciveData_UART1[0]=='S'&&ReciveData_UART1[1]=='T'&&ReciveData_UART1[2]=='O'&&ReciveData_UART1[3]=='P')
    {
        Flag_Stop = 1;
        FlagSend = 0;
    }
    else if (ReciveData_UART1[0]=='S'&&ReciveData_UART1[1]=='T'&&ReciveData_UART1[2]=='A'&&ReciveData_UART1[3]=='R')
    {
        Flag_Stop = 0;
        FlagSend = 1;
    }
//参数校准
    else if (ReciveData_UART1[0]=='A'&&ReciveData_UART1[1]=='1')
    {
        parameter_Ang=parameter_Ang + 0.05;
        UARTprintf("parameter_Ang=%d",(int)(parameter_Ang*10));
    }
    else if (ReciveData_UART1[0]=='P'&&ReciveData_UART1[1]=='1')
    {
        parameter_Ang=parameter_Ang - 0.05;
        UARTprintf("parameter_Ang=%d",(int)(parameter_Ang*10));
    }
    else if (ReciveData_UART1[0]=='A'&&ReciveData_UART1[1]=='2')
    {
        parameter_Dis=parameter_Dis + 0.5;
        UARTprintf("parameter_Dis%d",(int)(parameter_Dis));
    }
    else if (ReciveData_UART1[0]=='P'&&ReciveData_UART1[1]=='2')
    {
        parameter_Dis=parameter_Dis - 0.5;
        UARTprintf("parameter_Dis%d",(int)(parameter_Dis));
    }
    if(ReciveData_UART1[0]=='F'||ReciveData_UART1[0]=='B'||ReciveData_UART1[0]=='R'||ReciveData_UART1[0]=='L')
        MotorOrderDisplacement = (ReciveData_UART1[1]-48)*100+(ReciveData_UART1[2]-48)*10+(ReciveData_UART1[3]-48);
}


//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UART1Send(uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        UARTCharPutNonBlocking(UART1_BASE, *pui8Buffer++);
    }
}



//*****************************************************************************
//
// uart1 init
//
//*****************************************************************************
void Uart1Iint(void)
{
    //
     // Enable the peripherals used by this example.
     //
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
     SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

     //
     // Enable processor interrupts.
     //
     IntMasterEnable();

     //
     // Set GPIO B0 and B1 as UART pins.
     //
     GPIOPinConfigure(GPIO_PB0_U1RX);
     GPIOPinConfigure(GPIO_PB1_U1TX);
     GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

     //
     // Configure the UART for 115,200, 8-N-1 operation.
     //
     UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
                             (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                              UART_CONFIG_PAR_NONE));
     //
     // Enable the UART interrupt.
     //
     IntEnable(INT_UART1);
     UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

     //
     // Prompt for text to be entered.
     //
     UART1Send((uint8_t *)"\n UART1 Is OK!!\n\n ", 17);
}

