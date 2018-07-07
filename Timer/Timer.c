/**
  ******************************************************************************
  * 文件名程: Timer.c
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
#include "timer.h"

extern uint8_t MotorOrderDirection;        //前：0  后：1  左：2  右： 3
extern uint8_t MotorOrderDisplacement;     //前后表示距离，左右表示转向角
char Time_Flag = 0;
uint32_t Counter = 0;
uint8_t Beep_Flag = 0;
uint32_t Beep_Counter = 0;
uint32_t Beep_Fre = 40;
extern uint8_t Flag_Stop;

/**
  * 函 数 名:MotorContolTimer.c
  * 函数功能: 电机定时器
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明:
  *   By Sw Young
  *   2018.03.29
  */
void MotorContolTimer(void)
{
    //
       // Enable the peripherals used by this example.
       //
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
       //
       // Enable processor interrupts.
       //
        IntMasterEnable();

       //
       // Configure the two 32-bit periodic timers.
       //
        TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
        TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);

        TimerLoadSet(TIMER0_BASE, TIMER_A,  SysCtlClockGet()/20000-1);  //20KHz
        TimerLoadSet(TIMER1_BASE, TIMER_A,  SysCtlClockGet() ); //5HZ

       //
       // Setup the interrupts for the timer timeouts.
       //
        IntEnable(INT_TIMER0A);
        IntEnable(INT_TIMER1A);

        TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
       //
       // Enable the timers.
       //
        TimerEnable(TIMER0_BASE, TIMER_A);
        TimerEnable(TIMER1_BASE, TIMER_A);
}
/**
  * 函 数 名:Timer0IntHandler.c
  * 函数功能: 电机定时器中断
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明:
  *   By Sw Young
  *   2018.03.29
  */
void Timer0IntHandler(void)
{
    //清除标志位
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    IntMasterDisable();
    if(Flag_Stop==0)
    {
        if(Beep_Flag)
          {
              Beep_Counter++;
              if(Beep_Counter<Beep_Fre)
                  GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6);
              if(Beep_Counter>Beep_Fre&&Beep_Counter<2*Beep_Fre)
              {
                  GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0);
              }
              if(Beep_Counter>2*Beep_Fre)
                  Beep_Counter = 0;
          }
          Time_Flag++;
          if(Time_Flag>1)
              Time_Flag = 0;
          if(Time_Flag>0)
          {
              Counter++;
              if(Counter>65535)
                  Counter=0;
              GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0);//执行脉冲来控制转速
              GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);//执行脉冲来控制转速
             // GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

          }
          else
          {
              GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);//执行脉冲来控制转速
              GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);//执行脉冲来控制转速

              //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

          }
    }
    IntMasterEnable();

}
/**
  * 函 数 名:Timer1IntHandler.c
  * 函数功能: 串口参数发送定时器中断
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明:
  *   By Sw Young
  *   2018.03.29
  */
void Timer1IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Update the interrupt status on the display.
    //
    if(MotorOrderDirection==0||MotorOrderDirection==1)
    {
        UARTprintf("Dis%d",(Counter*20)/6400);
    }
    else if (MotorOrderDirection==2||MotorOrderDirection==3)
    {
        UARTprintf("Ang%d",(int)(Counter/89.3));
    }
}
