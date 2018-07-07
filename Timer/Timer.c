/**
  ******************************************************************************
  * �ļ�����: Timer.c
  * ��    ��: By Sw Young
  * ��    ��: V1.0
  * ��    ��:
  * ��д����: 2018.3.29
  ******************************************************************************
  * ˵����
  * Ӳ��ƽ̨��
  *   MCUc:TM4C123��2�����߲��������DRV8825���������WiFi
  * ������˵����
  *   ͨ�����߾�ȷ����С����ǰ�������˾��룻��ת��ת�Ƕȡ�
  * Github��https://github.com/youngsw/Remote_Control_Car_PointRace_3_Car
  ******************************************************************************
**/
#include "timer.h"

extern uint8_t MotorOrderDirection;        //ǰ��0  ��1  ��2  �ң� 3
extern uint8_t MotorOrderDisplacement;     //ǰ���ʾ���룬���ұ�ʾת���
char Time_Flag = 0;
uint32_t Counter = 0;
uint8_t Beep_Flag = 0;
uint32_t Beep_Counter = 0;
uint32_t Beep_Fre = 40;
extern uint8_t Flag_Stop;

/**
  * �� �� ��:MotorContolTimer.c
  * ��������: �����ʱ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��:
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
  * �� �� ��:Timer0IntHandler.c
  * ��������: �����ʱ���ж�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��:
  *   By Sw Young
  *   2018.03.29
  */
void Timer0IntHandler(void)
{
    //�����־λ
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
              GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0);//ִ������������ת��
              GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);//ִ������������ת��
             // GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

          }
          else
          {
              GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);//ִ������������ת��
              GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);//ִ������������ת��

              //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

          }
    }
    IntMasterEnable();

}
/**
  * �� �� ��:Timer1IntHandler.c
  * ��������: ���ڲ������Ͷ�ʱ���ж�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��:
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
