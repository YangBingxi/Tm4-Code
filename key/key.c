#include "key.h"

uint32_t ReadPin0;
uint32_t ReadPin4;
int KeyPress4=0;

void Key_Configure(void)
{
    //使能GPIO外设
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    //解锁PF0,直接对寄存器进行操作
      HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
      HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
      //输入GPIO配置
      GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0);
      //  GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN);
      GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

}

void Key_Interrupt(void)
{//注意，在使用此函数时，请在初始化表里面添加    void Int_Handler_GPIOF(void)   中断服务程序
    //中断配置
       /****初始化外部中断并且设置外部中断为低电平触发方式********/
       GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_1);                     //打开外部中断
       GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_1, GPIO_LOW_LEVEL);//PF4&PF1下降沿触发
       //GPIOIntRegister(GPIO_PORTF_BASE, Int_Handler_GPIOF);
       IntEnable(INT_GPIOF);
       //IntPrioritySet(INT_GPIOF, 0);                                   //中断优先级
       IntMasterEnable();

}

int Key_Scan(int PF)
{      //PF=0  -->PF0
       //PF=4  -->PF4
   int  KeyFlag=0;
    if(PF==0)
    {
        ReadPin0=GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0);
               if((ReadPin0&GPIO_PIN_0)  != GPIO_PIN_0)
                   {
                           delay_ms(100);//delay 100ms
                           ReadPin0=GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0);
                               if((ReadPin0&GPIO_PIN_0)  != GPIO_PIN_0)
                               {
                                   KeyFlag=1;
                                   while(!GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0));
                               }


                   }
    }
    if(PF==4)
    {
        KeyFlag=0;
        ReadPin4=GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4);
               if((ReadPin4&GPIO_PIN_4)  != GPIO_PIN_4)
                   {
                           delay_ms(100);//delay 100ms
                           ReadPin4=GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4);
                               if((ReadPin4&GPIO_PIN_4)  != GPIO_PIN_4)
                               {
                                   if(KeyPress4>=0&&KeyPress4<=4)
                                            KeyPress4=(1+KeyPress4);
                                     KeyFlag=1;
                                   while(!GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4));
                               }

                   }
    }
       return KeyFlag;
}

void Int_Handler_GPIOF(void)
  {
      /***********标准中断服务函数所需操作**********/
        uint32_t ui32IntStatus;
        ui32IntStatus = GPIOIntStatus(GPIO_PORTF_BASE, true);
      GPIOIntClear(GPIO_PORTF_BASE, ui32IntStatus);//清楚中断标志位
      //只用一个按键
     if((ui32IntStatus & GPIO_PIN_4)  == GPIO_PIN_4)//PF4
                 {

                     //功能：按下PF4时切换任务
                     //初始值为零，不运行任何程序
                     //=1   ->   手动模式
                     //=2   ->   自动模式
                     //=3   ->   混合模式
                     //=4   ->   停机
                     //任务切换单循环，不重复
                     //用此KeyPress4来选择呼吸灯
                    if(KeyPress4>=0&&KeyPress4<=4)
                             KeyPress4=(1+KeyPress4);
              /*********EX0 = 0; *关闭外部中断****避免在main程序结束前重复进入外部中断**************/
                          GPIOIntDisable(GPIO_PORTF_BASE, GPIO_PIN_4);
                 }
//     if((ui32IntStatus & GPIO_PIN_0)  == GPIO_PIN_0)//PF0
//                     {
//             //此处填写所需要的程序
//                     }
}
