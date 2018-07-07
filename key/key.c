#include "key.h"

uint32_t ReadPin0;
uint32_t ReadPin4;
int KeyPress4=0;

void Key_Configure(void)
{
    //ʹ��GPIO����
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    //����PF0,ֱ�ӶԼĴ������в���
      HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
      HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
      //����GPIO����
      GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0);
      //  GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN);
      GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

}

void Key_Interrupt(void)
{//ע�⣬��ʹ�ô˺���ʱ�����ڳ�ʼ�����������    void Int_Handler_GPIOF(void)   �жϷ������
    //�ж�����
       /****��ʼ���ⲿ�жϲ��������ⲿ�ж�Ϊ�͵�ƽ������ʽ********/
       GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_1);                     //���ⲿ�ж�
       GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_1, GPIO_LOW_LEVEL);//PF4&PF1�½��ش���
       //GPIOIntRegister(GPIO_PORTF_BASE, Int_Handler_GPIOF);
       IntEnable(INT_GPIOF);
       //IntPrioritySet(INT_GPIOF, 0);                                   //�ж����ȼ�
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
      /***********��׼�жϷ������������**********/
        uint32_t ui32IntStatus;
        ui32IntStatus = GPIOIntStatus(GPIO_PORTF_BASE, true);
      GPIOIntClear(GPIO_PORTF_BASE, ui32IntStatus);//����жϱ�־λ
      //ֻ��һ������
     if((ui32IntStatus & GPIO_PIN_4)  == GPIO_PIN_4)//PF4
                 {

                     //���ܣ�����PF4ʱ�л�����
                     //��ʼֵΪ�㣬�������κγ���
                     //=1   ->   �ֶ�ģʽ
                     //=2   ->   �Զ�ģʽ
                     //=3   ->   ���ģʽ
                     //=4   ->   ͣ��
                     //�����л���ѭ�������ظ�
                     //�ô�KeyPress4��ѡ�������
                    if(KeyPress4>=0&&KeyPress4<=4)
                             KeyPress4=(1+KeyPress4);
              /*********EX0 = 0; *�ر��ⲿ�ж�****������main�������ǰ�ظ������ⲿ�ж�**************/
                          GPIOIntDisable(GPIO_PORTF_BASE, GPIO_PIN_4);
                 }
//     if((ui32IntStatus & GPIO_PIN_0)  == GPIO_PIN_0)//PF0
//                     {
//             //�˴���д����Ҫ�ĳ���
//                     }
}
