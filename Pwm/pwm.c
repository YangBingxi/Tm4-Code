#include "pwm.h"
void BeepPwmInit (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);//ʹ��PWM0ģ��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//ʹ��PWM0��PWM1�������GPIO
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);//����PH0/PH1ΪPWM����
    GPIOPinConfigure(GPIO_PB6_M0PWM0);    //#define GPIO_PB6_M0PWM0         0x00011804
    GPIOPinConfigure(GPIO_PB7_M0PWM1);    //#define GPIO_PB7_M0PWM1         0x00011C04
    //��������8MA���������
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_6,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_32);     // PWMʱ�����ã�32����Ƶ
    //����PWM������0���Ӽ���������ͬ��
    PWMGenConfigure(PWM0_BASE,PWM_GEN_0,PWM_GEN_MODE_UP_DOWN| PWM_GEN_MODE_NO_SYNC);
    //����PWM������1��Ƶ�ʣ�ʱ��Ƶ��/PWM��Ƶ��/n��80M/32/5000=500HZ
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 5000);
    //����PWM0/PWM1�����������
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,2500);//50%ռ�ձ�
    //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 500);//25%ռ�ձ�
    PWMDeadBandEnable(PWM0_BASE, PWM_GEN_0, 100, 100);
    //ʹ��PWM0��PWM1�����
    PWMOutputState(PWM0_BASE, (PWM_OUT_0_BIT |PWM_OUT_1_BIT), true);
    //ʹ��PWM������
    PWMGenDisable(PWM0_BASE, PWM_GEN_0);
}
