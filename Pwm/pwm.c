#include "pwm.h"
void BeepPwmInit (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);//使能PWM0模块
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//使能PWM0和PWM1输出所在GPIO
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);//配置PH0/PH1为PWM功能
    GPIOPinConfigure(GPIO_PB6_M0PWM0);    //#define GPIO_PB6_M0PWM0         0x00011804
    GPIOPinConfigure(GPIO_PB7_M0PWM1);    //#define GPIO_PB7_M0PWM1         0x00011C04
    //驱动电流8MA，推挽输出
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_6,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_32);     // PWM时钟配置：32不分频
    //配置PWM发生器0：加减计数，不同步
    PWMGenConfigure(PWM0_BASE,PWM_GEN_0,PWM_GEN_MODE_UP_DOWN| PWM_GEN_MODE_NO_SYNC);
    //设置PWM发生器1的频率，时钟频率/PWM分频数/n，80M/32/5000=500HZ
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 5000);
    //设置PWM0/PWM1输出的脉冲宽度
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,2500);//50%占空比
    //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 500);//25%占空比
    PWMDeadBandEnable(PWM0_BASE, PWM_GEN_0, 100, 100);
    //使能PWM0和PWM1的输出
    PWMOutputState(PWM0_BASE, (PWM_OUT_0_BIT |PWM_OUT_1_BIT), true);
    //使能PWM发生器
    PWMGenDisable(PWM0_BASE, PWM_GEN_0);
}
