/*
 * colorful_LED.c
 *
 *  Created on: 2017��10��6��
 *      Author: 79864
 */
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "driverlib/fpu.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "utils/uartstdio.h"
#include "driverlib/pwm.h"

void LED_ColorInit(void)
{
    // PWMʱ�����ã�4��Ƶ
    SysCtlPWMClockSet(SYSCTL_PWMDIV_4);
    //ʹ��PWM1ģ��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    //ʹ��PWM�������GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinConfigure(GPIO_PF1_M1PWM5);  //�ú�����������λ����
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    //����GPIOΪPWM����
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    //����PWM������1���Ӽ�����
    PWMGenConfigure(PWM1_BASE,PWM_GEN_2,PWM_GEN_MODE_UP_DOWN| PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE,PWM_GEN_3,PWM_GEN_MODE_UP_DOWN| PWM_GEN_MODE_NO_SYNC);
     //����PWM������1������
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 25500);        //f~1k
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 25500);        //f~1k
    //����GPIO�����������
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 00000);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 00000);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 00000);
    //ʹ�����
    PWMOutputState(PWM1_BASE, (PWM_OUT_5_BIT|PWM_OUT_6_BIT|PWM_OUT_7_BIT), true);
    //ʹ��PWM������2��3
    PWMGenEnable(PWM1_BASE,PWM_GEN_2);
    PWMGenEnable(PWM1_BASE,PWM_GEN_3);

}
void LED_Color(uint8_t r,uint8_t g,uint8_t b)
{
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, r*100);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, g*100);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, b*100);
}

