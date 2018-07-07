/*
 * colorful_LED.c
 *
 *  Created on: 2017年10月6日
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
    // PWM时钟配置：4分频
    SysCtlPWMClockSet(SYSCTL_PWMDIV_4);
    //使能PWM1模块
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    //使能PWM输出所在GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinConfigure(GPIO_PF1_M1PWM5);  //该函数不能连续位操作
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    //配置GPIO为PWM功能
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    //配置PWM发生器1：加减计数
    PWMGenConfigure(PWM1_BASE,PWM_GEN_2,PWM_GEN_MODE_UP_DOWN| PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE,PWM_GEN_3,PWM_GEN_MODE_UP_DOWN| PWM_GEN_MODE_NO_SYNC);
     //设置PWM发生器1的周期
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 25500);        //f~1k
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 25500);        //f~1k
    //设置GPIO输出的脉冲宽度
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 00000);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 00000);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 00000);
    //使能输出
    PWMOutputState(PWM1_BASE, (PWM_OUT_5_BIT|PWM_OUT_6_BIT|PWM_OUT_7_BIT), true);
    //使能PWM发生器2、3
    PWMGenEnable(PWM1_BASE,PWM_GEN_2);
    PWMGenEnable(PWM1_BASE,PWM_GEN_3);

}
void LED_Color(uint8_t r,uint8_t g,uint8_t b)
{
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, r*100);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, g*100);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, b*100);
}

