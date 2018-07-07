
#include "delay.h"

void delay_ms(unsigned long y)
{
    SysCtlDelay(SysCtlClockGet()*y/(3000));
}
void delay_us(unsigned long y)
{
    SysCtlDelay(SysCtlClockGet()*y/(3000000));
}

