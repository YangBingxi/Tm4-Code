#ifndef _KEY_H
#define _KEY_H

#include "head.h"
#include "delay/delay.h"

extern uint32_t ReadPin0;
extern uint32_t ReadPin4;
extern int  KeyPress4;


void Key_Configure(void);
void Key_Interrupt(void);
int Key_Scan(int PF);
void Int_Handler_GPIOF(void);

#endif
