#ifndef PWM_H
#define PWM_H

#include <stdint.h>

void pwmSetup(void);
void setLeftPWM(int32_t speed);
void setRightPWM(int32_t speed);
void beep(uint32_t);

#define R_PWM TIM3->CCR2
#define L_PWM TIM3->CCR3


#define turnMotorOff(void) setLeftPWM(0);setRightPWM(0)

#endif


