
#ifndef TIMERS_SERVICES_H_
#define TIMERS_SERVICES_H_


#include "Timers.h"

void PWM_Init(void);
void PWM_Freq_KHZ_Timer1(u16 freq);
void PWM_Freq_HZ_Timer1(u16 freq);
void PWM_Duty_Timer1(u16 duty);
void PWM_Duty_Timer0(u16 duty);

void Timer1_SetInterruptTime_ms (u16 time,void(*LocalFptr)(void));
void Timer1_SetInterruptTime_us (u16 time,void(*LocalFptr)(void));
void Timer1_SetInterruptTime_s (u16 time,void(*LocalFptr)(void));
void PWM_Measure(u32* Pfreq,u8* Pduty);
void PWM_Measure2(u32* Pfreq,u8* Pduty);
static void Func_ICU(void);
#endif /* TIMERS_SERVICES_H_ */