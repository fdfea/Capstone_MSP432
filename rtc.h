#ifndef RTC_H
#define RTC_H

#include <ti/sysbios/hal/Hwi.h>

struct RTC_Control
{
    Hwi_Handle RtcHwi;

};

//global variable for RTC
static struct RTC_Control RTC_Ctrl;

//interrupt handler
extern void RTC_C_IRQHandler(uintptr_t Arg);

void RTC_init(void);
void RTC_free(void);
void RTC_setTime(time_t Seconds);
time_t RTC_getTime(void);
char *RTC_getDate(void);
void RTC_setAlarm(uint_fast8_t Minutes,
                   uint_fast8_t Hours,
                   uint_fast8_t DoW,
                   uint_fast8_t DoM);

#endif /* RTC_H */
