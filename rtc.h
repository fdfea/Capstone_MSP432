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

void rtc_init(void);
void rtc_free(void);
void rtc_set_time(time_t Seconds);
time_t rtc_get_time(void);
char *rtc_get_date(void);
void rtc_set_alarm(uint_fast8_t Minutes,
                   uint_fast8_t Hours,
                   uint_fast8_t DoW,
                   uint_fast8_t DoM);

#endif /* RTC_H */
