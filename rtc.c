#include <time.h>

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/sysbios/family/arm/msp432/Seconds.h>

#include "rtc.h"

void RTC_init(void)
{
    MAP_WDT_A_holdTimer();

    /*
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ, \
        GPIO_PIN0 | GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

    CS_setExternalClockSourceFrequency(32000,48000000);

    CS_startLFXT(CS_LFXT_DRIVE3);

    MAP_RTC_C_initCalendar(&currentTime, RTC_C_FORMAT_BCD);
    */

    MAP_RTC_C_setCalendarEvent(RTC_C_CALENDAREVENT_MINUTECHANGE);

    MAP_RTC_C_clearInterruptFlag(RTC_C_TIME_EVENT_INTERRUPT \
        | RTC_C_CLOCK_ALARM_INTERRUPT);
    MAP_RTC_C_enableInterrupt(RTC_C_TIME_EVENT_INTERRUPT \
        | RTC_C_CLOCK_ALARM_INTERRUPT);

    MAP_Interrupt_enableInterrupt(INT_RTC_C);
    MAP_Interrupt_enableMaster();

    Hwi_Params HwiParams;

    Hwi_Params_init(&HwiParams);
    HwiParams.priority = 0x40;

    RTC_Ctrl.RtcHwi = Hwi_create(INT_RTC_C, RTC_C_IRQHandler, &HwiParams, NULL);
    if (RTC_Ctrl.RtcHwi == NULL)
    {
        while (1);
    }

    MAP_RTC_C_startClock();
}

void RTC_free(void)
{
    Hwi_delete(&RTC_Ctrl.RtcHwi);
}

void RTC_setTime(time_t Seconds)
{
    Seconds_set((UInt32) Seconds);
}

time_t RTC_getTime(void)
{
    return (time_t) Seconds_get();
}

char *RTC_getDate(void)
{
    time_t secs = RTC_getTime();
    char *pDate = ctime(&secs);
    return pDate;
}

void RTC_setAlarm(uint_fast8_t Minutes,
                   uint_fast8_t Hours,
                   uint_fast8_t DoW,
                   uint_fast8_t DoM)
{
    MAP_RTC_C_setCalendarAlarm(Minutes, Hours, DoW, DoM);
}
