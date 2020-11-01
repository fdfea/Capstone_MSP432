/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
 /*****************************************************************************

 Application Name     -   Get Time and Weather Demo
 Application Overview -   
 
 Application Details  - 
 
 *****************************************************************************/
 
//*****************************************************************************
//
//! \addtogroup get_time_and_weather
//! @{
//
//*****************************************************************************

/* Standard includes                                                          */
#include <get_time.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>

#include <mqueue.h>
#include <time.h>
#include <pthread.h>

/* Simplelink includes                                                        */
#include <ti/drivers/net/wifi/simplelink.h>

/* Driverlib includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* SlNetSock includes (to enable portability to other interfaces like ETH)    */
#include <ti/drivers/net/wifi/slnetifwifi.h>

/* Common interface includes                                                  */
#include "network_if.h"
#include "uart_term.h"

/* Application includes                                                       */
#include "Board.h"
#include "pthread.h"
#include "ustdlib.h"

#include "rtc.h"

/* Weather */
#define WEATHER_SERVER  "api.openweathermap.org"
#define PREFIX_BUFFER   "GET /data/2.5/forecast?q="
#ifdef IMPERIAL
#define POST_BUFFER     "&mode=xml&units=imperial&APPID=62c0faaec06406fd96cb58cfac0ca642\r\n HTTP/1.1\r\nHost:api.openweathermap.org\r\nAccept: */"
#else
#define POST_BUFFER     "&mode=xml&units=metric&APPID=62c0faaec06406fd96cb58cfac0ca642\r\n HTTP/1.1\r\nHost:api.openweathermap.org\r\nAccept: */"
#endif
#define POST_BUFFER2    "*\r\n\r\n"

//***** Definitions *****
#define TIMEOUT_SEC     2
#define TIMEOUT_USEC    0
#define DATA_PORT       5004

/* Expiration value for the timer that is being used to toggle the Led.       */
#define TIMER_EXPIRATION_VALUE   100 * 1000000

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************

/* General application functions */
void TimerPeriodicIntHandler(sigval val);
void LedTimerConfigNStart();
void LedTimerDeinitStop();
void RTC_C_IRQHandler(uintptr_t Arg);
static void *udpServerThreadProc(void *pArg);
extern void *i2cThreadProc(void *arg0);

/****************************************************************************************************************
                   GLOBAL VARIABLES
****************************************************************************************************************/
Application_CB App_CB;
demo_Config_t       flashDemoConfigParams = { "cc3120-demo",
                                              "password",
                                              SL_WLAN_SEC_TYPE_WPA_WPA2,
                                              0xFF,
                                              "Dallas, US",
                                              0xFA,         // GMT-6
                                              0xFF }; //simplelinkapps1, ecsapps1

pthread_t gProvisioningThread   = (pthread_t) NULL;

/* AP Security Parameters                                                     */
SlWlanSecParams_t SecurityParams = { 0 };

/* Date and Time Parameters */
const uint8_t SNTPserver[30] = "time-c.nist.gov";

/* Tuesday is the 1st day in 2013 - the relative year */
const uint8_t daysOfWeek2013[7][3] = {{"Tue"},
                                   {"Wed"},
                                   {"Thu"},
                                   {"Fri"},
                                   {"Sat"},
                                   {"Sun"},
                                   {"Mon"}};

const uint8_t monthOfYear[12][3] = {{"Jan"},
                                 {"Feb"},
                                 {"Mar"},
                                 {"Apr"},
                                 {"May"},
                                 {"Jun"},
                                 {"Jul"},
                                 {"Aug"},
                                 {"Sep"},
                                 {"Oct"},
                                 {"Nov"},
                                 {"Dec"}};

const uint8_t numOfDaysPerMonth[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

const uint8_t digits[] = "0123456789";

uint8_t locTempString[400];
uint8_t NumDaysReceived = 0;
Tmp_Day_t Tmp_Day[5];

volatile bool udpThreadStop;
volatile bool i2cThreadStop;

/****************************************************************************************************************
                   Banner VARIABLES
****************************************************************************************************************/
char lineBreak[]                = "\n\r";

/****************************************************************************************************************
                 Local Functions
****************************************************************************************************************/
int32_t WiFi_IF_Connect(void);

//static void RTC_C_IRQHandler(uintptr_t arg);

//*****************************************************************************
//
//! Push Button Handler1(GPIOSW2). Press push button1 (GPIOSW2) Whenever user
//! wants to publish a message. Write message into message queue signaling the
//! event publish messages
//!
//! \param none
//!
//! return none
//
//*****************************************************************************
void pushButtonInterruptHandler2(uint_least8_t index)
{
    /* Disable the SW2 interrupt */
    GPIO_disableInt(Board_BUTTON0);
}

//*****************************************************************************
//
//! Push Button Handler2(GPIOSW3). Press push button3 Whenever user wants to
//! disconnect from the remote broker. Write message into message queue
//! indicating disconnect from broker.
//!
//! \param none
//!
//! return none
//
//*****************************************************************************
void pushButtonInterruptHandler3(uint_least8_t index)
{
    /* Disable the SW3 interrupt */
    GPIO_disableInt(Board_BUTTON1);
}

//*****************************************************************************
//
//! Application Boarders display on UART
//!
//! \param  ch - Character to be displayed , n - number of time to display
//!
//! \return none
//!
//*****************************************************************************
void printBorder(char ch, int n)
{
    int        i = 0;

    for(i=0; i<n; i++)    putch(ch);
}

/***********************************************************
  Function: itoa
*/
static uint16_t itoa(int16_t cNum, uint8_t *cString)
{
    uint16_t length = 0;
    uint8_t* ptr = NULL;
    int16_t uTemp = cNum;

    /* value 0 is a special case */
    if (cNum == 0)
    {
        length = 1;
        *cString = '0';

        return length;
    }

    /* Find out the length of the number, in decimal base */
    length = 0;
    while (uTemp > 0)
    {
        uTemp /= 10;
        length++;
    }

    /* Do the actual formatting, right to left */
    uTemp = cNum;
    ptr = cString + length;
    while (uTemp > 0)
    {
        --ptr;
        *ptr = digits[uTemp % 10];
        uTemp /= 10;
    }

    return length;
}

//*****************************************************************************
//
//! \brief This function reboot the M4 host processor
//!
//! \param[in]  none
//!
//! \return none
//!
////****************************************************************************
//void mcuReboot(void)
//{
//    /* stop network processor activities before reseting the MCU */
//    sl_Stop(SL_STOP_TIMEOUT );
//
//    UART_PRINT("[Common] CC3220 MCU reset request\r\n");
//
//    /* Reset the MCU in order to test the bundle */
//    PRCMHibernateCycleTrigger();
//}

//*****************************************************************************
//
//! Periodic Timer Interrupt Handler
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void TimerPeriodicIntHandler(sigval val)
{
    /* Increment our interrupt counter.                                       */
    App_CB.timerInts++;

    if (!(App_CB.timerInts & 0x1))
    {
        /* Turn Led Off                                                       */
        GPIO_write(Board_LED0, Board_LED_OFF);
    }
    else
    {
        /* Turn Led On                                                        */
        GPIO_write(Board_LED0, Board_LED_ON);
    }
}

//*****************************************************************************
//
//! Function to configure and start timer to blink the LED while device is
//! trying to connect to an AP
//!
//! \param none
//!
//! return none
//
//*****************************************************************************
void LedTimerConfigNStart()
{
    struct itimerspec value;
    sigevent sev;

    /* Create Timer                                                           */
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_notify_function = &TimerPeriodicIntHandler;
    timer_create(2, &sev, &App_CB.timer);

    /* start timer                                                            */
    value.it_interval.tv_sec = 0;
    value.it_interval.tv_nsec = TIMER_EXPIRATION_VALUE;
    value.it_value.tv_sec = 0;
    value.it_value.tv_nsec = TIMER_EXPIRATION_VALUE;

    timer_settime(App_CB.timer, 0, &value, NULL);
}

//*****************************************************************************
//
//! Disable the LED blinking Timer as Device is connected to AP
//!
//! \param none
//!
//! return none
//
//*****************************************************************************
void LedTimerDeinitStop()
{
    /* Disable the LED blinking Timer as Device is connected to AP.           */
    timer_delete(App_CB.timer);
}

//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
int32_t DisplayAppBanner(char* appName, char* appVersion)
{
     int32_t            ret = 0;
     uint8_t            macAddress[SL_MAC_ADDR_LEN];
     uint16_t           macAddressLen = SL_MAC_ADDR_LEN;
     uint16_t           ConfigSize = 0;
     uint8_t            ConfigOpt = SL_DEVICE_GENERAL_VERSION;
     SlDeviceVersion_t ver = {0};

     ConfigSize = sizeof(SlDeviceVersion_t);

     /* Print device version info. */
     ret = sl_DeviceGet(SL_DEVICE_GENERAL, &ConfigOpt, &ConfigSize, (uint8_t*)(&ver));

     /* Print device Mac address */
     ret = sl_NetCfgGet(SL_NETCFG_MAC_ADDRESS_GET, 0, &macAddressLen, &macAddress[0]);

     ConfigOpt = SL_DEVICE_IOT_UDID;
     ConfigSize = sizeof(App_CB.lockUDID);

     ret = sl_DeviceGet(SL_DEVICE_IOT, &ConfigOpt, &ConfigSize, App_CB.lockUDID);

     if(ret < 0)
     {
         UART_PRINT("\t Error - Failed to get device UDID (%d)\r\n", ret);
     }

     UART_PRINT(lineBreak);
     UART_PRINT("\t");
     printBorder('=', 44);
     UART_PRINT(lineBreak);
     UART_PRINT("\t CC3120  %s Example Ver: %s",appName, appVersion);
     UART_PRINT(lineBreak);
     UART_PRINT("\t");
     printBorder('=', 44);
     UART_PRINT(lineBreak);
     UART_PRINT(lineBreak);
     UART_PRINT("\t CHIP: 0x%x",ver.ChipId);
     UART_PRINT(lineBreak);
     UART_PRINT("\t MAC:  %d.%d.%d.%d",ver.FwVersion[0],ver.FwVersion[1],ver.FwVersion[2],ver.FwVersion[3]);
     UART_PRINT(lineBreak);
     UART_PRINT("\t PHY:  %d.%d.%d.%d",ver.PhyVersion[0],ver.PhyVersion[1],ver.PhyVersion[2],ver.PhyVersion[3]);
     UART_PRINT(lineBreak);
     UART_PRINT("\t NWP:  %d.%d.%d.%d",ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3]);
     UART_PRINT(lineBreak);
     UART_PRINT("\t ROM:  %d",ver.RomVersion);
     UART_PRINT(lineBreak);
     UART_PRINT("\t HOST: %s", SL_DRIVER_VERSION);
     UART_PRINT(lineBreak);
     UART_PRINT("\t MAC address: %02x:%02x:%02x:%02x:%02x:%02x", macAddress[0], macAddress[1], macAddress[2], macAddress[3], macAddress[4], macAddress[5]);
     UART_PRINT(lineBreak);
     UART_PRINT(lineBreak);
     UART_PRINT("\t");
     printBorder('=', 44);
     UART_PRINT(lineBreak);
     UART_PRINT(lineBreak);

     return ret;
}

//*****************************************************************************
//
//! This function connect the MQTT device to an AP with the SSID which was
//! configured in SSID_NAME definition which can be found in Network_if.h file,
//! if the device can't connect to to this AP a request from the user for other
//! SSID will appear.
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
int32_t WiFi_IF_Connect()
{
    int32_t lRetVal;
    char SSID_Remote_Name[32];
    int8_t Str_Length;

    memset(SSID_Remote_Name, '\0', sizeof(SSID_Remote_Name));
    Str_Length = strlen(SSID_NAME);

    // If SSID_NAME is defined as a non-empty string, copy into SSID name buffer
    if (Str_Length)
    {
        /* Copy the Default SSID to the local variable                        */
        strncpy(SSID_Remote_Name, SSID_NAME, Str_Length);
    }

    GPIO_write(Board_LED0, Board_LED_OFF);
    GPIO_write(Board_LED1, Board_LED_OFF);
    GPIO_write(Board_LED2, Board_LED_OFF);

    /* Reset The state of the machine                                         */
    Network_IF_ResetMCUStateMachine();

    /* Start the driver                                                       */
    lRetVal = Network_IF_InitDriver(ROLE_STA);
    if (lRetVal < 0)
    {
        UART_PRINT("Failed to start SimpleLink Device\n\r", lRetVal);
        return -1;
    }

    /* switch on Green LED to indicate Simplelink is properly up.             */
    GPIO_write(Board_LED2, Board_LED_ON);

    // Start Timer to blink Red LED till AP connection
    LedTimerConfigNStart();

    /* Initialize AP security params                                          */
    SecurityParams.Key = (signed char *) SECURITY_KEY;
    SecurityParams.KeyLen = strlen(SECURITY_KEY);
    SecurityParams.Type = SECURITY_TYPE;

    /* Connect to the Access Point                                            */
    lRetVal = Network_IF_ConnectAP(SSID_Remote_Name, SecurityParams);
    if (lRetVal < 0)
    {
        UART_PRINT("Connection to an AP failed\n\r");
        return -1;
    }

    // Disable the LED blinking Timer as Device is connected to AP.
    LedTimerDeinitStop();

    // Switch ON RED LED to indicate that Device acquired an IP.
    GPIO_write(Board_LED0, Board_LED_ON);

    sleep(1);

    GPIO_write(Board_LED0, Board_LED_OFF);
    GPIO_write(Board_LED1, Board_LED_OFF);
    GPIO_write(Board_LED2, Board_LED_OFF);

    return 0;
}

/***********************************************************
  Function: getHostIPforWeather
*/
static int32_t getHostIPforWeather(void)
{
    int32_t status = -1;

    /* Receive IP for host based on the address */
    status = sl_NetAppDnsGetHostByName((signed char *)App_CB.weatherHostName,
                                       strlen((const char*)App_CB.weatherHostName),
                                       (unsigned long*)&App_CB.weatherDestinationIP,
                                       SL_AF_INET);
    ASSERT_ON_ERROR(status);

    return 0;
}

/***********************************************************
  Function: getHostIPforDate
*/
static int32_t getHostIPforDate(void)
{
    int32_t status = -1;
    App_CB.timeDestinationIP = 0;

    /* Retreive IP for host based on the address */
    status = sl_NetAppDnsGetHostByName((signed char *)App_CB.timeHostName,
                                       strlen((const char*)App_CB.timeHostName),
                                       (unsigned long*)&App_CB.timeDestinationIP,
                                       SL_AF_INET);
    ASSERT_ON_ERROR(status);

    return 0;
}

/***********************************************************
  Function: createConnectionforWeather
*/
static int32_t createConnectionforWeather(void)
{
    SlSockAddrIn_t  Addr;

    int16_t sd = 0;
    int16_t AddrSize = 0;
    int32_t ret_val = 0;

    Addr.sin_family = SL_AF_INET;
    Addr.sin_port = sl_Htons(80);

    /* Change the DestinationIP endianity, to big endian */
    Addr.sin_addr.s_addr = sl_Htonl(App_CB.weatherDestinationIP);

    AddrSize = sizeof(SlSockAddrIn_t);

    sd = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
    if( sd < 0 )
    {
        UART_PRINT(" Error creating socket\n\r\n\r");
        ASSERT_ON_ERROR(sd);
    }

    ret_val = sl_Connect(sd, ( SlSockAddr_t *)&Addr, AddrSize);
    if( ret_val < 0 )
    {
        /* error */
        UART_PRINT(" Error connecting to server\n\r\n\r");
        ASSERT_ON_ERROR(ret_val);
    }

    return sd;
}

/***********************************************************
  Function: createConnectionforDate
*/
static int32_t createConnectionforDate(void)
{
    int32_t sd = 0;
    struct SlTimeval_t TimeVal;

    sd = sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, /*SL_IPPROTO_UDP*/ 0);
    if( sd < 0 )
    {
        UART_PRINT(" Error creating socket\n\r\n\r");
        ASSERT_ON_ERROR(sd);
    }

    TimeVal.tv_sec = TIMEOUT_SEC;
    TimeVal.tv_usec = TIMEOUT_USEC;
    sl_SetSockOpt(sd,SL_SOL_SOCKET,SL_SO_RCVTIMEO, (_u8 *)&TimeVal,sizeof(TimeVal));

    return sd;
}

/***********************************************************
  Function: getForecastData
*/
static int32_t getForecastData(void)
{
    uint8_t *p_startPtr = NULL;
    uint8_t *p_endPtr = NULL;
    uint8_t* p_bufLocation = NULL;
    uint8_t  temp_endValue;
    int32_t  retVal = -1;
    int8_t   newDay = 1;
    uint8_t  msgIndex;
    uint16_t dataIndex;
    uint16_t recvBuffoffset;

    UART_PRINT("Start getforecastdata\n\r");
    memset(App_CB.weatherRecvbuff, 0, sizeof(App_CB.weatherRecvbuff));

    /* Puts together the HTTP GET string. */
    p_bufLocation = App_CB.weatherSendBuff;
    strcpy((char *)p_bufLocation, PREFIX_BUFFER);

    p_bufLocation += strlen(PREFIX_BUFFER);
    strcpy((char *)p_bufLocation, (const char *)App_CB.weatherCityName);

    p_bufLocation += strlen((const char *)App_CB.weatherCityName);
    strcpy((char *)p_bufLocation, POST_BUFFER);

    p_bufLocation += strlen(POST_BUFFER);
    strcpy((char *)p_bufLocation, POST_BUFFER2);

    /* Send the HTTP GET string to the open TCP/IP socket. */
    retVal = sl_Send(App_CB.weatherSockID, App_CB.weatherSendBuff,
                     strlen((const char *)App_CB.weatherSendBuff), 0);

    UART_PRINT("strlen: %d \n\r", strlen((const char *)App_CB.weatherSendBuff));
    UART_PRINT("retval: %d \n\r", retVal);

    if(retVal != strlen((const char *)App_CB.weatherSendBuff))
        ASSERT_ON_ERROR(retVal);

    /* Receive response */
    for (msgIndex = 0; msgIndex < 18; msgIndex++)
    {
        retVal = sl_Recv(App_CB.weatherSockID,
                         App_CB.weatherRecvbuff,
                         1000, 0);
        ASSERT_ON_ERROR(retVal);

        /* Remove unwanted characters */
        for (dataIndex = 0; dataIndex < MAX_SEND_RCV_SIZE; dataIndex++)
        {
            if (App_CB.weatherRecvbuff[dataIndex] == 0)
            {
                App_CB.weatherRecvbuff[dataIndex] = '0';
            }
        }
        App_CB.weatherRecvbuff[dataIndex] = '\0';

        recvBuffoffset = 0;

        if(NumDaysReceived < 5)
        {
            /* Get the date and temp string */
            p_startPtr = (uint8_t *)strstr((const char *)&App_CB.weatherRecvbuff[recvBuffoffset],
                                           "time from=");
            if( NULL != p_startPtr )
            {
                p_endPtr = (uint8_t *)strstr((const char *)p_startPtr, "/temperature>");
                if( NULL != p_endPtr )
                {
                    recvBuffoffset = p_endPtr - &App_CB.weatherRecvbuff[0];
                    temp_endValue = *p_endPtr;
                    *p_endPtr = 0;
                    usprintf((char *)locTempString,"%s",p_startPtr);
                    *p_endPtr = temp_endValue;

                    /* Parse the data */
                    p_startPtr = (uint8_t *)strstr((const char *)locTempString, "time from=");
                    p_startPtr = p_startPtr + strlen("time from=") + 1;
                    p_endPtr = (uint8_t *)strstr((const char *)p_startPtr, "T");
                    if( NULL != p_endPtr )
                    {
                        temp_endValue = *p_endPtr;
                        *p_endPtr = 0;
                        usprintf((char *)Tmp_Day[NumDaysReceived].TmpDate,"%s",
                                 p_startPtr);
                        *p_endPtr = temp_endValue;

                        if (NumDaysReceived > 0)
                        {
                            newDay = ustrcmp((const char *)Tmp_Day[NumDaysReceived-1].TmpDate,
                                             (const char *)Tmp_Day[NumDaysReceived].TmpDate);
                        }

                        if (newDay != 0)
                        {
                            p_startPtr = (uint8_t *)strstr((const char *)locTempString, "ial\" value=");
                            if( NULL != p_startPtr )
                            {
                                p_startPtr = p_startPtr + strlen("ial\" value=") + 1;
                                p_endPtr = (uint8_t *)strstr((const char *)p_startPtr, "\"");
                                if( NULL != p_endPtr )
                                {
                                    temp_endValue = *p_endPtr;
                                    *p_endPtr = 0;
                                    usprintf((char *)Tmp_Day[NumDaysReceived].TmpCurrent,"%s",
                                             p_startPtr);
                                    *p_endPtr = temp_endValue;
                                }
                            }

                            p_startPtr = (uint8_t *)strstr((const char *)locTempString, "\" name=");
                            if( NULL != p_startPtr )
                            {
                                p_startPtr = p_startPtr + strlen("\" name=") + 1;
                                p_endPtr = (uint8_t *)strstr((const char *)p_startPtr, "\"");
                                if( NULL != p_endPtr )
                                {
                                    temp_endValue = *p_endPtr;
                                    *p_endPtr = 0;
                                    usprintf((char *)Tmp_Day[NumDaysReceived].WeatherSymbol,"%s",
                                             p_startPtr);
                                    *p_endPtr = temp_endValue;
                                }
                            }

                            p_startPtr = (uint8_t *)strstr((const char *)locTempString, "min=");
                            if( NULL != p_startPtr )
                            {
                                p_startPtr = p_startPtr + strlen("min=") + 1;
                                p_endPtr = (uint8_t *)strstr((const char *)p_startPtr, "\"");
                                if( NULL != p_endPtr )
                                {
                                    temp_endValue = *p_endPtr;
                                    *p_endPtr = 0;
                                    usprintf((char *)Tmp_Day[NumDaysReceived].TmpMin,"%s",p_startPtr);
                                    *p_endPtr = temp_endValue;
                                }
                            }
                            p_startPtr = (uint8_t *)strstr((const char *)locTempString, "max=");
                            if( NULL != p_startPtr )
                            {
                                p_startPtr = p_startPtr + strlen("max=") + 1;
                                p_endPtr = (uint8_t *)strstr((const char *)p_startPtr, "\"");
                                if( NULL != p_endPtr )
                                {
                                    temp_endValue = *p_endPtr;
                                    *p_endPtr = 0;
                                    usprintf((char *)Tmp_Day[NumDaysReceived].TmpMax,"%s",p_startPtr);
                                    *p_endPtr = temp_endValue;
                                }
                            }
                            NumDaysReceived++;
                        }
                        else
                        {
                            p_startPtr = (uint8_t *)strstr((const char *)locTempString, "min=");
                            if( NULL != p_startPtr )
                            {
                                p_startPtr = p_startPtr + strlen("min=") + 1;
                                p_endPtr = (uint8_t *)strstr((const char *)p_startPtr, "\"");
                                if( NULL != p_endPtr )
                                {
                                    temp_endValue = *p_endPtr;

                                    *p_endPtr = 0;
                                    if (ustrcmp((const char *)Tmp_Day[NumDaysReceived - 1].TmpMin,
                                                (const char *)p_startPtr) == 1)
                                    {
                                        usprintf((char *)Tmp_Day[NumDaysReceived- 1].TmpMin,"%s",
                                                 p_startPtr);
                                    }
                                    *p_endPtr = temp_endValue;
                                }
                            }

                            p_startPtr = (uint8_t *)strstr((const char *)locTempString, "max=");
                            if( NULL != p_startPtr )
                            {
                                p_startPtr = p_startPtr + strlen("max=") + 1;
                                p_endPtr = (uint8_t *)strstr((const char *)p_startPtr, "\"");
                                if( NULL != p_endPtr )
                                {
                                    temp_endValue = *p_endPtr;
                                    *p_endPtr = 0;
                                    if (ustrcmp((const char *)Tmp_Day[NumDaysReceived - 1].TmpMax,
                                                (const char *)p_startPtr) == -1)
                                    {
                                        usprintf((char *)Tmp_Day[NumDaysReceived - 1].TmpMax,"%s",
                                                 p_startPtr);
                                    }
                                    *p_endPtr = temp_endValue;
                                }
                            }
                        }
                    }
                }
            }
            else
            {
                break;
            }
        }
    }

    UART_PRINT("\n\r\n\r");
    for (dataIndex = 0; dataIndex < 5; dataIndex++)
    {
        UART_PRINT(" Date: %s, Symbol:%s CUR:%s MIN:%s MAX:%s \n\r",
                   Tmp_Day[dataIndex].TmpDate, Tmp_Day[dataIndex].WeatherSymbol,
                   Tmp_Day[dataIndex].TmpCurrent, Tmp_Day[dataIndex].TmpMin,
                   Tmp_Day[dataIndex].TmpMax);
    }
    return 0;   //SUCCESS
}


/***********************************************************
  Function: getSNTPTime
*/
static int32_t getSNTPTime(int16_t gmt_hr, int16_t gmt_min)
{
    /*
                                NTP Packet Header:


           0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9  0  1
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |LI | VN  |Mode |    Stratum    |     Poll      |   Precision    |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |                          Root  Delay                           |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |                       Root  Dispersion                         |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |                     Reference Identifier                       |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |                                                                |
          |                    Reference Time-stamp (64)                    |
          |                                                                |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |                                                                |
          |                    Originate Time-stamp (64)                    |
          |                                                                |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |                                                                |
          |                     Receive Time-stamp (64)                     |
          |                                                                |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |                                                                |
          |                     Transmit Time-stamp (64)                    |
          |                                                                |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |                 Key Identifier (optional) (32)                 |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |                                                                |
          |                                                                |
          |                 Message Digest (optional) (128)                |
          |                                                                |
          |                                                                |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    */

    SlSockAddrIn_t  LocalAddr;
    SlSockAddr_t Addr;

    uint8_t    dataBuf[48];
    int32_t    retVal = -1;
    int32_t    AddrSize = 0;

    /* For time zone with negative GMT value, change minutes to negative for
     * computation */
    if(gmt_hr < 0 && gmt_min > 0)
        gmt_min = gmt_min * (-1);

    memset(dataBuf, 0, sizeof(dataBuf));
    dataBuf[0] = '\x1b';

    Addr.sa_family = SL_AF_INET;
    /* the source port */
    Addr.sa_data[0] = 0x00;
    Addr.sa_data[1] = 0x7B;    /* 123 */
    Addr.sa_data[2] = (uint8_t)((App_CB.timeDestinationIP >> 24) & 0xff);
    Addr.sa_data[3] = (uint8_t)((App_CB.timeDestinationIP >> 16) & 0xff);
    Addr.sa_data[4] = (uint8_t)((App_CB.timeDestinationIP >> 8) & 0xff);
    Addr.sa_data[5] = (uint8_t) (App_CB.timeDestinationIP & 0xff);

    retVal = sl_SendTo(App_CB.timeSockID, dataBuf, sizeof(dataBuf), 0,
                     &Addr, sizeof(Addr));

    if (retVal != sizeof(dataBuf))
    {
        /* could not send SNTP request */
        UART_PRINT(" Device couldn't send SNTP request\n\r\n\r");
    }

    AddrSize = sizeof(SlSockAddrIn_t);
    LocalAddr.sin_family = SL_AF_INET;
    LocalAddr.sin_port = 0;
    LocalAddr.sin_addr.s_addr = SL_INADDR_ANY/*0*/;

    retVal = sl_RecvFrom(App_CB.timeSockID, dataBuf, sizeof(dataBuf), 0,
                       (SlSockAddr_t *)&LocalAddr,  (SlSocklen_t*)&AddrSize);
    if (retVal <= 0)
    {
        UART_PRINT(" Device couldn't receive time information \n\r");
    }

    if (/*(dataBuf[0] & 0x7) != 4*/ false) /* expect only server response */
    {
        /* MODE is not server, abort */
        UART_PRINT(" Device is expecting response from server only!\n\r");
    }
    else
    {
        App_CB.timeElapsedSec = dataBuf[40];
        App_CB.timeElapsedSec  <<= 8;
        App_CB.timeElapsedSec  += dataBuf[41];
        App_CB.timeElapsedSec  <<= 8;
        App_CB.timeElapsedSec  += dataBuf[42];
        App_CB.timeElapsedSec  <<= 8;
        App_CB.timeElapsedSec  += dataBuf[43];

        /*
        App_CB.timeElapsedSec  -= TIME2013;
        App_CB.timeElapsedSec  += (gmt_hr * SEC_IN_HOUR);
        App_CB.timeElapsedSec  += (gmt_min * SEC_IN_MIN);
        App_CB.timeCCPtr = &App_CB.time[0];
        App_CB.timeSGeneralVar = App_CB.timeElapsedSec/SEC_IN_DAY;
        memcpy(App_CB.timeCCPtr, daysOfWeek2013[App_CB.timeSGeneralVar%7], 3);
        App_CB.timeCCPtr += 3;
        *App_CB.timeCCPtr++ = '\x20';
        App_CB.timeSGeneralVar %= 365;
        for (index = 0; index < 12; index++)
        {
            App_CB.timeSGeneralVar -= numOfDaysPerMonth[index];
            if (App_CB.timeSGeneralVar < 0)
                break;
        }
        memcpy(App_CB.timeCCPtr, monthOfYear[index], 3);
        App_CB.timeCCPtr += 3;
        *App_CB.timeCCPtr++ = '\x20';
        App_CB.timeSGeneralVar += numOfDaysPerMonth[index];
        App_CB.timeCCLen = itoa(App_CB.timeSGeneralVar + 1, App_CB.timeCCPtr);
        App_CB.timeCCPtr += App_CB.timeCCLen;
        *App_CB.timeCCPtr++ = '\x20';
        App_CB.timeUGeneralVar = App_CB.timeElapsedSec/SEC_IN_DAY;
        App_CB.timeUGeneralVar /= 365;
        App_CB.timeCCLen = itoa(YEAR2013 + App_CB.timeUGeneralVar , App_CB.timeCCPtr);
        App_CB.timeCCPtr += App_CB.timeCCLen;
        *App_CB.timeCCPtr++ = '\x20';
        App_CB.timeUGeneralVar = App_CB.timeElapsedSec%SEC_IN_DAY;
        App_CB.timeUGeneralVar1 = App_CB.timeUGeneralVar%SEC_IN_HOUR;
        App_CB.timeUGeneralVar /= SEC_IN_HOUR;
        App_CB.timeCCLen = itoa(App_CB.timeUGeneralVar, App_CB.timeCCPtr);
        App_CB.timeCCPtr += App_CB.timeCCLen;
        *App_CB.timeCCPtr++ = ':';
        App_CB.timeUGeneralVar = App_CB.timeUGeneralVar1/SEC_IN_MIN;
        App_CB.timeUGeneralVar1 %= SEC_IN_MIN;
        App_CB.timeCCLen = itoa(App_CB.timeUGeneralVar, App_CB.timeCCPtr);
        App_CB.timeCCPtr += App_CB.timeCCLen;
        *App_CB.timeCCPtr++ = ':';
        App_CB.timeCCLen = itoa(App_CB.timeUGeneralVar1, App_CB.timeCCPtr);
        App_CB.timeCCPtr += App_CB.timeCCLen;
        *App_CB.timeCCPtr++ = '\x20';
        *App_CB.timeCCPtr++ = '\0';

        UART_PRINT("\r\n Server ");
        UART_PRINT("%s",SNTPserver);
        UART_PRINT(" has responded with time information");
        UART_PRINT("\n\r\r\n");
        UART_PRINT("%s",App_CB.time);
        UART_PRINT("\n\r\r\n");
        */
    }

    return 0;
}

/***********************************************************
  Function: getWeather
*/
static int32_t getWeather(void)
{
    int32_t retVal = -1;

    strcpy((char *)App_CB.weatherHostName, WEATHER_SERVER);

    retVal = getHostIPforWeather();
    if(retVal < 0)
    {
        UART_PRINT("Unable to reach Host\n\r");
        ASSERT_ON_ERROR(retVal);
    }

    App_CB.weatherSockID = createConnectionforWeather();
    ASSERT_ON_ERROR(App_CB.weatherSockID);

    strcpy((char *)App_CB.weatherCityName, (const char *)flashDemoConfigParams.city);

    retVal = getForecastData();
    ASSERT_ON_ERROR(retVal);

    retVal = sl_Close(App_CB.weatherSockID);
    ASSERT_ON_ERROR(retVal);
    return 0;
}

/***********************************************************
  Function: getTime
*/
static int32_t getTime()
{
    int32_t retVal = -1;

    strcpy((char *)App_CB.timeHostName, (const char *)SNTPserver);

    retVal = getHostIPforDate();
    if(retVal < 0)
    {
        UART_PRINT(" Unable to reach Host\n\r\n\r");
        ASSERT_ON_ERROR(retVal);
    }

    App_CB.timeSockID = createConnectionforDate();
    ASSERT_ON_ERROR(App_CB.timeSockID);

    retVal = getSNTPTime(flashDemoConfigParams.gmtTimeZoneHR,
                         GMT_TIME_ZONE_MIN);
    ASSERT_ON_ERROR(retVal);

    retVal = sl_Close(App_CB.timeSockID);
    ASSERT_ON_ERROR(retVal);
    return 0;
}

static void *udpServerThreadProc(void* pArg)
{
    int32_t sd = 0;
    int32_t retVal = -1;
    SlSocklen_t ServerSize, ClientSize;
    struct SlTimeval_t TimeVal;
    SlSockAddrIn_t ServerAddr, ClientAddr;
    uint8_t dataBuf[48];

    sd = sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, /*SL_IPPROTO_UDP*/ 0);
    if(sd < 0)
    {
        UART_PRINT("Socket create failed\n\r\n\r");
        return NULL;
    }

    TimeVal.tv_sec = 10;
    TimeVal.tv_usec = 0;
    sl_SetSockOpt(sd, SL_SOL_SOCKET, SL_SO_RCVTIMEO, (_u8 *)&TimeVal, sizeof(TimeVal));

    ServerSize = sizeof(ServerAddr);
    ServerAddr.sin_family = SL_AF_INET;
    ServerAddr.sin_port = sl_Htons(DATA_PORT);
    ServerAddr.sin_addr.s_addr = sl_Htonl(SL_INADDR_ANY);

    retVal = sl_Bind(sd, (SlSockAddr_t *)&ServerAddr, ServerSize);
    if (retVal < 0)
    {
        UART_PRINT("Socket bind failed\r\n");
        return NULL;
    }

    UART_PRINT("Listening on port %d...\r\n", DATA_PORT);

    while (!udpThreadStop)
    {
        ClientSize = sizeof(SlSockAddrIn_t);
        memset(&ClientAddr, 0, ClientSize);
        memset(dataBuf, 0, sizeof(dataBuf));

        retVal = sl_RecvFrom(sd, dataBuf, sizeof(dataBuf), 0,
                             (SlSockAddr_t *)&ClientAddr, (SlSocklen_t*)&ClientSize);
        if (retVal <= 0)
        {
            //UART_PRINT("Recieve timed out\n\r");
        }
        else
        {
            UART_PRINT("Recieved: %s\r\n", dataBuf);
        }
    }

    retVal = sl_Close(sd);
    if (retVal < 0)
    {
        UART_PRINT("Socket close failed\r\n");
    }
    return NULL;
}

/****************************************************************************************************************
                 Main Thread
****************************************************************************************************************/
void mainThread(void * args)
{
    int32_t retc = 0;

    /* Thread vars */
    pthread_t spawn_thread = (pthread_t) NULL;
    pthread_attr_t pAttrs_spawn;
    struct sched_param priParam;

    //pthread_mutex_t uartMutex;

    /* Peripheral parameters and handles */
    UART_Handle tUartHndl;

    /* Clear lockUDID */
    memset(&App_CB.lockUDID[0], 0x00, sizeof(App_CB.lockUDID));

    /* Initialize SlNetSock layer with CC3x20 interface                      */
    SlNetIf_init(0);
    SlNetIf_add(SLNETIF_ID_1, "CC3220", (const SlNetIf_Config_t *) &SlNetIfConfigWifi, SLNET_IF_WIFI_PRIO);

    SlNetSock_init(0);
    SlNetUtil_init(0);

    /* Init pins used as GPIOs */
    GPIO_init();
    /* Init SPI for communicating between M4 and NWP */
    SPI_init();

    /* Configure the UART                                                     */
    tUartHndl = InitTerm();
    /* remove uart receive from LPDS dependency                               */
    UART_control(tUartHndl, UART_CMD_RXDISABLE, NULL);

    /* Create the sl_Task                                                     */
    pthread_attr_init(&pAttrs_spawn);
    priParam.sched_priority = SPAWN_TASK_PRIORITY;
    retc |= pthread_attr_setschedparam(&pAttrs_spawn, &priParam);
    retc |= pthread_attr_setstacksize(&pAttrs_spawn, TASK_STACK_SIZE);
    retc |= pthread_attr_setdetachstate(&pAttrs_spawn, PTHREAD_CREATE_DETACHED);

    retc |= pthread_create(&spawn_thread, &pAttrs_spawn, sl_Task, NULL);
    if (retc != 0)
    {
        UART_PRINT("could not create simplelink task\n\r");
        while (1); // Insert error handling
    }

    /* Started to allow host to read back MAC address when printing App banner */
    retc = sl_Start(0, 0, 0);
    if (retc < 0)
    {
        /* Handle Error */
        UART_PRINT("\n sl_Start failed\n");
        while (1);// Insert error handling
    }

    /* Output device information to the UART terminal                         */
    retc = DisplayAppBanner(APPLICATION_NAME, APPLICATION_VERSION);

    retc = sl_Stop(SL_STOP_TIMEOUT);
    if (retc < 0)
    {
        /* Handle Error */
        UART_PRINT("\n sl_Stop failed\n");
        while (1);
    }

    /*
    retc = pthread_mutex_init(&uartMutex, NULL);
    if (retc < 0)
    {
        UART_PRINT("\r\n Mutex init failed \r\n");
        while (1);
    }
    */

    /* Main application loop */
    while (1)
    {
        UART_PRINT("\n\r\n\r**** Starting main loop ****\n\r");

        App_CB.resetApplication = false;
        App_CB.initState = 0;

        /* Connect to AP */
        UART_PRINT("Using hardcoded profile for connection.\n\r");
        App_CB.apConnectionState = WiFi_IF_Connect();

        udpThreadStop = false;
        pthread_t udpServerThread;
        pthread_attr_t udpThreadAttr;
        pthread_attr_init(&udpThreadAttr);
        retc |= pthread_attr_setstacksize(&udpThreadAttr, TASK_STACK_SIZE);
        retc |= pthread_attr_setdetachstate(&udpThreadAttr, PTHREAD_CREATE_DETACHED);
        retc |= pthread_create(&udpServerThread, &udpThreadAttr, udpServerThreadProc, NULL);
        if (retc < 0)
        {
            UART_PRINT("UDP Server create failed\r\n");
            while (1);
        }

        i2cThreadStop = false;
        pthread_t i2cThread;
        pthread_attr_t i2cThreadAttr;
        pthread_attr_init(&i2cThreadAttr);
        retc |= pthread_attr_setstacksize(&i2cThreadAttr, TASK_STACK_SIZE);
        retc |= pthread_attr_setdetachstate(&i2cThreadAttr, PTHREAD_CREATE_DETACHED);
        retc |= pthread_create(&i2cThread, &i2cThreadAttr, i2cThreadProc, NULL);
        if (retc < 0)
        {
            UART_PRINT("UDP Server create failed\r\n");
            while (1);
        }

        rtc_init();

        retc = getTime();

        App_CB.timeElapsedSec -= TZ_EST_OFFSET_SECS;

        rtc_set_time((time_t) App_CB.timeElapsedSec);

        rtc_set_alarm(41, 19, RTC_C_ALARMCONDITION_OFF, RTC_C_ALARMCONDITION_OFF);

        while (App_CB.resetApplication == false)
        {
            UART_PRINT("Date: %s\r\n", rtc_get_date());

            sleep(10);
        }

        udpThreadStop = true;
        i2cThreadStop = true;

        rtc_free();

        //wait for server to stop
        pthread_join(udpServerThread, NULL);
        //wait for i2c to stop
        pthread_join(i2cThread, NULL);
    }
}

/*
 * Interrupt handler for RTC Alarm
 */
extern void RTC_C_IRQHandler(uintptr_t Arg)
{
    uint32_t status;

    status = MAP_RTC_C_getEnabledInterruptStatus();
    MAP_RTC_C_clearInterruptFlag(status);

    if (status & RTC_C_CLOCK_READ_READY_INTERRUPT)
    {

    }

    if (status & RTC_C_TIME_EVENT_INTERRUPT)
    {
        UART_PRINT("RTC Int: Minute Passed\r\n");

    }

    if (status & RTC_C_CLOCK_ALARM_INTERRUPT)
    {
        UART_PRINT("RTC Int: Alarm Triggered\r\n");
        //can set next alarm here
        //do i2c stuff here (leds)
        //do spi stuff here (screen)
        //rtc_set_alarm(22, 18, RTC_C_ALARMCONDITION_OFF, RTC_C_ALARMCONDITION_OFF);
    }

}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
