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

/* Standard includes                                                          */
#include <get_time.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>

#include <mqueue.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>

/* Simplelink includes                                                        */
#include <ti/drivers/net/wifi/simplelink.h>

/* Driverlib includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/devices/msp432p4xx/driverlib/aes256.h>

#include <ti/drivers/apps/Button.h>

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
#include "SMO.h"

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************

/* General application functions */
void TimerPeriodicIntHandler(sigval val);
void LedTimerConfigNStart();
void LedTimerDeinitStop();
void RTC_C_IRQHandler(uintptr_t Arg);

static void *udpServerThreadProc(void *pArg);
extern void *peripheralThreadProc(void *arg0);

static void SMO_handleTimeout(void);
static void SMO_okayButtonHandler(Button_Handle handle, Button_EventMask events);

static int SMO_scheduleNextEvent(SMO_Control *Ctrl, uint8_t Hour, uint8_t Min);
static int SMO_handleEvent(SMO_Control *Ctrl);
static void SMO_stopEvent(void);

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

/* AP Security Parameters */
SlWlanSecParams_t SecurityParams = { 0 };

/* Date and Time Parameters */
const uint8_t SNTPserver[30] = "time-c.nist.gov";

//RTC calendar
static volatile RTC_C_Calendar newTime;

//booleans to notify threads to stop
volatile bool udpThreadStop;
volatile bool peripheralThreadStop;

//controller for smart medication organizer
static SMO_Control SMO_Ctrl;
static pthread_mutex_t SMO_Mutex;

//handle for okay button
static Button_Handle okayButtonHandle;

/* AES256 Key
 * pass=6OhmsApart
 * salt=A391991F9584AA9D
 * key=B385BB330C98AA5DFA026E2BE378BA53AFDFAFBEA5055D52C55CCECE6C1E8447
 * iv=A874759E213BD6B8FC3023FE9B1B7E57
 */
static const uint8_t AesKey256[32] = {
    0xB3, 0x85, 0xBB, 0x33, 0x0C, 0x98, 0xAA, 0x5D,
    0xFA, 0x02, 0x6E, 0x2B, 0xE3, 0x78, 0xBA, 0x53,
    0xAF, 0xDF, 0xAF, 0xBE, 0xA5, 0x05, 0x5D, 0x52,
    0xC5, 0x5C, 0xCE, 0xCE, 0x6C, 0x1E, 0x84, 0x47
};

//buffer to hold decrypted SMO_Packet
static uint8_t DataAESdecrypted[16][AES256_BLOCKSIZE];

/*
const uint8_t EncryptedPacket[3][AES256_BLOCKSIZE] = {
    { 0x59, 0x3e, 0x48, 0x1d, 0x72, 0xd8, 0xe0, 0x3f,
      0xfc, 0xce, 0x56, 0xff, 0xba, 0x3a, 0x9c, 0xcd },
    { 0xa5, 0xfd, 0xb8, 0xcb, 0x34, 0x71, 0x17, 0x0a,
      0x83, 0x84, 0x72, 0xb4, 0xf5, 0xee, 0x28, 0xc9 },
    { 0x02, 0xef, 0xea, 0x7f, 0x1d, 0x69, 0xef, 0xaa,
      0x74, 0x87, 0x3d, 0xab, 0x69, 0xa9, 0x6f, 0xad }
};
*/

/****************************************************************************************************************
                   Banner VARIABLES
****************************************************************************************************************/
char lineBreak[]                = "\n\r";

/****************************************************************************************************************
                 Local Functions
****************************************************************************************************************/
int32_t WiFi_IF_Connect(void);

//*****************************************************************************
//
// Push Button Handler1(GPIOSW2). Press push button1 (GPIOSW2) Whenever user
// wants to publish a message. Write message into message queue signaling the
// event publish messages
//
//*****************************************************************************
void pushButtonInterruptHandler2(uint_least8_t index)
{
    /* Disable the SW2 interrupt */
    GPIO_disableInt(Board_BUTTON0);
}

//*****************************************************************************
//
// Push Button Handler2(GPIOSW3). Press push button3 Whenever user wants to
// disconnect from the remote broker. Write message into message queue
// indicating disconnect from broker.
//
//*****************************************************************************
void pushButtonInterruptHandler3(uint_least8_t index)
{
    /* Disable the SW3 interrupt */
    GPIO_disableInt(Board_BUTTON1);
}

//*****************************************************************************
//
// Application Boarders display on UART
//
//*****************************************************************************
void printBorder(char ch, int n)
{
    int        i = 0;

    for(i=0; i<n; i++)    putch(ch);
}

//*****************************************************************************
//
// Periodic Timer Interrupt Handler
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
// Function to configure and start timer to blink the LED while device is
// trying to connect to an AP
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
// Disable the LED blinking Timer as Device is connected to AP
//
//*****************************************************************************
void LedTimerDeinitStop()
{
    /* Disable the LED blinking Timer as Device is connected to AP.           */
    timer_delete(App_CB.timer);
}

//*****************************************************************************
//
// Application startup display on UART
//
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
// This function connect the MQTT device to an AP with the SSID which was
// configured in SSID_NAME definition which can be found in Network_if.h file,
// if the device can't connect to to this AP a request from the user for other
// SSID will appear.
//
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

static int32_t getSNTPTime(int16_t gmt_hr, int16_t gmt_min)
{
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

    if ((dataBuf[0] & 0x7) != 4) /* expect only server response */
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
    }

    return 0;
}

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
    uint8_t DataBuf[16][16];
    int nBytes;

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
        memset(DataBuf, 0, sizeof(DataBuf));

        nBytes = sl_RecvFrom(sd, DataBuf, sizeof(DataBuf), 0,
                             (SlSockAddr_t *)&ClientAddr, (SlSocklen_t*)&ClientSize);
        if (nBytes <= 0)
        {
            //UART_PRINT("Recieve timed out\n\r");
            continue;
        }

        UART_PRINT("Recieved %d bytes\r\n", nBytes);

        /*
         * Expected SMO Data Packet Structure
         * ======================================================
         * 1 byte -- Medication Event packet header type (0x98)
         * ------------------------------------------------------
         * 1 byte -- how many Medication Events (1-6)
         * ======================================================
         * n * (35) bytes -- array of Medication Events
         * ======================================================
         * Medication Event Data Structure
         * ======================================================
         * 1 byte -- hour to take (0-23)
         * ------------------------------------------------------
         * 1 byte -- min to take (0-59)
         * ------------------------------------------------------
         * 1 byte -- how many to take (1-5)
         * ------------------------------------------------------
         * 1 byte -- which compartment (1-6)
         * ------------------------------------------------------
         * 1 byte -- length used by medication information
         * ------------------------------------------------------
         * 30 bytes (chars) -- medication information
         * ======================================================
         */
        int Res = 0;

        //AES-256 decrypt the packet
        int nBlocks = nBytes / AES256_BLOCKSIZE + (nBytes % AES256_BLOCKSIZE != 0);
        nBlocks = nBlocks > 16 ? 16 : nBlocks;
        AES256_setDecipherKey(AES256_BASE, AesKey256, AES256_KEYLENGTH_256BIT);
        int i;
        for (i = 0; i < nBlocks; i++)
        {
            AES256_decryptData(AES256_BASE, DataBuf[i], DataAESdecrypted[i]);
        }

        //check packet type
        if (DataAESdecrypted[0][0] != SMO_PACKET_TYPE_HEADER)
        {
            UART_PRINT("Invalid packet type recieved\r\n");
            continue;
        }

        uint8_t nMeds = DataAESdecrypted[0][1];
        if (nMeds == 0 || nMeds > SMO_PACKET_MAX_MEDS)
        {
            UART_PRINT("Invalid number of meds recieved\r\n");
            continue;
        }

        //copy received data into packet
        SMO_Packet Pkt;
        memcpy(&Pkt, DataAESdecrypted, SMO_PACKET_HEADER_SIZE + nMeds*sizeof(SMO_PacketMed));

        newTime = MAP_RTC_C_getCalendarTime();

        //configure SMO from user data
        pthread_mutex_lock(&SMO_Mutex);
        Res = SMO_Control_configure(&SMO_Ctrl, &Pkt);
        pthread_mutex_unlock(&SMO_Mutex);
        if (Res < 0)
        {
            UART_PRINT("Error configuring SMO\r\n");
            continue;
        }

        //schedule first event
        pthread_mutex_lock(&SMO_Mutex);
        Res = SMO_scheduleNextEvent(&SMO_Ctrl, newTime.hours, newTime.minutes);
        pthread_mutex_unlock(&SMO_Mutex);
        if (Res < 0)
        {
            UART_PRINT("Error scheduling event\r\n");
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

    /* Initialize screen to display load screen */
    Screen_init();

    /* This can be moved */
    peripheralThreadStop = false;
    pthread_t peripheralThread;
    pthread_attr_t peripheralThreadAttr;
    pthread_attr_init(&peripheralThreadAttr);
    retc |= pthread_attr_setstacksize(&peripheralThreadAttr, TASK_STACK_SIZE);
    retc |= pthread_attr_setdetachstate(&peripheralThreadAttr, PTHREAD_CREATE_DETACHED);
    retc |= pthread_create(&peripheralThread, &peripheralThreadAttr, peripheralThreadProc, NULL);
    if (retc < 0)
    {
        UART_PRINT("peripheral Thread create failed\r\n");
        while (1);
    }

    /* Thread vars */
    pthread_t spawn_thread = (pthread_t) NULL;
    pthread_attr_t pAttrs_spawn;
    struct sched_param priParam;

    Button_Params  buttonParams;

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
    /* Init buttons driver */
    Button_init();
    /* Init SPI for communicating between M4 and NWP */
    SPI_init();

    /* Configure the UART                                                     */
    tUartHndl = InitTerm();
    /* remove uart receive from LPDS dependency                               */
    UART_control(tUartHndl, UART_CMD_RXDISABLE, NULL);


    Button_Params_init(&buttonParams);
    okayButtonHandle = Button_open(CONFIG_BUTTON_0, SMO_okayButtonHandler, &buttonParams);

    if (okayButtonHandle == NULL)
    {
        UART_PRINT("Button open failed\r\n");
    }

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
        //while (1); // Insert error handling
    }

    /* Started to allow host to read back MAC address when printing App banner */
    //retc = sl_Start(0, 0, 0);
    if (retc < 0)
    {
        /* Handle Error */
        UART_PRINT("\n sl_Start failed\n");
        //while (1);// Insert error handling
    }

    /* Output device information to the UART terminal                         */
    retc = DisplayAppBanner(APPLICATION_NAME, APPLICATION_VERSION);

    //retc = sl_Stop(SL_STOP_TIMEOUT);
    if (retc < 0)
    {
        /* Handle Error */
        UART_PRINT("\n sl_Stop failed\n");
        //while (1);
    }

    pthread_mutexattr_t Attr;
    pthread_mutexattr_init(&Attr);
    pthread_mutexattr_settype(&Attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&SMO_Mutex, &Attr);

    /* Main application loop */
    while (1)
    {
        UART_PRINT("\n\r\n\r**** Starting main loop ****\n\r");

        App_CB.resetApplication = false;
        App_CB.initState = 0;

        /* Connect to AP */
        UART_PRINT("Using hardcoded profile for connection.\n\r");
        App_CB.apConnectionState = WiFi_IF_Connect();

        RTC_init();
        SMO_Control_init(&SMO_Ctrl);

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

        retc = getTime();

        App_CB.timeElapsedSec -= TZ_EST_OFFSET_SECS;

        RTC_setTime((time_t) App_CB.timeElapsedSec);

        /*
        SMO_Packet Pkt;
        Pkt.PacketType = 98;
        Pkt.nMeds = 3;
        {
            SMO_PacketMed Med1;
            Med1.AlarmHour = 16;
            Med1.AlarmMin = 59;
            Med1.nCmptmt = 1;
            Med1.nPills = 2;
            Med1.Length = 10;
            memcpy(Med1.Payload, "Medicine1", Med1.Length);

            SMO_PacketMed Med2;
            Med2.AlarmHour = 17;
            Med2.AlarmMin = 20;
            Med2.nCmptmt = 2;
            Med2.nPills = 1;
            Med2.Length = 10;
            memcpy(Med2.Payload, "Medicine2", Med2.Length);

            SMO_PacketMed Med3;
            Med3.AlarmHour = 17;
            Med3.AlarmMin = 18;
            Med3.nCmptmt = 3;
            Med3.nPills = 1;
            Med3.Length = 10;
            memcpy(Med3.Payload, "Medicine3", Med3.Length);

            Pkt.Meds[0] = Med1;
            Pkt.Meds[1] = Med2;
            Pkt.Meds[2] = Med3;
        }

        pthread_mutex_lock(&SMO_Mutex);
        retc = SMO_Control_configure(&SMO_Ctrl, &Pkt);
        pthread_mutex_unlock(&SMO_Mutex);
        if (retc < 0)
        {
            UART_PRINT("Error configuring SMO\r\n");
        }

        newTime = MAP_RTC_C_getCalendarTime();
        pthread_mutex_lock(&SMO_Mutex);
        retc = SMO_scheduleNextEvent(&SMO_Ctrl, newTime.hours, newTime.minutes);
        pthread_mutex_unlock(&SMO_Mutex);
        if (retc < 0)
        {
            UART_PRINT("Error scheduling event\r\n");
        }
        */

        while (App_CB.resetApplication == false)
        {
            char *Date = RTC_getDate();

            UART_PRINT("Date: %s\r\n", Date);

            /*
            //AES data must be multiple of 16
            int DateLen = strlen(Date);
            int i;

            uint8_t DataBuf[DATE_BLOCKS][AES256_BLOCKSIZE];
            uint8_t EncryptedBuf[DATE_BLOCKS][AES256_BLOCKSIZE];
            uint8_t DecryptedBuf[DATE_BLOCKS][AES256_BLOCKSIZE];
            memset(DataBuf, 0, sizeof(DataBuf));
            int DataLen = (DateLen > sizeof(DataBuf)) ? sizeof(DataBuf) : DateLen;
            memcpy(DataBuf, Date, DataLen);

            UART_PRINT("AES Data: %s\r\n", (char *) DataBuf);

            AES256_setCipherKey(AES256_BASE, AesKey256, AES256_KEYLENGTH_256BIT);
            for (i = 0; i < DATE_BLOCKS; i++)
            {
                AES256_encryptData(AES256_BASE, DataBuf[i], EncryptedBuf[i]);
            }

            UART_PRINT("Encrypted Data: ");
            for (i = 0; i < sizeof(EncryptedBuf); i++)
            {
                UART_PRINT("%x", (char) EncryptedBuf[0][i]);
            }
            UART_PRINT("\r\n");

            AES256_setDecipherKey(AES256_BASE, AesKey256, AES256_KEYLENGTH_256BIT);
            for (i = 0; i < DATE_BLOCKS; i++)
            {
                AES256_decryptData(AES256_BASE, EncryptedBuf[i], DecryptedBuf[i]);
            }

            UART_PRINT("Decrypted Data: %s\r\n", (char *) DecryptedBuf);
            */

            /*
            uint8_t DecryptedBuf[3][AES256_BLOCKSIZE];

            AES256_setDecipherKey(AES256_BASE, AesKey256, AES256_KEYLENGTH_256BIT);
            int i;
            for (i = 0; i < 3; i++)
            {
                AES256_decryptData(AES256_BASE, EncryptedPacket[i], DecryptedBuf[i]);
            }

            UART_PRINT("Decrypted Data: ");
            for (i = 0; i < sizeof(DecryptedBuf); i++)
            {
                UART_PRINT("%x", DecryptedBuf[0][i]);
            }
            UART_PRINT("\r\n");
            */

            sleep(10);
        }

        udpThreadStop = true;
        peripheralThreadStop = true;

        //wait for server to stop
        pthread_join(udpServerThread, NULL);
        //wait for peripherals to stop
        /*
        pthread_join(peripheralThread, NULL);
        */
    }

    //SMO_Control_free(&SMO_Ctrl);
    //RTC_free();
}

/*
 * Interrupt handler for RTC Alarm
 */
extern void RTC_C_IRQHandler(uintptr_t Arg)
{
    uint32_t status;

    status = MAP_RTC_C_getEnabledInterruptStatus();
    MAP_RTC_C_clearInterruptFlag(status);

    newTime = MAP_RTC_C_getCalendarTime();

    if (status & RTC_C_CLOCK_READ_READY_INTERRUPT)
    {

    }

    if (status & RTC_C_TIME_EVENT_INTERRUPT)
    {
        UART_PRINT("RTC Int: Minute Passed\r\n");

        //update screen time display every minute
        //Screen_updateTime(newTime.minutes, newTime.hours);

        pthread_mutex_lock(&SMO_Mutex);
        if (SMO_Ctrl.Timer.Timing)
        {
            SMO_Ctrl.Timer.Count++;
            if (SMO_Ctrl.Timer.Count == SMO_EVENT_TIMEOUT_MINS)
            {
                SMO_Timer_stop(&SMO_Ctrl.Timer);
                SMO_handleTimeout();
            }
        }
        pthread_mutex_unlock(&SMO_Mutex);
    }

    if (status & RTC_C_CLOCK_ALARM_INTERRUPT)
    {
        int Res = 0;
        UART_PRINT("RTC Int: Alarm Triggered\r\n");

        pthread_mutex_lock(&SMO_Mutex);
        if (SMO_Ctrl.Timer.Timing)
        {
            SMO_Timer_stop(&SMO_Ctrl.Timer);
            SMO_stopEvent();
        }
        Res = SMO_handleEvent(&SMO_Ctrl);
        pthread_mutex_unlock(&SMO_Mutex);
        if (Res < 0)
        {
            UART_PRINT("Error handling event\r\n");
        }
        else
        {
            pthread_mutex_lock(&SMO_Mutex);
            SMO_Timer_start(&SMO_Ctrl.Timer);
            pthread_mutex_unlock(&SMO_Mutex);
        }

        pthread_mutex_lock(&SMO_Mutex);
        Res = SMO_scheduleNextEvent(&SMO_Ctrl, newTime.hours, newTime.minutes);
        pthread_mutex_unlock(&SMO_Mutex);
        if (Res < 0)
        {
            UART_PRINT("Error scheduling next event\r\n");
        }
    }
}

static void SMO_okayButtonHandler(Button_Handle handle, Button_EventMask events)
{
    if (Button_EV_CLICKED == (events & Button_EV_CLICKED))
    {
        UART_PRINT("Okay button clicked\r\n");
        pthread_mutex_lock(&SMO_Mutex);
        //user presses button to acknowledge event
        if (SMO_Ctrl.Timer.Timing)
        {
            SMO_Timer_stop(&SMO_Ctrl.Timer);
            SMO_stopEvent();
        }
        pthread_mutex_lock(&SMO_Mutex);
    }
}

static void SMO_handleTimeout(void)
{
    UART_PRINT("Event timer timed out\r\n");
    //when timer goes off, stop peripherals
    SMO_stopEvent();
}

static void SMO_stopEvent(void)
{
    UART_PRINT("Stopping SMO event\r\n");
    //stop LEDs
    //LED_allOff();

    //remove med info from screen
    //Screen_removeMedInfo();

    //turn off speaker
    //Speaker_off();
}

static int SMO_scheduleNextEvent(SMO_Control *Ctrl, uint8_t Hour, uint8_t Min)
{
    int Res = 0;

    SMO_Event *NextEvent = SMO_Control_nextEvent(Ctrl, Hour, Min);
    if (NextEvent == NULL)
    {
        UART_PRINT("No event available\r\n");
        Res = -EINVAL;
        goto Error;
    }

    UART_PRINT("Scheduling next event for %02d:%02d\r\n", NextEvent->AlarmHour, NextEvent->AlarmMin);
    //set an alarm for the next event
    RTC_setAlarm(NextEvent->AlarmMin, NextEvent->AlarmHour, RTC_C_ALARMCONDITION_OFF, RTC_C_ALARMCONDITION_OFF);

    //save the event so we know what to do when alarm triggers
    Ctrl->CurrentEvent = NextEvent;

Error:
    return Res;
}

static int SMO_handleEvent(SMO_Control *Ctrl)
{
    int Res = 0;
    char ScreenStr[255];
    char *BeginStr = &ScreenStr[0], *CurrentStr = &ScreenStr[0], *MedStr = NULL;
    uint8_t Tmp, Index, Compartments = Ctrl->CurrentEvent->Compartments;

    if (Ctrl->CurrentEvent == NULL)
    {
        UART_PRINT("No event ready\r\n");
        Res = -EINVAL;
        goto Error;
    }

    UART_PRINT("Starting SMO event\r\n");

    while (Compartments != 0ULL)
    {
        Tmp = Compartments & -Compartments;
        Index = 31 - __CLZ(Tmp);

        //handle LEDs
        UART_PRINT("Lighting LED %d\r\n", Index);
        //LED_on(Index);

        //add med info to screen string
        MedStr = SMO_Control_getMedStr(Ctrl, Index);
        if (MedStr != NULL)
        {
            CurrentStr += sprintf(CurrentStr, "%s, Cmptmt: %d, Pills: %d\r\n",
                                  MedStr, Index, Ctrl->CurrentEvent->nPills[Index]);
        }

        Compartments ^= Tmp;
    }

    //display med info
    UART_PRINT("Displaying on screen:\r\n%s", BeginStr);
    //Screen_printMedInfo(ScreenStr);

    //turn on speaker
    UART_PRINT("Turning speaker on\r\n");
    //Speaker_on();

Error:
    return Res;
}
