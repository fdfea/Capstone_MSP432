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
#include <mqueue.h>

/* TI-DRIVERS Header files */
#include <ti/drivers/net/wifi/simplelink.h>

/* Application Name and Version*/
#define APPLICATION_NAME        "Get Time Wifi Demo"
#define APPLICATION_VERSION     "0.1.0"

#define PROVISIONING        (0)

#define PUSH_BUTTONS        (0)

/* Priority of Wi-Fi interface in SimpleLink NetSock Driver */
#define SLNET_IF_WIFI_PRIO       (5)

#define SPAWN_TASK_PRIORITY     (9)
#define TASK_STACK_SIZE         (2048)

/* CC3220 Specific */
/* OCP register used to store device role when coming out of hibernate */
/* if ocpRegOffset is set -> AP role, otherwise -> STATION role */
#define OCP_REGISTER_INDEX      (0)
#define OCP_REGISTER_OFFSET     (10)

#define SSID_LEN_MAX            (32)
#define BSSID_LEN_MAX           (6)

#define SL_STOP_TIMEOUT         (200)

/*  Date and Time */
#define SMALL_BUF           32
#define TIME2013            3565987200       /* 43 years + 11 days of leap years */
#define YEAR2013            2013
#define SEC_IN_MIN          60
#define SEC_IN_HOUR         3600
#define SEC_IN_DAY          86400
#define GMT_TIME_ZONE_HR    1
#define GMT_TIME_ZONE_MIN   00

#define SECONDS_BEFORE_UNIX_EPOCH   2208988800
#define TZ_EST_OFFSET_SECS          14400

/* Weather */
#define MAX_SEND_BUF_SIZE   512
#define MAX_SEND_RCV_SIZE   1000

/* Loop forever, user can change it as per application's requirement          */
#define LOOP_FOREVER() \
            {\
                while(1); \
            }
/* check the error code and handle it                                         */
#define ASSERT_ON_ERROR(error_code)\
            {\
                 if(error_code < 0) \
                   {\
                        ERR_PRINT(error_code);\
                        return error_code;\
                 }\
            }

/* Basic Macros for Manipulating Status Variables                            */
#define CLR_STATUS_BIT_ALL(status_variable)  (status_variable = 0)
#define SET_STATUS_BIT(status_variable, bit) (status_variable |= (1<<(bit)))
#define CLR_STATUS_BIT(status_variable, bit) (status_variable &= ~(1<<(bit)))
#define CLR_STATUS_BIT_ALL(status_variable)  (status_variable = 0)
#define GET_STATUS_BIT(status_variable, bit) (0 != (status_variable & (1<<(bit))))

/* Network Specific Status Bit Macros                                        */
#define IS_NW_PROCSR_ON(status_variable)    \
                GET_STATUS_BIT(status_variable, AppStatusBits_NwpInit)

#define IS_CONNECTED(status_variable)       \
                GET_STATUS_BIT(status_variable, AppStatusBits_Connection)

#define IS_IP_LEASED(status_variable)       \
                GET_STATUS_BIT(status_variable, AppStatusBits_IpLeased)

#define IS_IP_ACQUIRED(status_variable)     \
                GET_STATUS_BIT(status_variable, AppStatusBits_IpAcquired)

#define IS_IPV6L_ACQUIRED(status_variable)  \
                GET_STATUS_BIT(status_variable, AppStatusBits_Ipv6lAcquired)

#define IS_IPV6G_ACQUIRED(status_variable)  \
                GET_STATUS_BIT(status_variable, AppStatusBits_Ipv6gAcquired)

#define IS_SMART_CFG_START(status_variable) \
                GET_STATUS_BIT(status_variable, AppStatusBits_SmartconfigStart)

#define IS_CONNECT_FAILED(status_variable)  \
                GET_STATUS_BIT(status_variable, AppStatusBits_Connection)

#define IS_PING_DONE(status_variable)       \
                GET_STATUS_BIT(status_variable, AppStatusBits_PingDone)

#define IS_AUTHENTICATION_FAILED(status_variable)   \
            GET_STATUS_BIT(status_variable, AppStatusBits_AuthenticationFailed)

/* Application specific status/error codes */
typedef enum
{
    /* Choosing -0x7D0 to avoid overlap w/ host-driver's error codes */
    AppStatusCodes_LanConnectionFailed = -0x7D0,
    AppStatusCodes_InternetConnectionFailed = AppStatusCodes_LanConnectionFailed - 1,
    AppStatusCodes_DeviceNotInStationMode = AppStatusCodes_InternetConnectionFailed - 1,
    AppStatusCodes_StatusCodeMax = -0xBB8
}AppStatusCodes;

typedef enum
{
    AppStatusBits_NwpInit = 0,                  /* If this bit is set: Network Processor is powered up */
    AppStatusBits_Connection = 1,               /* If this bit is set: the device is connected to the AP or client is connected to device (AP) */
    AppStatusBits_IpLeased = 2,                 /* If this bit is set: the device has leased IP to any connected client */
    AppStatusBits_IpAcquired = 3,               /* If this bit is set: the device has acquired an IP */
    AppStatusBits_SmartconfigStart = 4,         /* If this bit is set: the SmartConfiguration process is started from SmartConfig app */
    AppStatusBits_P2pDevFound = 5,              /* If this bit is set: the device (P2P mode) found any p2p-device in scan */
    AppStatusBits_P2pReqReceived = 6,           /* If this bit is set: the device (P2P mode) found any p2p-negotiation request */
    AppStatusBits_ConnectionFailed = 7,         /* If this bit is set: the device(P2P mode) connection to client(or reverse way) is failed */
    AppStatusBits_PingDone = 8,                 /* If this bit is set: the device has completed the ping operation */
    AppStatusBits_Ipv6lAcquired = 9,            /* If this bit is set: the device has acquired an IPv6 address */
    AppStatusBits_Ipv6gAcquired = 10,           /* If this bit is set: the device has acquired an IPv6 address */
    AppStatusBits_AuthenticationFailed = 11,
    AppStatusBits_ResetRequired = 12,
}AppStatusBits;

/* Control block definition */
typedef struct Application_ControlBlock_t
{
    /* Network IF Vars */
    uint32_t  status;                                   /* SimpleLink Status */
    uint32_t  gatewayIP;                                /* Network Gateway IP address */
    uint32_t  staIP;                                    /* Device station IP address */
    uint8_t   connectionSSID[SSID_LEN_MAX+1];           /* Connection SSID */
    uint8_t   ssidLen;                                  /* Connection SSID */
    uint8_t   connectionBSSID[BSSID_LEN_MAX];           /* Connection BSSID */
    uint8_t   numConnAttempts;                          /* Number of times device attempted to connect to AP */

    /* Application Vars */
    bool      resetApplication;                         /* Represents wether there is a pending application reset request */
    int32_t   apConnectionState;                        /* Connection state: (0) - connected, (negative) - disconnected               */
    uint32_t  initState;                                /* Represents what initialization state the MQTT interface is in */

    uint8_t  weatherRecvbuff[MAX_SEND_RCV_SIZE];
    uint8_t  weatherSendBuff[MAX_SEND_BUF_SIZE];
    uint8_t  weatherHostName[SMALL_BUF];
    uint8_t  weatherCityName[SMALL_BUF];
    uint32_t weatherDestinationIP;
    int16_t  weatherSockID;

    uint32_t timeElapsedSec;
    uint32_t timeUGeneralVar;
    uint32_t timeUGeneralVar1;
    uint8_t timeHostName[SMALL_BUF];
    uint16_t timeCCLen;
    uint32_t timeDestinationIP;
    int32_t timeSockID;
    int32_t timeSGeneralVar;
    uint8_t time[30];
    uint8_t *timeCCPtr;

    /* General */
    uint16_t    timerInts;
    timer_t     timer;

    /* Security */
    uint8_t     lockUDID[16];


}Application_CB;

/*
  Type: Tmp_Day
*/
typedef struct {
    uint8_t TmpDate[14];
    uint8_t TmpCurrent[10];
    uint8_t TmpMin[10];
    uint8_t TmpMax[10];
    uint8_t WeatherSymbol[30];
} Tmp_Day_t;

typedef struct {
    uint8_t    ssid[100];
    uint8_t    passKey[100];
    uint8_t    secParamType;
    uint8_t    demoNetworkConfigured;
    uint8_t    city[100];
    int8_t     gmtTimeZoneHR;
    uint8_t    demoCityConfigured;
} demo_Config_t;

//****************************************************************************
//                      FUNCTION PROTOTYPES
//****************************************************************************
void mcuReboot(void);
