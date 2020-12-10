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

//*****************************************************************************
// network_if.c
//
// Networking interface functions for CC3220 device
//
//*****************************************************************************
//*****************************************************************************
//
//! \addtogroup common_interface
//! @{
//
//*****************************************************************************
/* Standard includes                                                          */
#include <get_time.h>
#include <string.h>
#include <stdlib.h>

/* Kernel (Non OS/Free-RTOS/TI-RTOS) includes                                 */
#include <pthread.h>
#include <mqueue.h>
#include <unistd.h>

/* Simplelink includes                                                        */
#include <ti/drivers/net/wifi/simplelink.h>

/* Common interface includes                                                  */
#include "network_if.h"
#include "uart_term.h"

/* Application includes********************************************************************
/                          LOCAL DEFINES
*****************************************************************************/

/* Network App specific status/error codes which are                          */
/* used only in this file.                                                    */
typedef enum
{
    /* Choosing this number to avoid overlap with host-driver's error codes.  */
    DEVICE_NOT_IN_STATION_MODE = -0x7F0,
    DEVICE_NOT_IN_AP_MODE = DEVICE_NOT_IN_STATION_MODE - 1,
    DEVICE_NOT_IN_P2P_MODE = DEVICE_NOT_IN_AP_MODE - 1,

    STATUS_CODE_MAX = -0xBB8
} e_NetAppStatusCodes;

/****************************************************************************************************************
                   GLOBAL VARIABLES
****************************************************************************************************************/
extern Application_CB App_CB;

///* Station IP address                                                         */
//unsigned long g_ulStaIp = 0;
///* Network Gateway IP address                                                 */
//unsigned long g_ulGatewayIP = 0;
///* Connection SSID                                                            */
//unsigned char g_ucConnectionSSID[SL_WLAN_SSID_MAX_LENGTH + 1];
///* Connection BSSID                                                           */
//unsigned char g_ucConnectionBSSID[SL_WLAN_BSSID_LENGTH ];
///* SimpleLink Status                                                          */
//volatile unsigned long g_ulStatus = 0;
///* Connection time delay index                                                */
//volatile unsigned short g_usConnectIndex;

//*****************************************************************************
//
//! This function initializes the application variables
//!
//! \param  None
//!
//! \return 0 on success, negative error-code on error
//
//*****************************************************************************
void InitializeAppVariables(void)
{
    App_CB.status = 0;
    App_CB.staIP = 0;
    App_CB.gatewayIP = 0;

    memset(App_CB.connectionSSID, 0, sizeof(App_CB.connectionSSID));
    memset(App_CB.connectionBSSID, 0, sizeof(App_CB.connectionBSSID));
}

//*****************************************************************************
//
//! The function initializes a CC3220 device and triggers it to start operation
//!
//! \param[in]  uiMode  - device mode in which device will be configured
//!
//! \return 0 : success, -ve : failure
//
//*****************************************************************************
long Network_IF_InitDriver(uint32_t uiMode)
{
    long lRetVal = -1;

    /* Reset CC3220 Network State Machine                                     */
    InitializeAppVariables();

    /* Following function configure the device to default state by cleaning   */
    /* the persistent settings stored in NVMEM (viz. connection profiles      */
    /* & policies, power policy etc) Applications may choose to skip this     */
    /* step if the developer is sure that the device is in its default state  */
    /* at start of application. Note that all profiles and persistent         */
    /* settings that were done on the device will be lost.                    */
    lRetVal = sl_Start(NULL, NULL, NULL);

    if (lRetVal < 0)
    {
        UART_PRINT("Failed to start the device \n\r");
        LOOP_FOREVER();
    }

    switch (lRetVal)
    {
        case ROLE_STA:
            UART_PRINT("Device came up in Station mode\n\r");
            break;
        case ROLE_AP:
            UART_PRINT("Device came up in Access-Point mode\n\r");
            break;
        case ROLE_P2P:
            UART_PRINT("Device came up in P2P mode\n\r");
            break;
        default:
            UART_PRINT("Error:unknown mode\n\r");
            break;
    }

    if (uiMode != lRetVal)
    {
        UART_PRINT("Switching Networking mode on application request\n\r");

        /* Switch to AP role and restart                                      */
        lRetVal = sl_WlanSetMode(uiMode);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(SL_STOP_TIMEOUT);
        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        if (lRetVal == uiMode)
        {
            switch (lRetVal)
            {
                case ROLE_STA:
                    UART_PRINT("Device came up in Station mode\n\r");
                    break;
                case ROLE_AP:
                    UART_PRINT("Device came up in Access-Point mode\n\r");
                    /* If the device is in AP mode, we need to wait for this  */
                    /* event before doing anything.                           */
                    while (!IS_IP_ACQUIRED(App_CB.status))
                    {
                        usleep(1000);
                    }
                    break;
                case ROLE_P2P:
                    UART_PRINT("Device came up in P2P mode\n\r");
                    break;
                default:
                    UART_PRINT("Error:unknown mode\n\r");
                    break;
            }
        }
        else
        {
            UART_PRINT("could not configure correct networking mode\n\r");
            LOOP_FOREVER();
        }
    }
    else
    {
        if (lRetVal == ROLE_AP)
        {
            while (!IS_IP_ACQUIRED(App_CB.status))
            {
                usleep(1000);
            }
        }
    }
    return 0;
}

//*****************************************************************************
//
//! The function de-initializes a CC3220 device
//!
//! \param  None
//!
//! \return On success, zero is returned. On error, other
//
//*****************************************************************************
long Network_IF_DeInitDriver(void)
{
    long lRetVal = -1;

    UART_PRINT("SL Disconnect...\n\r");

    /* Disconnect from the AP                                                 */
    lRetVal = Network_IF_DisconnectFromAP();

    /* Stop the simplelink host                                               */
    sl_Stop(SL_STOP_TIMEOUT);

    /* Reset the state to uninitialized                                       */
    Network_IF_ResetMCUStateMachine();
    return lRetVal;
}

//*****************************************************************************
//
//! Connect to an Access Point using the specified SSID
//!
//! \param[in]  pcSsid          - is a string of the AP's SSID
//! \param[in]  SecurityParams  - is Security parameter for AP
//!
//! \return On success, zero is returned. On error, -ve value is returned
//
//*****************************************************************************
long Network_IF_ConnectAP(char *pcSsid, SlWlanSecParams_t SecurityParams)
{
//    char acCmdStore[128];
//    unsigned short usConnTimeout;
//    unsigned char ucRecvdAPDetails;
    long lRetVal;
    unsigned long ulIP = 0;
    unsigned long ulSubMask = 0;
    unsigned long ulDefGateway = 0;
    unsigned long ulDns = 0;

    App_CB.numConnAttempts = 0;

    /* Disconnect from the AP                                                 */
    Network_IF_DisconnectFromAP();

    CLR_STATUS_BIT(App_CB.status, AppStatusBits_Connection);
    CLR_STATUS_BIT(App_CB.status, AppStatusBits_IpAcquired);

    /* Continue only if SSID is not empty                                     */
    if (pcSsid != NULL)
    {
        /* This triggers the CC3220 to connect to a specific AP.              */
        lRetVal = sl_WlanConnect((signed char *) pcSsid, strlen((const char *) pcSsid),
        NULL, &SecurityParams, NULL);
        ASSERT_ON_ERROR(lRetVal);

        /* Wait for ~10 sec to check if connection to desired AP succeeds      */
        while (App_CB.numConnAttempts < 10)
        {
            sleep(1);

            if (IS_CONNECTED(App_CB.status) && IS_IP_ACQUIRED(App_CB.status))
            {
                break;
            }
            App_CB.numConnAttempts++;
        }
    }
    else
    {
        UART_PRINT("Empty SSID, Could not connect\n\r");
        return -1;
    }

    //[TODO: Provide option to read AP info via UART like in MQTT example]
    if(!(IS_CONNECTED(App_CB.status)) || !(IS_IP_ACQUIRED(App_CB.status)))
    {
        UART_PRINT("\n\rFailed to connect to provided AP.\n\r");
        return -1;
    }
    else
    {
        /* Put message on UART                                                    */
        UART_PRINT("\n\rDevice has connected to %s\n\r", pcSsid);

        /* Get IP address                                                         */
        lRetVal = Network_IF_IpConfigGet(&ulIP, &ulSubMask, &ulDefGateway, &ulDns);
        ASSERT_ON_ERROR(lRetVal);

        //save the device ip address
        App_CB.staIP = ulIP;

        /* Send the information                                                   */
        UART_PRINT("Device IP Address is %d.%d.%d.%d \n\r\n\r", SL_IPV4_BYTE(ulIP, 3), SL_IPV4_BYTE(ulIP, 2), SL_IPV4_BYTE(ulIP, 1), SL_IPV4_BYTE(ulIP, 0));
        return 0;
    }
}

//*****************************************************************************
//
//! Disconnects from an Access Point
//!
//! \param  none
//!
//! \return 0 disconnected done, other already disconnected
//
//*****************************************************************************
long Network_IF_DisconnectFromAP(void)
{
    long lRetVal = 0;

    if (IS_CONNECTED(App_CB.status))
    {
        lRetVal = sl_WlanDisconnect();
        if (0 == lRetVal)
        {
            while (IS_CONNECTED(App_CB.status))
            {
                usleep(1000);
            }
            return lRetVal;
        }
        else
        {
            return lRetVal;
        }
    }
    else
    {
        return lRetVal;
    }
}

//*****************************************************************************
//
//! Get the IP Address of the device.
//!
//! \param[in]  pulIP               - IP Address of Device
//! \param[in]  pulSubnetMask       - Subnetmask of Device
//! \param[in]  pulDefaultGateway   - Default Gateway value
//! \param[in]  pulDNSServer        - DNS Server
//!
//! \return On success, zero is returned. On error, -1 is returned
//
//*****************************************************************************
long Network_IF_IpConfigGet(unsigned long *pulIP, unsigned long *pulSubnetMask, unsigned long *pulDefaultGateway, unsigned long *pulDNSServer)
{
    unsigned short usDHCP = 0;
    long lRetVal = -1;
    unsigned short len = sizeof(SlNetCfgIpV4Args_t);
    SlNetCfgIpV4Args_t ipV4 = { 0 };

    /* get network configuration                                              */
    lRetVal = sl_NetCfgGet(SL_NETCFG_IPV4_STA_ADDR_MODE, &usDHCP, &len, (unsigned char *) &ipV4);
    ASSERT_ON_ERROR(lRetVal);

    *pulIP = ipV4.Ip;
    *pulSubnetMask = ipV4.IpMask;
    *pulDefaultGateway = ipV4.IpGateway;
    *pulDefaultGateway = ipV4.IpDnsServer;

    return lRetVal;
}

//*****************************************************************************
//
//!  This function obtains the server IP address using a DNS lookup
//!
//! \param[in]  pcHostName        The server hostname
//! \param[out] pDestinationIP    This parameter is filled with host IP address
//!
//! \return On success, +ve value is returned. On error, -ve value is returned
//
//*****************************************************************************
long Network_IF_GetHostIP(char* pcHostName, unsigned long * pDestinationIP)
{
    long lStatus = 0;

    lStatus = sl_NetAppDnsGetHostByName((signed char *) pcHostName, strlen(pcHostName), pDestinationIP, SL_AF_INET);
    ASSERT_ON_ERROR(lStatus);

    UART_PRINT("Get Host IP succeeded.\n\rHost: %s IP: %d.%d.%d.%d \n\r\n\r", pcHostName, SL_IPV4_BYTE(*pDestinationIP, 3), SL_IPV4_BYTE(*pDestinationIP, 2), SL_IPV4_BYTE(*pDestinationIP, 1),
                SL_IPV4_BYTE(*pDestinationIP, 0));

    return lStatus;
}

//*****************************************************************************
//
//! Reset state from the state machine
//!
//! \param  None
//!
//! \return none
//!
//*****************************************************************************
void Network_IF_ResetMCUStateMachine()
{
    App_CB.status = 0;
}

//*****************************************************************************
//
//! Return the current state bits
//!
//! \param  None
//!
//! \return none
//!
//
//*****************************************************************************
unsigned long Network_IF_CurrentMCUState()
{
    return App_CB.status;
}

//*****************************************************************************
//
//! Sets a state from the state machine
//!
//! \param[in]  cStat   - Status of State Machine defined in e_StatusBits
//!
//! \return none
//!
//*****************************************************************************
void Network_IF_SetMCUMachineState(char cStat)
{
    SET_STATUS_BIT(App_CB.status, cStat);
}

//*****************************************************************************
//
//! Unsets a state from the state machine
//!
//! \param[in]  cStat   - Status of State Machine defined in e_StatusBits
//!
//! \return none
//!
//*****************************************************************************
void Network_IF_UnsetMCUMachineState(char cStat)
{
    CLR_STATUS_BIT(App_CB.status, cStat);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
