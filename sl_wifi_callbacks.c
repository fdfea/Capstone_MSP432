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

/* Simplelink includes                                                        */
#include <get_time.h>
#include <ti/drivers/net/wifi/simplelink.h>

/* Application includes                                                       */
#include "uart_term.h"

/****************************************************************************************************************
                   GLOBAL VARIABLES
****************************************************************************************************************/
/* Overall application control block */
extern Application_CB App_CB;

/****************************************************************************************************************
   SimpleLink Asynchronous Event Handlers -- Start
****************************************************************************************************************/
void SimpleLinkHttpServerEventHandler(SlNetAppHttpServerEvent_t *pSlHttpServerEvent, SlNetAppHttpServerResponse_t *pSlHttpServerResponse)
{
    /* Unused in this application                                             */
}

#if (HTTP_SERVER == 0)
void SimpleLinkNetAppRequestEventHandler(SlNetAppRequest_t *pNetAppRequest, SlNetAppResponse_t *pNetAppResponse)
{
    /* Unused in this application                                             */
}
#endif

void SimpleLinkNetAppRequestMemFreeEventHandler(uint8_t *buffer)
{
    /* Unused in this application                                             */
}

//*****************************************************************************
//!
//! On Successful completion of Wlan Connect, This function triggers connection
//! status to be set.
//!
//! \param[in]  pSlWlanEvent    - pointer indicating Event type
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pSlWlanEvent)
{
    SlWlanEventDisconnect_t* pEventData = NULL;

    switch (pSlWlanEvent->Id)
    {
        case SL_WLAN_EVENT_CONNECT:
        {
            SET_STATUS_BIT(App_CB.status, AppStatusBits_Connection);

            memcpy(App_CB.connectionSSID, pSlWlanEvent->Data.Connect.SsidName, pSlWlanEvent->Data.Connect.SsidLen);
            memcpy(App_CB.connectionBSSID, pSlWlanEvent->Data.Connect.Bssid, SL_WLAN_BSSID_LENGTH);

            App_CB.ssidLen = pSlWlanEvent->Data.Connect.SsidLen;

            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s , BSSID: "
                    "%x:%x:%x:%x:%x:%x\n\r", App_CB.connectionSSID,
                     App_CB.connectionBSSID[0], App_CB.connectionBSSID[1],
                     App_CB.connectionBSSID[2], App_CB.connectionBSSID[3],
                     App_CB.connectionBSSID[4], App_CB.connectionBSSID[5]);

        }
        break;

        case SL_WLAN_EVENT_DISCONNECT:
        {
            CLR_STATUS_BIT(App_CB.status, AppStatusBits_Connection);
            CLR_STATUS_BIT(App_CB.status, AppStatusBits_IpAcquired);

            App_CB.numConnAttempts = 0;

            pEventData = &pSlWlanEvent->Data.Disconnect;

            /* If the user has initiated 'Disconnect' request, 'reason_code'  */
            /* is SL_WLAN_DISCONNECT_USER_INITIATED                           */
            if (SL_WLAN_DISCONNECT_USER_INITIATED == pEventData->ReasonCode)
            {
                UART_PRINT("Device disconnected from the AP on application's "
                        "request \n\r");
            }
            else
            {
                UART_PRINT("Device disconnected from the AP on an ERROR..!! \n\r");
            }
        }
        break;

        case SL_WLAN_EVENT_STA_ADDED:
        {
            /* when device is in AP mode and any client connects to it.       */
            SET_STATUS_BIT(App_CB.status, AppStatusBits_Connection);
        }
        break;

        case SL_WLAN_EVENT_STA_REMOVED:
        {
            /* when device is in AP mode and any client disconnects from it.  */
            CLR_STATUS_BIT(App_CB.status, AppStatusBits_Connection);
            CLR_STATUS_BIT(App_CB.status, AppStatusBits_IpLeased);
        }
        break;

        case SL_WLAN_EVENT_PROVISIONING_PROFILE_ADDED:
            UART_PRINT("[WLAN EVENT] Profile Added\r\n");
        break;

        case SL_WLAN_EVENT_PROVISIONING_STATUS:
        {
            uint16_t status = pSlWlanEvent->Data.ProvisioningStatus.ProvisioningStatus;

            switch (status)
            {
            case SL_WLAN_PROVISIONING_GENERAL_ERROR:
            case SL_WLAN_PROVISIONING_ERROR_ABORT:
            {
                UART_PRINT("[WLAN EVENT] Provisioning Error status=%d\r\n", status);
            }
                break;
            case SL_WLAN_PROVISIONING_ERROR_ABORT_INVALID_PARAM:
            case SL_WLAN_PROVISIONING_ERROR_ABORT_HTTP_SERVER_DISABLED:
            case SL_WLAN_PROVISIONING_ERROR_ABORT_PROFILE_LIST_FULL:
            {
                UART_PRINT("[WLAN EVENT] Provisioning Error status=%d\r\n", status);
            }
                break;
            case SL_WLAN_PROVISIONING_ERROR_ABORT_PROVISIONING_ALREADY_STARTED:
            {
                UART_PRINT("[WLAN EVENT] Provisioning already started");
            }
                break;

            case SL_WLAN_PROVISIONING_CONFIRMATION_STATUS_FAIL_NETWORK_NOT_FOUND:
            {
                UART_PRINT("[WLAN EVENT] Confirmation fail: network not found\r\n");
            }
                break;

            case SL_WLAN_PROVISIONING_CONFIRMATION_STATUS_FAIL_CONNECTION_FAILED:
            {
                UART_PRINT("[WLAN EVENT] Confirmation fail: Connection failed\r\n");
            }
                break;

            case SL_WLAN_PROVISIONING_CONFIRMATION_STATUS_CONNECTION_SUCCESS_IP_NOT_ACQUIRED:
            {
                UART_PRINT(
                        "[WLAN EVENT] Confirmation fail: IP address not acquired\r\n");
            }
                break;

            case SL_WLAN_PROVISIONING_CONFIRMATION_STATUS_SUCCESS_FEEDBACK_FAILED:
            {
                UART_PRINT(
                        "[WLAN EVENT] Connection Success (feedback to Smartphone app failed)\r\n");
            }
                break;

            case SL_WLAN_PROVISIONING_CONFIRMATION_STATUS_SUCCESS:
            {
                UART_PRINT("[WLAN EVENT] Confirmation Success!\r\n");
            }
                break;

            case SL_WLAN_PROVISIONING_AUTO_STARTED:
            {
                UART_PRINT("[WLAN EVENT] Auto-Provisioning Started\r\n");
                /* stop auto provisioning - may trigger in case of returning to default */
            }
                break;

            case SL_WLAN_PROVISIONING_STOPPED:
            {
                UART_PRINT("[WLAN EVENT] Provisioning stopped\r\n");
                if (ROLE_STA == pSlWlanEvent->Data.ProvisioningStatus.Role)
                {
                    UART_PRINT(" [WLAN EVENT] - WLAN Connection Status:%d\r\n",
                               pSlWlanEvent->Data.ProvisioningStatus.WlanStatus);

                    if (SL_WLAN_STATUS_CONNECTED
                            == pSlWlanEvent->Data.ProvisioningStatus.WlanStatus)
                    {
                        UART_PRINT(" [WLAN EVENT] - Connected to SSID:%s\r\n",
                                   pSlWlanEvent->Data.ProvisioningStatus.Ssid);

                        memcpy(App_CB.connectionSSID,
                               pSlWlanEvent->Data.ProvisioningStatus.Ssid,
                               pSlWlanEvent->Data.ProvisioningStatus.Ssidlen);
                        App_CB.ssidLen = pSlWlanEvent->Data.ProvisioningStatus.Ssidlen;
                        break;
                    }
                    else
                    {
                        CLR_STATUS_BIT(App_CB.status, AppStatusBits_Connection);
                        CLR_STATUS_BIT(App_CB.status, AppStatusBits_IpAcquired);
                        CLR_STATUS_BIT(App_CB.status, AppStatusBits_Ipv6lAcquired);
                        CLR_STATUS_BIT(App_CB.status, AppStatusBits_Ipv6gAcquired);

                        break;
                    }
                }
            }
            break;

            case SL_WLAN_PROVISIONING_SMART_CONFIG_SYNCED:
            {
                UART_PRINT("[WLAN EVENT] Smart Config Synced!\r\n");
            }
            break;

            case SL_WLAN_PROVISIONING_CONFIRMATION_WLAN_CONNECT:
            {
                SET_STATUS_BIT(App_CB.status, AppStatusBits_Connection);
                CLR_STATUS_BIT(App_CB.status, AppStatusBits_IpAcquired);
                CLR_STATUS_BIT(App_CB.status, AppStatusBits_Ipv6lAcquired);
                CLR_STATUS_BIT(App_CB.status, AppStatusBits_Ipv6gAcquired);

                UART_PRINT("[WLAN EVENT] Connection to AP succeeded\r\n");
            }
            break;

            case SL_WLAN_PROVISIONING_CONFIRMATION_IP_ACQUIRED:
            {
                SET_STATUS_BIT(App_CB.status, AppStatusBits_IpAcquired);
                UART_PRINT("[WLAN EVENT] IP address acquired\r\n");
            }
            break;

            case SL_WLAN_PROVISIONING_SMART_CONFIG_SYNC_TIMEOUT:
            {
                UART_PRINT("[WLAN EVENT] Smart Config Sync timeout\r\n");
            }
            break;

            default:
            {
                UART_PRINT("[WLAN EVENT] Unknown Provisioning Status: %d\r\n", pSlWlanEvent->Data.ProvisioningStatus.ProvisioningStatus);
            }
            break;
            }
        }
        break;

        default:
        {
            UART_PRINT("[WLAN EVENT] Unexpected event %d\n\r", pSlWlanEvent->Id);
        }
        break;
    }
}

//*****************************************************************************
//
//! The Function Handles the Fatal errors
//!
//! \param[in]  slFatalErrorEvent - Pointer to Fatal Error Event info
//!
//! \return     None
//!
//*****************************************************************************
void SimpleLinkFatalErrorEventHandler(SlDeviceFatal_t *slFatalErrorEvent)
{
    switch (slFatalErrorEvent->Id)
    {
        case SL_DEVICE_EVENT_FATAL_DEVICE_ABORT:
        {
            UART_PRINT("[ERROR] - FATAL ERROR: Abort NWP event detected: "
                    "AbortType=%d, AbortData=0x%x\n\r", slFatalErrorEvent->Data.DeviceAssert.Code, slFatalErrorEvent->Data.DeviceAssert.Value);
        }
        break;

        case SL_DEVICE_EVENT_FATAL_DRIVER_ABORT:
        {
            UART_PRINT("[ERROR] - FATAL ERROR: Driver Abort detected. \n\r");
        }
        break;

        case SL_DEVICE_EVENT_FATAL_NO_CMD_ACK:
        {
            UART_PRINT("[ERROR] - FATAL ERROR: No Cmd Ack detected "
                    "[cmd opcode = 0x%x] \n\r", slFatalErrorEvent->Data.NoCmdAck.Code);
        }
        break;

        case SL_DEVICE_EVENT_FATAL_SYNC_LOSS:
        {
            UART_PRINT("[ERROR] - FATAL ERROR: Sync loss detected n\r");
        }
        break;

        case SL_DEVICE_EVENT_FATAL_CMD_TIMEOUT:
        {
            UART_PRINT("[ERROR] - FATAL ERROR: Async event timeout detected "
                    "[event opcode =0x%x]  \n\r", slFatalErrorEvent->Data.CmdTimeout.Code);
        }
        break;

        default:
        {
            UART_PRINT("[ERROR] - FATAL ERROR: Unspecified error detected \n\r");
        }
        break;
    }
}

//*****************************************************************************
//
//! This function handles network events such as IP acquisition, IP leased, IP
//! released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    SlNetAppEventData_u *pNetAppEventData = NULL;

    if (NULL == pNetAppEvent){
        return;
    }

    pNetAppEventData = &pNetAppEvent->Data;

    switch (pNetAppEvent->Id)
    {
        case SL_NETAPP_EVENT_IPV4_ACQUIRED:
        {
            SlIpV4AcquiredAsync_t   *pEventData = NULL;

            SET_STATUS_BIT(App_CB.status, AppStatusBits_IpAcquired);

            /* Ip Acquired Event Data */
            pEventData = &pNetAppEvent->Data.IpAcquiredV4;

            /* Gateway IP address */
            App_CB.gatewayIP = pEventData->Gateway;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                   "Gateway=%d.%d.%d.%d\n\r",
                   SL_IPV4_BYTE(pNetAppEventData->IpAcquiredV4.Ip, 3),
                   SL_IPV4_BYTE(pNetAppEventData->IpAcquiredV4.Ip, 2),
                   SL_IPV4_BYTE(pNetAppEventData->IpAcquiredV4.Ip, 1),
                   SL_IPV4_BYTE(pNetAppEventData->IpAcquiredV4.Ip, 0),
                   SL_IPV4_BYTE(pNetAppEventData->IpAcquiredV4.Gateway, 3),
                   SL_IPV4_BYTE(pNetAppEventData->IpAcquiredV4.Gateway, 2),
                   SL_IPV4_BYTE(pNetAppEventData->IpAcquiredV4.Gateway, 1),
                   SL_IPV4_BYTE(pNetAppEventData->IpAcquiredV4.Gateway, 0));
        }
        break;

        case SL_NETAPP_EVENT_IPV6_ACQUIRED:
        {
            if (!GET_STATUS_BIT(App_CB.status, AppStatusBits_Ipv6lAcquired))
            {
                SET_STATUS_BIT(App_CB.status, AppStatusBits_Ipv6lAcquired);
                UART_PRINT("[NETAPP EVENT] Local IPv6 Acquired\n\r");
            }
            else
            {
                SET_STATUS_BIT(App_CB.status, AppStatusBits_Ipv6gAcquired);
                UART_PRINT("[NETAPP EVENT] Global IPv6 Acquired\n\r");
            }
        }
        break;

        case SL_NETAPP_EVENT_DHCPV4_LEASED:
        {
            SET_STATUS_BIT(App_CB.status, AppStatusBits_IpLeased);
            App_CB.staIP = (pNetAppEvent)->Data.IpLeased.IpAddress;

            UART_PRINT("[NETAPP EVENT] IP Leased to Client: IP=%d.%d.%d.%d , ",\
                       SL_IPV4_BYTE(App_CB.staIP, 3), SL_IPV4_BYTE(App_CB.staIP, 2),
                       SL_IPV4_BYTE(App_CB.staIP, 1), SL_IPV4_BYTE(App_CB.staIP, 0));
        }
        break;

        case SL_NETAPP_EVENT_DHCPV4_RELEASED:
        {
            CLR_STATUS_BIT(App_CB.status, AppStatusBits_IpLeased);

            UART_PRINT("[NETAPP EVENT] IP Released for Client: IP=%d.%d.%d.%d , ",\
                       SL_IPV4_BYTE(App_CB.staIP, 3), SL_IPV4_BYTE(App_CB.staIP, 2),
                       SL_IPV4_BYTE(App_CB.staIP, 1), SL_IPV4_BYTE(App_CB.staIP, 0));

            UART_PRINT("Reason: ");
            switch (pNetAppEventData->IpReleased.Reason)
            {
            case SL_IP_LEASE_PEER_RELEASE:
                UART_PRINT("Peer released\n\r");
                break;

            case SL_IP_LEASE_PEER_DECLINE:
                UART_PRINT("Peer declined\n\r");
                break;

            case SL_IP_LEASE_EXPIRED:
                UART_PRINT("Lease expired\n\r");
                break;
            }
        }
        break;

        default:
        {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r", pNetAppEvent->Id);
        }
        break;
    }
}

//*****************************************************************************
//
//! This function handles resource request
//!
//! \param[in]  pNetAppRequest  - Contains the resource requests
//! \param[in]  pNetAppResponse - Should be filled by the user with the
//!                               relevant response information
//!
//! \return     None
//!
//*****************************************************************************
void SimpleLinkNetAppRequestHandler(SlNetAppRequest_t *pNetAppRequest, SlNetAppResponse_t *pNetAppResponse)
{
    /* Unused in this application                                             */
}

//*****************************************************************************
//
//! This function handles HTTP server events
//!
//! \param[in]  pServerEvent     - Contains the relevant event information
//! \param[in]  pServerResponse  - Should be filled by the user with the
//!                                relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlNetAppHttpServerEvent_t *pHttpEvent, SlNetAppHttpServerResponse_t *pHttpResponse)
{
    /* Unused in this application                                             */
}

//*****************************************************************************
//
//! This function handles General Events
//!
//! \param[in]  pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    /* Most of the general errors are not FATAL. are to be handled            */
    /* appropriately by the application.                                      */
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n", pDevEvent->Data.Error.Code, pDevEvent->Data.Error.Source);
}

//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]  pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    /* This application doesn't work w/ socket - Events are not expected      */
    switch (pSock->Event)
    {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch (pSock->SocketAsyncEvent.SockTxFailData.Status)
            {
                case SL_ERROR_BSD_ECLOSE:
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                            "failed to transmit all queued packets\n\r", pSock->SocketAsyncEvent.SockTxFailData.Sd);
                    break;
                default:
                    UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , "
                            "reason (%d) \n\n", pSock->SocketAsyncEvent.SockTxFailData.Sd, pSock->SocketAsyncEvent.SockTxFailData.Status);
                    break;
            }
            break;

        default:
            UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n", pSock->Event);
            break;
    }
}

/****************************************************************************************************************
   SimpleLink Asynchronous Event Handlers -- End
****************************************************************************************************************/
