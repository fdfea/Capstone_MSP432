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
// network_if.h
//
// Networking interface macro and function prototypes for CC3120 device
//
//*****************************************************************************

#ifndef __NETWORK_IF__H__
#define __NETWORK_IF__H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

/* Simplelink includes */
#include <ti/drivers/net/wifi/simplelink.h>

/* Values for below macros shall be modified as per access-point(AP)          */
/* properties SimpleLink device will connect to following AP when application */
/* is executed. */

/* AP SSID */
#define SSID_NAME               "Valley House" //ENTER NETWORK NAME HERE
/* Security type (OPEN or WEP or WPA) */
#define SECURITY_TYPE           SL_WLAN_SEC_TYPE_WPA_WPA2 /*SL_WLAN_SEC_TYPE_OPEN*/
/* Password of the secured AP */
#define SECURITY_KEY            "Charlottesville1!" //ENTER NETWORK PASSWORD HERE (Don't push this to GitHub you dummy)
#define SSID_AP_MODE            "<ap-ssid>"
#define SEC_TYPE_AP_MODE        SL_WLAN_SEC_TYPE_OPEN
#define PASSWORD_AP_MODE        ""

//*****************************************************************************
// APIs
//*****************************************************************************
long Network_IF_InitDriver(uint32_t uiMode);
long Network_IF_DeInitDriver(void);
long Network_IF_ConnectAP(char * pcSsid, SlWlanSecParams_t SecurityParams);
long Network_IF_DisconnectFromAP();
long Network_IF_IpConfigGet(unsigned long *aucIP,
                            unsigned long *aucSubnetMask,
                            unsigned long *aucDefaultGateway,
                            unsigned long *aucDNSServer);
long Network_IF_GetHostIP(char* pcHostName, unsigned long * pDestinationIP);
unsigned long Network_IF_CurrentMCUState(void);
void Network_IF_UnsetMCUMachineState(char stat);
void Network_IF_SetMCUMachineState(char stat);
void Network_IF_ResetMCUStateMachine(void);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __NETWORK_IF__H__
