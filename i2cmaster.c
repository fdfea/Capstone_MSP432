/*
 * Copyright (c) 2016-2019, Texas Instruments Incorporated
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

/*
 *    ======== i2cmaster.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include "board.h"

/* Buffers */
uint8_t txBuffer[32];
uint8_t rxBuffer[32];

extern volatile bool i2cThreadStop;

void LED_Init(I2C_Handle i2c){
    bool            retVal = false;
    I2C_Transaction i2cTransaction;
    txBuffer[0] = 0x00;
    txBuffer[1] = 0x40;
    i2cTransaction.slaveAddress = 0x28;
    i2cTransaction.writeCount = 2;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.readCount = 0;
    i2cTransaction.readBuf = rxBuffer;

    /* Re-try writing to slave till I2C_transfer returns true */
    do {
        retVal = I2C_transfer(i2c, &i2cTransaction);
    } while(!retVal);
}


void LED_Color(I2C_Handle i2c, uint8_t led, uint8_t r, uint8_t g, uint8_t b){
    bool            retVal = false;
    I2C_Transaction i2cTransaction;
    txBuffer[0] = led + 0x0F;
    txBuffer[1] = r;
    txBuffer[2] = g;
    txBuffer[3] = b;
    i2cTransaction.slaveAddress = 0x28;
    i2cTransaction.writeCount = 4;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.readCount = 0;
    i2cTransaction.readBuf = rxBuffer;

    /* Re-try writing to slave till I2C_transfer returns true */
    do {
        retVal = I2C_transfer(i2c, &i2cTransaction);
    } while(!retVal);
}

/*
 *  ======== mainThread ========
 */
void *i2cThreadProc(void *arg0)
{
    I2C_Handle      i2c;
    I2C_Params      i2cParams;

    I2C_init();


    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_100kHz;
    i2c = I2C_open(Board_I2C1, &i2cParams);

    if (i2c == NULL) {
        while (1) {}
    }
    LED_Init(i2c);

    int i, j, k;
    while(!i2cThreadStop) {
        for (i = 0; i < 127; i++){
            LED_Color(i2c, 0, i, 0, 0);
        }
        for (j = 0; j < 127; j++){
            LED_Color(i2c, 0, 0, j, 0);
        }
        for (k = 0; k < 127; k++){
            LED_Color(i2c, 0, 0, 0, k);
        }
    }
    /* Deinitialized I2C */
    I2C_close(i2c);
    return (0);
}

