#ifndef __LP5018_H
#define __LP5018_H

/* Driver Header files */
#include <stdint.h>
#include <ti/drivers/I2C.h>
#include <ti/devices/msp432p4xx/inc/msp.h>

/* Buffers */
uint8_t txBuffer[32];
uint8_t rxBuffer[32];

I2C_Handle		i2cHandle;
I2C_Params		i2cParams;
I2C_Transaction i2cTransaction;

// Function Prototypes
void LP5018_init();
void LP5018_setColor(uint8_t led, uint8_t r, uint8_t g, uint8_t b);
void LP5018_setAllColor(uint8_t r, uint8_t g, uint8_t b);
void LP5018_setBrightness(uint8_t led, uint8_t brightness);
void LP5018_setAllBrightness(uint8_t brightness);

#endif
