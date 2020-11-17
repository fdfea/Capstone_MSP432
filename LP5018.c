#include "LP5018.h"
#include "board.h"

void LP5018_Init(){
    bool retVal = false;
    txBuffer[0] = 0x00;
    txBuffer[1] = 0x40;
    i2cTransaction.slaveAddress = 0x28;
    i2cTransaction.writeCount = 2;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.readCount = 0;
    i2cTransaction.readBuf = rxBuffer;
}

void LP5018_setColor(uint8_t led, uint8_t r, uint8_t g, uint8_t b){
    bool retVal = false;
    txBuffer[0] = led + 0x0F;
    txBuffer[1] = r;
    txBuffer[2] = g;
    txBuffer[3] = b;
    i2cTransaction.writeCount = 4;
    i2cTransaction.readCount = 0;
    /* Re-try writing to slave till I2C_transfer returns true */
    do {
        retVal = I2C_transfer(i2cHandle, &i2cTransaction);
    } while(!retVal);
}

void LP5018_setAllColor(uint8_t r, uint8_t g, uint8_t b){
    bool retVal = false;
	int i;
	txBuffer[0] = 0x0F;
	for(i = 0; i < 18;){
		txBuffer[++i] = r;
		txBuffer[++i] = g;
		txBuffer[++i] = b;
	}
    i2cTransaction.writeCount = ++i;
    i2cTransaction.readCount = 0;
    /* Re-try writing to slave till I2C_transfer returns true */
    do {
        retVal = I2C_transfer(i2cHandle, &i2cTransaction);
    } while(!retVal);
}

void LP5018_setBrightness(uint8_t led, uint8_t brightness){
	bool retVal = false;
	txBuffer[0] = led + 0x07;
	txBuffer[1] = brightness;
	i2cTransaction.writeCount = 2;
	i2cTransaction.readCount = 0;
	/* Re-try writing to slave till I2C_transfer returns true */
	do {
	    retVal = I2C_transfer(i2cHandle, &i2cTransaction);
	} while(!retVal);
}

void LP5018_setAllBrightness(uint8_t brightness){
    bool retVal = false;
	txBuffer[0] = 0x07;
	txBuffer[1] = brightness;
	txBuffer[2] = brightness;
	txBuffer[3] = brightness;
	txBuffer[4] = brightness;
	txBuffer[5] = brightness;
	txBuffer[6] = brightness;
	i2cTransaction.writeCount = 7;
	i2cTransaction.readCount = 0;
    /* Re-try writing to slave till I2C_transfer returns true */
    do {
        retVal = I2C_transfer(i2cHandle, &i2cTransaction);
    } while(!retVal);
}

