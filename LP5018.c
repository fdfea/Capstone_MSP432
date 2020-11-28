#include "LP5018.h"
#include "board.h"

#include "uart_term.h"

void LP5018_init(){
    bool retVal = false;
    I2C_Transaction i2cTransaction;
    txBuffer[0] = 0x00;
    txBuffer[1] = 0x40;
    P6->DIR |= 0x08;
    P6->OUT |= 0x08;
    i2cTransaction.slaveAddress = 0x28;
    i2cTransaction.writeCount = 2;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.readCount = 0;
    i2cTransaction.readBuf = rxBuffer;
    UART_PRINT("Started transfering LP5018\r\n");
    do {
        retVal = I2C_transfer(i2cHandle, &i2cTransaction);
    } while (!retVal);
    UART_PRINT("Finished initializing LP5018\r\n");
}

void LP5018_read(){
    bool retVal = false;
    I2C_Transaction i2cTransaction;
    uint8_t readBuf[32] = {0};
    txBuffer[0] = 0x00;
    i2cTransaction.slaveAddress = 0x28;
    i2cTransaction.writeCount = 1;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.readCount = 30;
    i2cTransaction.readBuf = readBuf;
    UART_PRINT("Starting LP5018 read\r\n");
    do {
        retVal = I2C_transfer(i2cHandle, &i2cTransaction);
    } while (!retVal);
    int i;
    UART_PRINT("LP5018 Read: ");
    for (i = 0; i < 32; i++)
    {
        UART_PRINT("%#x, ", readBuf[i]);
    }
    UART_PRINT("\r\n");
}

void LP5018_setColor(uint8_t led, uint8_t r, uint8_t g, uint8_t b){
    bool retVal = false;
    I2C_Transaction i2cTransaction;
    txBuffer[0] = led + 0x0F;
    txBuffer[1] = r;
    txBuffer[2] = g;
    txBuffer[3] = b;
    i2cTransaction.slaveAddress = 0x28;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.writeCount = 4;
    i2cTransaction.readCount = 0;
    /* Re-try writing to slave till I2C_transfer returns true */
    do {
        retVal = I2C_transfer(i2cHandle, &i2cTransaction);
    } while(!retVal);
}

void LP5018_setAllColor(uint8_t r, uint8_t g, uint8_t b){
    bool retVal = false;
    I2C_Transaction i2cTransaction;
	int i;
	txBuffer[0] = 0x0F;
	for(i = 0; i < 18;){
		txBuffer[++i] = r;
		txBuffer[++i] = g;
		txBuffer[++i] = b;
	}
    i2cTransaction.slaveAddress = 0x28;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.writeCount = ++i;
    i2cTransaction.readCount = 0;
    /* Re-try writing to slave till I2C_transfer returns true */
    do {
        retVal = I2C_transfer(i2cHandle, &i2cTransaction);
    } while(!retVal);
}

void LP5018_setBrightness(uint8_t led, uint8_t brightness){
	bool retVal = false;
    I2C_Transaction i2cTransaction;
	txBuffer[0] = led + 0x07;
	txBuffer[1] = brightness;
    i2cTransaction.slaveAddress = 0x28;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.readBuf = rxBuffer;
	i2cTransaction.writeCount = 2;
	i2cTransaction.readCount = 0;
	/* Re-try writing to slave till I2C_transfer returns true */
	do {
	    retVal = I2C_transfer(i2cHandle, &i2cTransaction);
	} while(!retVal);
}

void LP5018_setAllBrightness(uint8_t brightness){
    bool retVal = false;
    I2C_Transaction i2cTransaction;
	txBuffer[0] = 0x07;
	txBuffer[1] = brightness;
	txBuffer[2] = brightness;
	txBuffer[3] = brightness;
	txBuffer[4] = brightness;
	txBuffer[5] = brightness;
	txBuffer[6] = brightness;
    i2cTransaction.slaveAddress = 0x28;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.readBuf = rxBuffer;
	i2cTransaction.writeCount = 7;
	i2cTransaction.readCount = 0;
    /* Re-try writing to slave till I2C_transfer returns true */
    do {
        retVal = I2C_transfer(i2cHandle, &i2cTransaction);
    } while(!retVal);
}

