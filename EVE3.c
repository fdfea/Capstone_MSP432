#include <string.h>
#include <stdlib.h>

#include "EVE.h"
#include "Board.h"

#include "uart_term.h"

void delay(uint32_t ms){
	uint32_t count;
	while(ms > 0){
		for(count = 0; count < 8000; count++){
			__nop();
		}
		ms--;
	}
}

static inline void waitFIFO(){
    while(EVE_read16(REG_CMDB_SPACE + RAM_REG) != 0xFFC){
        __nop();
    }
}

void EVE_reset(){
    // Toggle PDN low
	EVE_PDN_PORT->OUT &= ~EVE_PDN;
	delay(100);
    // Toggle PDN high
	EVE_PDN_PORT->OUT |= EVE_PDN;
	delay(250);
}

int EVE_startBurst(){
    uint32_t address = REG_CMDB_WRITE + RAM_REG;
    // Make sure we aren't already in the middle of a burst transaction
    if(txIdx){
        return 0;
    }
    // Wait until EVE is ready for transaction
    waitFIFO();
    // Set Address
    txBuf[txIdx++] = (uint8_t)((address>>16) | WRITE);
    txBuf[txIdx++] = (uint8_t)(address>>8);
    txBuf[txIdx++] = (uint8_t)(address);
    return 1;
}

int EVE_sendBurst(){
    int ret;
    spiTransaction.count = txIdx;
    ret = SPI_transfer(spiHandle, &spiTransaction);
    txIdx = 0;
    return ret;
}

void EVE_burst8(uint8_t data){
    txBuf[txIdx++] = data;
}

void EVE_burst16(uint16_t data){
    txBuf[txIdx++] = (uint8_t)(data);
    txBuf[txIdx++] = (uint8_t)(data>>8);
}

void EVE_burst32(uint32_t data){
    txBuf[txIdx++] = (uint8_t)(data);
    txBuf[txIdx++] = (uint8_t)(data>>8);
    txBuf[txIdx++] = (uint8_t)(data>>16);
    txBuf[txIdx++] = (uint8_t)(data>>24);
}

void EVE_write8(uint32_t address, uint8_t data){
	// Set Address
    txBuf[0] = (uint8_t)((address>>16) | WRITE);
    txBuf[1] = (uint8_t)(address>>8);
    txBuf[2] = (uint8_t)(address);
	// Set Data
    txBuf[3] = (uint8_t)(data);
	// Send Transaction
	spiTransaction.count = 4;
	SPI_transfer(spiHandle, &spiTransaction);
}

void EVE_write16(uint32_t address, uint16_t data){
	// Set Address
    txBuf[0] = (uint8_t)((address>>16) | WRITE);
    txBuf[1] = (uint8_t)(address>>8);
    txBuf[2] = (uint8_t)(address);
	// Set Data
    txBuf[3] = (uint8_t)(data);
    txBuf[4] = (uint8_t)(data>>8);
	// Send Transaction
	spiTransaction.count = 5;
	SPI_transfer(spiHandle, &spiTransaction);
}

void EVE_write32(uint32_t address, uint32_t data){
	// Set Address
    txBuf[0] = (uint8_t)((address>>16) | WRITE);
    txBuf[1] = (uint8_t)(address>>8);
    txBuf[2] = (uint8_t)(address);
	// Set Data
    txBuf[3] = (uint8_t)(data);
    txBuf[4] = (uint8_t)(data>>8);
    txBuf[5] = (uint8_t)(data>>16);
    txBuf[6] = (uint8_t)(data>>24);
	// Send Transaction
	spiTransaction.count = 7;
	SPI_transfer(spiHandle, &spiTransaction);
}

uint8_t EVE_read8(uint32_t address){
    uint8_t dataRead = 0;
	// Set Address
    memset(txBuf, 0, 5);
	txBuf[0] = (uint8_t)((address>>16) | READ);
	txBuf[1] = (uint8_t)(address>>8);
    txBuf[2] = (uint8_t)(address);
	// Send Transaction
    spiTransaction.count = 5;
    SPI_transfer(spiHandle, &spiTransaction);
    // Read Data
    dataRead |= rxBuf[4];
	return dataRead;
}

uint16_t EVE_read16(uint32_t address){
    uint16_t dataRead = 0;
	// Set Address
    memset(txBuf, 0, 6);
	txBuf[0] = (uint8_t)((address>>16) | READ);
	txBuf[1] = (uint8_t)(address>>8);
	txBuf[2] = (uint8_t)(address);
	// Send Transaction
	spiTransaction.count = 6;
	SPI_transfer(spiHandle, &spiTransaction);
    // Read Data
    dataRead |= (uint16_t)rxBuf[4];
    dataRead |= (uint16_t)rxBuf[5] << 8;
    return dataRead;
}

uint32_t EVE_read32(uint32_t address){
	uint32_t dataRead = 0;
	// Set Address
    memset(txBuf, 0, 8);
	txBuf[0] = (uint8_t)((address>>16) | READ);
	txBuf[1] = (uint8_t)(address>>8);
	txBuf[2] = (uint8_t)(address);
	// Send Transaction
	spiTransaction.count = 8;
	SPI_transfer(spiHandle, &spiTransaction);
    // Read Data
    dataRead |= (uint32_t)rxBuf[4];
    dataRead |= (uint32_t)rxBuf[5] << 8;
    dataRead |= (uint32_t)rxBuf[6] << 16;
    dataRead |= (uint32_t)rxBuf[7] << 24;
	return dataRead;
}

void EVE_sendHCMD(uint8_t command, uint8_t param){
	// Set Command
	txBuf[0] = command;
	txBuf[1] = param;
	txBuf[2] = 0x00;
	// Send Transaction
	spiTransaction.count = 3;
	SPI_transfer(spiHandle, &spiTransaction);
}

void EVE_init(){
    // Initialize SPI buffers
    txBuf = (uint8_t*)calloc(1024,1);
    rxBuf = (uint8_t*)calloc(1024,1);
    spiTransaction.txBuf = txBuf;
    spiTransaction.rxBuf = rxBuf;
    txIdx = 0;
    // Configure EVE Reset
    EVE_PDN_PORT->DIR |= EVE_PDN;
    //UART_PRINT("Eve here 1\r\n");
	EVE_reset();
    // Activate EVE Clock
	EVE_sendHCMD(CLKEXT,0);
	EVE_sendHCMD(CLK36M,0x46);
	EVE_sendHCMD(ACTIVE,0);
    // Wait for EVE to start up
	delay(300);
	//UART_PRINT("Eve here 2\r\n");
	uint8_t readVal = 0x00;
	while(/*EVE_read8(REG_ID + RAM_REG)*/readVal != 0x7C){
	    readVal = EVE_read8(REG_ID + RAM_REG);
	    //UART_PRINT("%d\r\n", readVal);
		delay(1);
	}
    //UART_PRINT("Eve here 3\r\n");
    // Set backlight to 0% power
	EVE_write8(REG_PWM_DUTY + RAM_REG, 0);
	// Initialize Display
	EVE_write16(REG_HCYCLE + RAM_REG, HCYCLE);
	EVE_write16(REG_HOFFSET + RAM_REG, HOFFSET);
	EVE_write16(REG_HSYNC0 + RAM_REG, HSYNC0);
	EVE_write16(REG_HSYNC1 + RAM_REG, HSYNC1);
	EVE_write16(REG_VCYCLE + RAM_REG, VCYCLE);
	EVE_write16(REG_VOFFSET + RAM_REG, VOFFSET);
	EVE_write16(REG_VSYNC0 + RAM_REG, VSYNC0);
	EVE_write16(REG_VSYNC1 + RAM_REG, VSYNC1);
	EVE_write8(REG_SWIZZLE + RAM_REG, SWIZZLE);
	EVE_write8(REG_PCLK_POL + RAM_REG, PCLK_POL);
	EVE_write16(REG_HSIZE + RAM_REG, HSIZE);
	EVE_write16(REG_VSIZE + RAM_REG, VSIZE);
    EVE_write8(REG_CSPREAD + RAM_REG, CSPREAD);
    EVE_write8(REG_DITHER + RAM_REG, DITHER);
    // Blank screen
	EVE_write32(RAM_DL+0, CLEAR_COLOR_RGB(0, 0, 0)); 
	EVE_write32(RAM_DL+4, CLEAR(1, 1, 1)); 
	EVE_write32(RAM_DL+8, DISPLAY()); 
	EVE_write8(REG_DLSWAP + RAM_REG, DLSWAP_FRAME);
	EVE_write8(REG_GPIO_DIR + RAM_REG, 0x80 | EVE_read8(REG_GPIO_DIR + RAM_REG));
	EVE_write8(REG_GPIO + RAM_REG, 0x080 | EVE_read8(REG_GPIO + RAM_REG));
	EVE_write8(REG_PCLK + RAM_REG, PCLK);
	EVE_write8(REG_PWM_DUTY + RAM_REG, 0x10);
    // Wait for FIFO queue to empty
    //UART_PRINT("Eve here 4\r\n");
	waitFIFO();
    //UART_PRINT("Eve here 5\r\n");
}

uint32_t EVE_initFlash(){
    uint16_t offset;
    // Wait for flash to initialize
    while(!EVE_read8(REG_FLASH_STATUS + RAM_REG)){
        __nop();
    }
    EVE_startBurst();
    EVE_burst32(CMD_FLASHFAST);
    EVE_burst32(0);
    EVE_sendBurst();
    //waitFIFO();
    offset = EVE_read16(REG_CMD_WRITE + RAM_REG);
    offset -= 4;
    offset &= 0x0fff;
    return EVE_read32(RAM_CMD + offset);
}

void EVE_setSound(uint16_t sound, uint16_t pitch){
	//waitFIFO();
	EVE_write16(REG_SOUND + RAM_REG, sound |= (pitch << 8));
}


void EVE_startSound(){
	//waitFIFO();
	EVE_write8(REG_PLAY + RAM_REG, 1);
}

void EVE_stopSound(){
    //waitFIFO();
    EVE_write8(REG_PLAY + RAM_REG, 0);
}

void EVE_toggleSound(){
    //waitFIFO();
    EVE_write8(REG_PLAY + RAM_REG, EVE_read8(REG_PLAY + RAM_REG)^1);
}

void EVE_setVolume(uint8_t vol){
	//waitFIFO();
	EVE_write8(REG_VOL_SOUND + RAM_REG, vol);
}

void EVE_writeString(char* string){
    int len = strlen(string);
    memcpy(&txBuf[txIdx], string, len+1);
    txIdx += len;
    int padding = 4 - (len % 4);
    switch(padding){
    case 4:
        txBuf[++txIdx] = 0x00;
    case 3:
        txBuf[++txIdx] = 0x00;
    case 2:
        txBuf[++txIdx] = 0x00;
    case 1:
        txBuf[++txIdx] = 0x00;
    }
}

void EVE_cmdFillWidth(uint32_t val){
    EVE_burst32(CMD_FILLWIDTH);
    EVE_burst32(val);
}

void EVE_cmdBGColor(uint32_t color){
    EVE_burst32(CMD_BGCOLOR);
    EVE_burst32(color);
}

void EVE_cmdFGColor(uint32_t color){
    EVE_burst32(CMD_FGCOLOR);
    EVE_burst32(color);
}

void EVE_cmdROMFont(uint32_t font, uint32_t slot){
    EVE_burst32(CMD_ROMFONT);
    EVE_burst32(font);
    EVE_burst32(slot);
}

void EVE_cmdText(int16_t x, int16_t y, int16_t font, uint16_t options, char* text){
    EVE_burst32(CMD_TEXT);
    EVE_burst16(x);
    EVE_burst16(y);
    EVE_burst16(font);
    EVE_burst16(options);
    EVE_writeString(text);
    //----------------
    //waitFIFO();
}

void EVE_cmdAppend(uint32_t ptr, uint32_t len){
    EVE_burst32(CMD_APPEND);
    EVE_burst32(ptr);
    EVE_burst32(len);
}

void EVE_cmdSpinner(int16_t x, int16_t y, uint16_t style, uint16_t scale){
    EVE_burst32(CMD_SPINNER);
    EVE_burst16(x);
    EVE_burst16(y);
    EVE_burst16(style);
    EVE_burst16(scale);
}

void EVE_cmdLoadImage(uint32_t dest, uint32_t options, uint8_t *data, uint32_t len){
    uint32_t bytes = 0;
    uint32_t blockSize = 0;
    int padding;
    EVE_startBurst();
    EVE_burst32(CMD_LOADIMAGE);
    EVE_burst32(dest);
    EVE_burst32(options);
    EVE_sendBurst();
    //waitFIFO();
    while(bytes < len){
        blockSize = (len-bytes) > 1000 ? 1000:bytes;
        EVE_startBurst();
        memcpy(&txBuf[txIdx], &data[bytes], blockSize);
        txIdx+=blockSize;
        padding = 4 - (blockSize % 4);
        switch(padding){
        case 4:
            txBuf[++txIdx] = 0x00;
        case 3:
            txBuf[++txIdx] = 0x00;
        case 2:
            txBuf[++txIdx] = 0x00;
        case 1:
            txBuf[++txIdx] = 0x00;
        }
        EVE_sendBurst();
        bytes+=blockSize;
        //waitFIFO();
    }
}

void EVE_cmdSetBitmap(uint32_t address, uint16_t format, uint16_t width, uint16_t height){
    EVE_burst32(CMD_SETBITMAP);
    EVE_burst32(address);
    EVE_burst16(format);
    EVE_burst16(width);
    EVE_burst16(height);
}

void EVE_cmd(uint32_t cmd){
    EVE_burst32(cmd);
}

void EVE_colorRGB(uint32_t color){
    EVE_burst32(DL_COLOR_RGB | color);
}
