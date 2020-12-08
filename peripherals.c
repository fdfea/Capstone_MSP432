#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "EVE.h"
#include "LP5018.h"
#include "board.h"

#include "peripherals.h"

#include "uart_term.h"

#define GRAY  	    0x919191UL
#define BLACK  	    0x222222UL
#define WHITE  	    0xFFFFFFUL
#define LAYOUT_Y1   120

extern volatile bool peripheralThreadStop;

char printBuf[1024];
char timeString[10];
char timeString2[2];
char dateString[20];
int sec;
int min;
int hour;
bool speakerOn;

//extern I2C_Handle i2cHandle;

void LED_init(){
	I2C_init();
	/* Create I2C for usage */
	I2C_Params_init(&i2cParams);
	i2cParams.bitRate = I2C_100kHz;
	i2cHandle = I2C_open(Board_I2C1, &i2cParams);
	if (i2cHandle == NULL)
	{
	    //UART_PRINT("Error opening I2C\r\n");
	    while (1);
	}
    //UART_PRINT("Opened peripheral I2C\r\n");
	LP5018_init();
	LP5018_setAllBrightness(0);
	LP5018_setAllColor(0,128,0);
}

void LED_on(int nLed){
	LP5018_setBrightness(nLed, 128);
}

void LED_allOn(void){
	LP5018_setAllBrightness(128);
}

void LED_off(int nLed){
	LP5018_setBrightness(nLed, 0);
}

void LED_allOff(void){
	LP5018_setAllBrightness(0);
}

void Screen_background(){
	EVE_cmdBGColor(BLACK/*0x008000*/);
	EVE_cmd(VERTEX_FORMAT(0));
	// Divide the screen
	EVE_cmd(DL_BEGIN | LINES);
	EVE_cmd(LINE_WIDTH(1*16));
	EVE_colorRGB(GRAY);
	EVE_cmd(VERTEX2F(0,LAYOUT_Y1-2));
	EVE_cmd(VERTEX2F(HSIZE,LAYOUT_Y1-2));
	EVE_cmd(DL_END);
}

void Screen_init(){
    SPI_init();
    SPI_Params_init(&spiParams);
    spiParams.bitRate = 1000000;
    spiHandle = SPI_open(Board_SPI4, &spiParams);
    //UART_PRINT("Here 1\r\n");
	EVE_init();
	//EVE_initFlash();
    //UART_PRINT("Here 2\r\n");
    // Loading screen while waiting for wifi
	EVE_startBurst();
    //UART_PRINT("Here 3\r\n");
	EVE_cmd(CMD_DLSTART);
    EVE_cmd(DL_CLEAR_RGB | BLACK);
    EVE_cmd(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG);
    EVE_cmdSpinner(400,240,1,1);
	EVE_cmd(DL_DISPLAY);
	EVE_cmd(CMD_SWAP);
    //UART_PRINT("Here 4\r\n");
	EVE_sendBurst();
	//Screen_reset();
	//UART_PRINT("Here 5\r\n");
}

void Screen_printf(const char *format, ...){
	va_list args;
	va_start(args, format);
	sprintf(printBuf, format, args);
	va_end(args);
}

void Screen_printMedInfo(char *MedInfo){
	sprintf(printBuf, MedInfo);
}

void Screen_reset(void){
	memset(printBuf, 0, sizeof(printBuf));
	//Screen_update();
}

void Screen_updateTime(int Hour, int Min){
	// Assuming 24 hour input, convert to 12 hour
	sprintf(timeString2, Hour > 11 && Hour < 24 ? "PM":"AM");
	Hour = Hour > 12 ? Hour-12:Hour;
	// Zero pad single digit min
	sprintf(timeString, Min < 10 ? "%d:0%d":"%d:%d", Hour, Min);
}

void Screen_updateDate(char *Date) {
	//snprintf(dateString, 20, "%s", Date);
    sprintf(dateString, "%s", Date);
}

void Screen_update(){
	EVE_startBurst();
	// Clear screen
	EVE_cmd(CMD_DLSTART);
	EVE_cmd(DL_CLEAR_RGB | BLACK);
	EVE_cmd(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG);
	// Static background
	Screen_background();
	// Display date/time
	EVE_cmdText(580, 20, 31, 0, timeString);
	EVE_cmdText(750, 40, 28, 0, timeString2);
	EVE_cmdText(580, 65, 29, 0, dateString);
	// Display medication info / other info
	//EVE_cmdText(10, 150, 30, 0, printBuf);
	EVE_cmd(DL_DISPLAY);
	EVE_cmd(CMD_SWAP);
	EVE_sendBurst();
}

void *peripheralThreadProc(void *arg0){
    //delay(1000);
    sleep(5);
    Screen_updateTime(15, 27);
	Screen_updateDate("November 14, 2020");
	//sec = 58;
	//min = 59;
	//hour = 11;//
	while(!peripheralThreadStop){
		// Simple clock, replace with rtc
	    /*
		sec++;
		delay(1000);
        if (sec == 60) {
            sec = 0;
            min++;
            if (min == 60) {
                min = 0;
                hour++;
                if(hour == 25){
                    hour = 1;
                }
            }
        }
        Screen_updateTime(hour, min);
        */
	    // Update display
	    Screen_update();
	    //UART_PRINT("Done updating\r\n");
		//UART_PRINT("Peripheral to sleep\r\n");
		//usleep(10000);
        //sleep(1);
		delay(1000);
		//UART_PRINT("Peripheral wake\r\n");
	}
	//UART_PRINT("Peripheral thread stopping\r\n");
	return (0);
}

void Speaker_init(void){
	EVE_setVolume(0x00);
	//EVE_setSound(SQUAREWAVE, MIDI_C1);
	EVE_setSound(WARBLE, MIDI_C1);
	//--
	EVE_startSound();
}

void Speaker_on(void){
	//EVE_startSound();
	EVE_setVolume(0xFF);
}

void Speaker_off(void){
	//EVE_stopSound();
	EVE_setVolume(0x00);
}
