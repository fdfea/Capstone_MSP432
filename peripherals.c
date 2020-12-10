#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include "peripherals.h"
#include "EVE.h"
#include "LP5018.h"
#include "board.h"

#define GRAY  	    0x919191UL
#define BLACK  	    0x222222UL
#define WHITE  	    0xFFFFFFUL
#define LAYOUT_Y1   120

#define SPI_BITRATE     1000000

#define SCREEN_UPDATE_INTERVAL_USEC 250000

extern volatile bool peripheralThreadStop;

char printBuf[1024]; //for medInfo to screen
char timeString[10]; //for time to screen
char timeString2[2]; //for am/pm to screen
char dateString[20]; //for month and year to screen
char deviceIdString[25]; //for device Id to screen

bool speakerOn;

void LED_init(void)
{
	I2C_init();
	I2C_Params_init(&i2cParams);
	i2cParams.bitRate = I2C_100kHz;
	i2cHandle = I2C_open(Board_I2C1, &i2cParams);
	if (i2cHandle == NULL)
	{
	    while (1);
	}
	LP5018_init();
	LP5018_setAllBrightness(0);
	LP5018_setAllColor(0,128,0);
}

void LED_on(int nLed)
{
	LP5018_setBrightness(nLed, 128);
}

void LED_allOn(void)
{
	LP5018_setAllBrightness(128);
}

void LED_off(int nLed)
{
	LP5018_setBrightness(nLed, 0);
}

void LED_allOff(void)
{
	LP5018_setAllBrightness(0);
}

void Screen_background(void)
{
	EVE_cmdBGColor(BLACK);
	EVE_cmd(VERTEX_FORMAT(0));
	// Divide the screen
	EVE_cmd(DL_BEGIN | LINES);
	EVE_cmd(LINE_WIDTH(1*16));
	EVE_colorRGB(GRAY);
	EVE_cmd(VERTEX2F(0,LAYOUT_Y1-2));
	EVE_cmd(VERTEX2F(HSIZE,LAYOUT_Y1-2));
	EVE_cmd(DL_END);
}

void Screen_init(void)
{
    SPI_init();
    SPI_Params_init(&spiParams);
    spiParams.bitRate = SPI_BITRATE;
    spiHandle = SPI_open(Board_SPI4, &spiParams);
    if (spiHandle == NULL)
    {
        while(1);
    }
    Screen_reset();
	EVE_init();
	EVE_initFlash();
    // Loading screen while waiting for wifi
	EVE_startBurst();
	EVE_cmd(CMD_DLSTART);
    EVE_cmd(DL_CLEAR_RGB | BLACK);
    EVE_cmd(DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG);
    EVE_cmdSpinner(400,240,1,1);
	EVE_cmd(DL_DISPLAY);
	EVE_cmd(CMD_SWAP);
	EVE_sendBurst();
}

void Screen_printf(const char *format, ...)
{
	va_list args;
	va_start(args, format);
	snprintf(printBuf, sizeof(printBuf), format, args);
	va_end(args);
}

void Screen_printMedInfo(char *MedInfo)
{
	snprintf(printBuf, sizeof(printBuf), MedInfo);
}

void Screen_printDeviceId(char *DeviceId)
{
    snprintf(deviceIdString, sizeof(deviceIdString), DeviceId);
}

void Screen_removeMedInfo(void)
{
    memset(printBuf, 0, sizeof(printBuf));
}

void Screen_reset(void)
{
	memset(printBuf, 0, sizeof(printBuf));
    memset(timeString, 0, sizeof(timeString));
    memset(timeString2, 0, sizeof(timeString2));
    memset(dateString, 0, sizeof(dateString));
    memset(deviceIdString, 0, sizeof(deviceIdString));
}

void Screen_updateTime(int Hour, int Min)
{
	// Assuming 24 hour input, convert to 12 hour
	snprintf(timeString2, sizeof(timeString2), Hour > 11 && Hour < 24 ? "PM":"AM");
	Hour = Hour > 12 ? Hour-12 : Hour;
	snprintf(timeString, sizeof(timeString), "%02d:%02d", Hour, Min);
}

void Screen_updateDate(char *Date)
{
	snprintf(dateString, 20, Date);
}

void Screen_update(void)
{
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
	EVE_cmdText(10, 150, 30, 0, printBuf);
	EVE_cmdText(10, 65, 28, 0, deviceIdString);
	EVE_cmd(DL_DISPLAY);
	EVE_cmd(CMD_SWAP);
	EVE_sendBurst();
}

void *peripheralThreadProc(void *pArg)
{
    delay(100);
	while(!peripheralThreadStop)
	{
	    Screen_update();
	    usleep(SCREEN_UPDATE_INTERVAL_USEC);
	}
	return NULL;
}

void Speaker_init(void)
{
	EVE_setVolume(0x00);
	//EVE_setSound(SQUAREWAVE, MIDI_C1);
	EVE_setSound(ALARM, MIDI_C1);
	EVE_startSound();
    speakerOn = false;
}

void Speaker_on(void)
{
	EVE_startSound();
	EVE_setVolume(0xFF);
	speakerOn = true;
}

void Speaker_off(void)
{
	EVE_stopSound();
	EVE_setVolume(0x00);
	speakerOn = false;
}
