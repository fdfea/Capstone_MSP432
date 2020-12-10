
/************************************************************
 * Peripherals.h
 *
 * High-level driver functions for peripheral devices,
 * including screen, LEDs, and speaker.
 *
 ************************************************************/

#ifndef PERIPHERALS_H
#define PERIPHERALS_H

void *peripheralThreadProc(void *pArg); //periodically update screen

void LED_init(void);
void LED_on(int nLed); //turn on the given LED (0-5)
void LED_allOn(void);
void LED_off(int nLed); //turn off the given LED (0-5)
void LED_allOff(void);
void Screen_init(void);
void Screen_reset(void); //clear everything from the screen
void Screen_updateTime(int Hour, int Min);
void Screen_printMedInfo(char *MedInfo);
void Screen_removeMedInfo(void);
void Screen_printDeviceId(char *DeviceId);
void Screen_updateDate(char *Date);
void Speaker_init(void);
void Speaker_on(void);
void Speaker_off(void);

#endif
