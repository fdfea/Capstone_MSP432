
/************************************************************
 * Peripherals.h
 *
 * High-level driver functions for peripheral devices,
 * including screen, LEDs, and speaker.
 *
 ************************************************************/

#ifndef PERIPHERALS_H
#define PERIPHERALS_H

void LED_init(void);
void LED_on(int nLed); //turn on the given LED (0-5)
void LED_allOn(void);
void LED_off(int nLed); //turn off the given LED (0-5)
void LED_allOff(void);
void Screen_init(void);
void Screen_updateTime(int Hour, int Min); //screen will always show time, call this every minute with new hour/min to update display
void Screen_printMedInfo(char *MedInfo); //I'm assuming there will be an area on the screen where we will show med data, this may span multiple lines
void Screen_removeMedInfo(void); //clears the screen area allocated for the med info
void Screen_reset(void);
void Screen_updateDate(char *Date);
void Speaker_init(void);
void Speaker_on(void); //starts playing the sound
void Speaker_off(void);

#endif
