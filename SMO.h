
#ifndef SMO_H
#define SMO_H

#include <stdint.h>
#include <stdbool.h>

#define SMO_VECTOR_MAX_SIZE     10
#define SMO_MAX_COMPARTMENTS    6
#define SMO_EVENT_TIMEOUT_MINS  5

#define SMO_PACKET_HEADER_SIZE          2
#define SMO_PACKET_MAX_MEDS             6
#define SMO_PACKET_MED_PAYLOAD_SIZE     30
#define SMO_PACKET_TYPE_HEADER          0x98

#define SMO_TIMER_DELAY     1

typedef struct SMO_Event
{
    uint8_t AlarmHour; //hour of alarm
    uint8_t AlarmMin; //minute of alarm
    uint8_t Compartments; //indices set indicate which LEDs should be lit
    uint8_t nPills[SMO_MAX_COMPARTMENTS]; //how many pills to take from each compartment

} SMO_Event;

typedef struct SMO_Vector
{
    SMO_Event **Events;
    uint8_t Size;

} SMO_Vector;

typedef struct SMO_Timer
{
    volatile bool Timing;
    volatile int Count;
    volatile bool Delaying;
    volatile int Delay;

} SMO_Timer;

typedef struct SMO_Control
{
    SMO_Vector EventsVec;
    SMO_Event *CurrentEvent;
    SMO_Timer Timer;
    char *CompartmentStrings[SMO_MAX_COMPARTMENTS]; //string for screen when event occurs

} SMO_Control;

typedef struct SMO_PacketMed
{
    uint8_t AlarmHour; //hour of alarm
    uint8_t AlarmMin; //minute of alarm
    uint8_t nPills; //how many pills to take
    uint8_t nCmptmt; //which compartment med is in
    uint8_t Length; //length of payload
    char Payload[SMO_PACKET_MED_PAYLOAD_SIZE]; //name of med, could add other information

} SMO_PacketMed;

typedef struct SMO_Packet
{
    uint8_t PacketType;
    uint8_t nMeds;
    SMO_PacketMed Meds[SMO_PACKET_MAX_MEDS];

} SMO_Packet;

void SMO_Timer_init(SMO_Timer *Timer);
void SMO_Timer_start(SMO_Timer *Timer);
void SMO_Timer_stop(SMO_Timer *Timer);

void SMO_Control_init(SMO_Control *Ctrl);
void SMO_Control_free(SMO_Control *Ctrl);
int SMO_Control_configure(SMO_Control *Ctrl, SMO_Packet *Pkt);
SMO_Event *SMO_Control_nextEvent(SMO_Control *Ctrl, uint8_t Hour, uint8_t Min);
char *SMO_Control_getMedStr(SMO_Control *Ctrl, uint8_t nCmptmt);

#endif
