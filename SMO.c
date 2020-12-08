
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "uart_term.h"

#include "SMO.h"

static void SMO_Event_init(SMO_Event *Event);
static void SMO_Event_setTime(SMO_Event *Event, uint8_t AlarmHour, uint8_t AlarmMin);
static int SMO_Event_addMed(SMO_Event *Event, uint8_t nCmptmt, uint8_t nPills);

static void SMO_Vector_init(SMO_Vector *Vec);
static void SMO_Vector_free(SMO_Vector *Vec);
static int SMO_Vector_addMed(SMO_Vector *Vec, SMO_PacketMed *Med);
static SMO_Event *SMO_Vector_findNextEvent(SMO_Vector *Vec, uint8_t Hour, uint8_t Min);

static int SMO_Control_addMedStr(SMO_Control *Ctrl, uint8_t nCmptmt, char *MedString, uint8_t Len);

static void SMO_Event_init(SMO_Event *Event)
{
    Event->AlarmHour = 0;
    Event->AlarmMin = 0;
    Event->Compartments = 0;
    memset(Event->nPills, 0, sizeof(Event->nPills));
}

static void SMO_Event_setTime(SMO_Event *Event, uint8_t AlarmHour, uint8_t AlarmMin)
{
    Event->AlarmHour = AlarmHour;
    Event->AlarmMin = AlarmMin;
}

static int SMO_Event_addMed(SMO_Event *Event, uint8_t nCmptmt, uint8_t nPills)
{
    int Res = 0;

    if (nCmptmt >= SMO_MAX_COMPARTMENTS || ((Event->Compartments & (1 << nCmptmt)) != 0))
    {
        //UART_PRINT("Error adding medication\r\n");
        Res = -EINVAL;
        goto Error;
    }

    Event->Compartments |= (1 << nCmptmt);
    Event->nPills[nCmptmt] = nPills;

Error:
    return Res;
}

static void SMO_Vector_init(SMO_Vector *Vec)
{
    Vec->Events = malloc(sizeof(void*)*SMO_VECTOR_MAX_SIZE);
    int i;
    for (i = 0; i < SMO_VECTOR_MAX_SIZE; ++i)
    {
        Vec->Events[i] = NULL;
    }
    Vec->Size = 0;
}

static void SMO_Vector_free(SMO_Vector *Vec)
{
    int i;
    if (Vec->Events != NULL)
    {
        for (i = 0; i < SMO_VECTOR_MAX_SIZE; ++i)
        {
            free(Vec->Events[i]);
            Vec->Events[i] = NULL;
        }
    }
    free(Vec->Events);
    Vec->Events = NULL;
    Vec->Size = 0;
}

static int SMO_Vector_addMed(SMO_Vector *Vec, SMO_PacketMed *Med)
{
    int Res = 0;
    SMO_Event *TmpEvent = NULL, *NewEvent = NULL;
    int i, AddIndex;

    if (Vec->Size >= SMO_VECTOR_MAX_SIZE)
    {
        Res = -EINVAL;
        //UART_PRINT("Vector capacity full\r\n");
        goto Error;
    }

    //events are sorted chronologically, with respect to time of day
    //find where medication should go
    AddIndex = Vec->Size;
    for (i = 0; i < Vec->Size; ++i)
    {
        TmpEvent = Vec->Events[i];
        if (TmpEvent->AlarmHour == Med->AlarmHour
            && TmpEvent->AlarmMin == Med->AlarmMin)
        {
            //UART_PRINT("Adding med to event at %02d:%02d in Cmptmt %d\r\n",
                       //TmpEvent->AlarmHour, TmpEvent->AlarmMin, Med->nCmptmt);
            SMO_Event_addMed(TmpEvent, Med->nCmptmt, Med->nPills);
            goto Success;
        }
        else if (TmpEvent->AlarmHour > Med->AlarmHour
                 || (TmpEvent->AlarmHour == Med->AlarmHour
                 && TmpEvent->AlarmMin > Med->AlarmMin))
        {
            AddIndex = i;
            break;
        }
    }

    //UART_PRINT("Creating new event for med at %02d:%02d in Cmptmt %d\r\n", Med->AlarmHour, Med->AlarmMin, Med->nCmptmt);
    //allocate memory for the new event
    NewEvent = malloc(sizeof(SMO_Event));
    SMO_Event_init(NewEvent);
    SMO_Event_setTime(NewEvent, Med->AlarmHour, Med->AlarmMin);
    SMO_Event_addMed(NewEvent, Med->nCmptmt, Med->nPills);

    //shift other events if necessary
    Vec->Size++;
    for (i = AddIndex; i < Vec->Size; ++i)
    {
        TmpEvent = Vec->Events[i];
        Vec->Events[i] = NewEvent;
        NewEvent = TmpEvent;
    }

Error:
Success:
    return Res;
}

static SMO_Event *SMO_Vector_findNextEvent(SMO_Vector *Vec, uint8_t Hour, uint8_t Min)
{
    SMO_Event *Event = NULL;

    if (Vec->Size == 0)
    {
        goto Error;
    }

    //first element is default, e.g. if the given time is after all events
    //in vector, the next event will be the first event on the next day
    Event = Vec->Events[0];
    int i;
    for (i = 0; i < Vec->Size; ++i)
    {
        if (Vec->Events[i]->AlarmHour > Hour
            || ((Vec->Events[i]->AlarmHour == Hour)
            && Vec->Events[i]->AlarmMin > Min))
        {
            Event = Vec->Events[i];
            break;
        }
    }

Error:
    return Event;
}

void SMO_Timer_init(SMO_Timer *Timer)
{
    Timer->Timing = false;
    Timer->Count = 0;
}

void SMO_Timer_start(SMO_Timer *Timer)
{
    //UART_PRINT("Starting timer\r\n");
    //start counting up to SMO event timeout, every minute
    Timer->Count = 0;
    Timer->Timing = true;
}

void SMO_Timer_stop(SMO_Timer *Timer)
{
    //UART_PRINT("Stopping timer\r\n");
    //reset timer state
    Timer->Timing = false;
    Timer->Count = 0;
}

void SMO_Control_init(SMO_Control *Ctrl)
{
    SMO_Vector_init(&Ctrl->EventsVec);
    Ctrl->CurrentEvent = NULL;
    SMO_Timer_init(&Ctrl->Timer);

    int i;
    for (i = 0; i < SMO_MAX_COMPARTMENTS; ++i)
    {
        Ctrl->CompartmentStrings[i] = NULL;
    }

}

void SMO_Control_free(SMO_Control *Ctrl)
{
    if (Ctrl->Timer.Timing)
    {
        SMO_Timer_stop(&Ctrl->Timer);
    }
    Ctrl->CurrentEvent = NULL;
    SMO_Vector_free(&Ctrl->EventsVec);

    int i;
    for (i = 0; i < SMO_MAX_COMPARTMENTS; ++i)
    {
        free(Ctrl->CompartmentStrings[i]);
        Ctrl->CompartmentStrings[i] = NULL;
    }
}

static int SMO_Control_addMedStr(SMO_Control *Ctrl, uint8_t nCmptmt, char *MedStr, uint8_t Len)
{
    int Res = 0;
    char Str[SMO_PACKET_MED_PAYLOAD_SIZE];

    if (nCmptmt >= SMO_MAX_COMPARTMENTS)
    {
        Res = -EINVAL;
        goto Error;
    }

    memset(Str, 0, sizeof(Str));
    strncpy(Str, MedStr, Len);

    if (Ctrl->CompartmentStrings[nCmptmt] != NULL)
    {
        free(Ctrl->CompartmentStrings[nCmptmt]);
    }
    //UART_PRINT("Adding med info in %d, %s\r\n", nCmptmt, Str);
    Ctrl->CompartmentStrings[nCmptmt] = strdup(Str);

Error:
    return Res;
}

int SMO_Control_configure(SMO_Control *Ctrl, SMO_Packet *Pkt)
{
    int Res = 0;

    //reset control before reconfiguring
    SMO_Control_free(Ctrl);
    SMO_Control_init(Ctrl);

    SMO_PacketMed *Med = NULL;
    int i;
    for (i = 0; i < Pkt->nMeds; ++i)
    {
        Med = &Pkt->Meds[i];
        Res |= SMO_Vector_addMed(&Ctrl->EventsVec, Med);
        Res |= SMO_Control_addMedStr(Ctrl, Med->nCmptmt, Med->Payload, Med->Length);
    }

    return Res;
}

SMO_Event *SMO_Control_nextEvent(SMO_Control *Ctrl, uint8_t Hour, uint8_t Min)
{
    return SMO_Vector_findNextEvent(&Ctrl->EventsVec, Hour, Min);
}

char *SMO_Control_getMedStr(SMO_Control *Ctrl, uint8_t nCmptmt)
{
    char *Str = NULL;

    if (nCmptmt >= SMO_MAX_COMPARTMENTS)
    {
       goto Error;
    }

    Str = Ctrl->CompartmentStrings[nCmptmt];

Error:
    return Str;
}
