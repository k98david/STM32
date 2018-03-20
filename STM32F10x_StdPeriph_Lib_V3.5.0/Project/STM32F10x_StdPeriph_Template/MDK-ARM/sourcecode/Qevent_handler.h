#include <stm32f10x.h>
#include "stdbool.h"   //for bool type

//=== Event definitions
#define	evNothing		 0
#define	evLEDFlash   0x0001
#define	evBuzzer     0x0002
#define	evRXprocess  0x0004
#define	evPairKey    0x0008

extern uint16_t gEvents;

void handleEvent(void);
void sendEvent(uint16_t);
void Delay1us(uint16_t ms);
