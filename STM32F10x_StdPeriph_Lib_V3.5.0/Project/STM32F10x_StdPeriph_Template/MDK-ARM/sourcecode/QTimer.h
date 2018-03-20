#include <stm32f10x.h>
#include "stdbool.h"   //for bool type

typedef struct {
	bool enabled;
	uint16_t counter;
	uint8_t beepTime; //conrol long or short beep
	uint8_t interval; //always set as an odd number, e.g., 1,3,5. When it is even, stop sounding.
} QTimer_t;


