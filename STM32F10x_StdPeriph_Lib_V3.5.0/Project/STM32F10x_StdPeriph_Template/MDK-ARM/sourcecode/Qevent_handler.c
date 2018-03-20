#include <stm32f10x.h>
#include "stdbool.h"   //for bool type
#include "Qevent_handler.h"  
#include "QTimer.h"
#include "QuanProcessCMD.h"
#include "stm32_eval.h"
uint16_t gEvents= evNothing;

extern QTimer_t gLEDflashTimer;
extern QTimer_t gBuzzerTimer;
void Delay1us(uint16_t ms);
//extern QTimer_t gLEDflashTimer;

void handleEvent(void){
	if(gEvents & evLEDFlash){
		gLEDflashTimer.enabled=true;
		gLEDflashTimer.counter=300;
		gEvents &=~ evLEDFlash;
	}else if(gEvents & evBuzzer){
		gBuzzerTimer.enabled=true;
		gBuzzerTimer.counter=6;
		gEvents &=~ evBuzzer;
	}else if(gEvents & evRXprocess){
		if(UAART_RX_Port==UARTRX_BLE){
			STM_EVAL_LEDOn(LEDG);  //DEbug
				Rx3ProcessCMD();   //BLE
		}else if(UAART_RX_Port==UARTRX_MTK){	
				STM_EVAL_LEDOn(LEDG);  //DEbug
				Rx5ProcessCMD();   //MTK
		}else if(UAART_RX_Port==UARTRX_Zigbee){
				Rx2ProcessCMD();   //Z200
		}
							STM_EVAL_LEDOff(LEDG);  //DEbug
		gEvents &=~ evRXprocess;
	}else if(gEvents & evPairKey){

		uint8_t Temp[]={"PAIR\r\n"};

	  USARTMTK_to_UART2_long(Temp,6);
		
		
		gEvents &=~ evPairKey;
	}
	
	
	

	
	
}



void sendEvent(uint16_t event){
	gEvents |=event;
}


void Delay1us(uint16_t ms){  //2us
	 uint16_t delay;
	 volatile uint32_t i;
	 for (delay = ms; delay >0 ; delay--)
	//1ms loop with -Os optimisation
	  {
	  for (i=1; i >0;i--){};
	  }
}
