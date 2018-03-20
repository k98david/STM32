#include <stm32f10x.h>
#define CommandLength BUFFERSIZE

void initCommands(void);

typedef struct {
	char commandName[15];
	uint8_t (*commandFunction)(char* parameterStr);
}CommandType_t;

uint8_t registerCommand(uint8_t cmdIndex, char* commandName, uint8_t (*funcPtr)(char *));


typedef enum{
	  UARTRX_BLE=3,
	  UARTRX_MTK=5,
		UARTRX_Zigbee=2
}UartRX_Port_t;
extern UartRX_Port_t UAART_RX_Port;

uint8_t Rx3ProcessCMD(void);
uint8_t Rx5ProcessCMD(void);
uint8_t Rx2ProcessCMD(void);


uint8_t processATTX(char * cmdStr);
uint8_t processoBuzzer(char * cmdStr);
uint8_t processclosedoor(char * cmdStr);
uint8_t processATT1(char * cmdStr);
uint8_t processATT2(char * cmdStr);

