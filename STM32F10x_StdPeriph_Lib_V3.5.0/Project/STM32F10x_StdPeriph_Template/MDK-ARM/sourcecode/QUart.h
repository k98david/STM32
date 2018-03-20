#include "stdbool.h"   //for bool type

#define BUFFERSIZE	250
#define TxBufferSize3   BUFFERSIZE
#define RxBufferSize3   BUFFERSIZE
#define TxBufferSize5   BUFFERSIZE
#define RxBufferSize5   BUFFERSIZE
#define TxBufferSize2   BUFFERSIZE
#define RxBufferSize2   BUFFERSIZE
#define TxBufferSize1   BUFFERSIZE
#define RxBufferSize1   BUFFERSIZE
#define TxBufferSize4   BUFFERSIZE
#define RxBufferSize4   BUFFERSIZE

extern uint8_t RxBuffer_BLE[RxBufferSize3];
extern uint8_t TxBuffer_BLE[TxBufferSize3];
extern uint8_t TxHead3; 
extern uint8_t TxTail3;
extern uint8_t RxHead3; 
extern uint8_t RxTail3;
extern uint8_t RxBuffer_MTK[RxBufferSize5];
extern uint8_t TxBuffer_MTK[TxBufferSize5];
extern uint8_t TxHead5; 
extern uint8_t TxTail5;
extern uint8_t RxHead5; 
extern uint8_t RxTail5;
extern uint8_t RxBuffer_UART1[RxBufferSize1];
extern uint8_t TxBuffer_UART1[TxBufferSize1];
extern uint8_t TxHead1; 
extern uint8_t TxTail1;
extern uint8_t RxHead1; 
extern uint8_t RxTail1;
extern uint8_t RxBuffer_Z200[RxBufferSize2];
extern uint8_t TxBuffer_Z200[TxBufferSize2];
extern uint8_t TxHead2; 
extern uint8_t TxTail2;
extern uint8_t RxHead2; 
extern uint8_t RxTail2;
extern uint8_t RxBuffer_UART4[RxBufferSize4];
extern uint8_t TxBuffer_UART4[TxBufferSize4];
extern uint8_t TxHead4; 
extern uint8_t TxTail4;
extern uint8_t RxHead4; 
extern uint8_t RxTail4;
extern USART_InitTypeDef USART_InitStructure;
extern uint8_t gTurn485ReceiveMode;
/* function */
void USARTMTK_to_UARTBLE_long(uint8_t *BufferPtr, uint32_t Length);
void UARTBLE_to_USARTMTK_long(uint8_t *BufferPtr, uint32_t Length);
