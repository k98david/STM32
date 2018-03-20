#include <stm32f10x.h>
#include <string.h>
#include "stdbool.h"   //for bool type
#include "QuanProcessCMD.h"
#include "config.h"
#include "QUart.h"
#include "Qevent_handler.h"  
#include "stm32_eval.h"  

#define CommandLength BUFFERSIZE


CommandType_t commandList[5];

uint8_t temp[BUFFERSIZE]={0};


UartRX_Port_t UAART_RX_Port=0;

CommandType_t bleCommandList[5];
uint8_t ATT1[]={"AT1="};
uint8_t ATT2[]={"AT2="};

uint8_t cmdIndex0=0;
uint8_t uart0GotCR=0; //will be 1 when CR(0x0D is encountered.
bool LED=true;
/* function */
uint8_t processATTX(char * cmdStr);
uint8_t processoBuzzer(char * cmdStr);
uint8_t processLed(char * cmdStr);
uint8_t processATT1(char * cmdStr);
uint8_t processATT2(char * cmdStr);

char command0[CommandLength];//temporary storage for UART0 command
uint8_t AT_TransparentTemp[BUFFERSIZE]={0};
uint8_t Len,y=0,Len2=0,Len3=0;
//bool BLE_Test=false;
uint8_t convertASCII2HexH(uint8_t *aWord);
uint8_t convertASCII2HexL(uint8_t *aWord);
uint8_t Rx3ProcessCMD(void)
{
	uint8_t i=0,x=0;
  uint8_t gotCommand=0;
  uint8_t datax=0;
  //char command0[CommandLength]=0;//temporary storage for UART0 command
	
  //for UART3(BLE)->UART5(MTK)
	while(RxHead3 != RxTail3){
		    //UAART_RX_Port=UARTRX_BLE;  //mark UART3 RX 
        datax=RxBuffer_BLE[RxTail3++];
        command0[cmdIndex0++] =datax;		  
        if (RxTail3>=RxBufferSize3){
            RxTail3=0;
        }
				/*
        if (cmdIndex0>=CommandLength){ //command too long
            cmdIndex0=0;
            break;
        }
				*/
        switch (datax){

            case 0x0D:
                uart0GotCR=1;
            break;
            
            case 0x0A:
              if (uart0GotCR){
                  command0[cmdIndex0-2]='\0';
                  gotCommand=1;
                  uart0GotCR=0;	
                  break;
              }
            break;
        default:
            uart0GotCR=0; //CR should be followed immediately by a LF
            break;
        }
	}

	if (gotCommand){
        char *pch=0;				
				pch = strtok(command0,"=");        
				cmdIndex0=0;
		   // UAART_RX_Port=UARTRX_BLE;  //mark UART3 RX 
			//	memcpy(temp,pch, 80); 	
				for (i=0; i< 5; i++){
						if (strcmp(pch, commandList[i].commandName)==0){
								pch = strtok(NULL, "\n");
								commandList[i].commandFunction(pch);
						}
				}
				/*
				for(x=0;x<CommandLength;x++){
				    command0[x]=0;
				}
				*/
    }
}

uint8_t Rx5ProcessCMD(void)
{
  uint8_t i=0,x=0;
  uint8_t gotCommand=0;
  uint8_t datax;
	//char command0[CommandLength];//temporary storage for UART0 command
  //for UART5(MTK)->UART3(BLE)
	while(RxHead5 != RxTail5){
//		    UAART_RX_Port=UARTRX_MTK;  //mark UART5
        datax=RxBuffer_MTK[RxTail5++];
        command0[cmdIndex0++] =datax;		  
        if (RxTail5>=RxBufferSize5){
            RxTail5=0;
        }
        if (cmdIndex0>=CommandLength){ //command too long
            cmdIndex0=0;
            break;
        }
        switch (datax){

            case 0x0D:
                uart0GotCR=1;
            break;
            
            case 0x0A:
              if (uart0GotCR){
                  command0[cmdIndex0-2]='\0';
                  gotCommand=1;
                  uart0GotCR=0;	
                  break;
              }
            break;
        default:
            uart0GotCR=0; //CR should be followed immediately by a LF
            break;
        }
	}
	if (gotCommand){
				char *pch=0;
				memcpy(temp,&command0[4],cmdIndex0-6);
				pch = strtok(command0,"=");        
				cmdIndex0=0;
				for (i=0; i< 5; i++){
						if (strcmp(pch, commandList[i].commandName)==0){
								//temp = strtok(NULL, " ");
								commandList[i].commandFunction(temp);
						}
				}
				/*
				for(x=0;x<CommandLength;x++){
				    command0[x]=0;
				}
				*/
    }
}

uint8_t Rx2ProcessCMD(void)
{	
	uint8_t i=0,x=0;
  uint8_t gotCommand=0;
  uint8_t datax;
	//char command0[CommandLength];//temporary storage for UART0 command
  //for UART5(MTK)->UART3(BLE)
	while(RxHead2 != RxTail2){
		  //  UAART_RX_Port=UARTRX_Zigbee;  //mark UART5
        datax=RxBuffer_Z200[RxTail2++];
        command0[cmdIndex0++] =datax;		  
        if (RxTail2>=RxBufferSize2){
            RxTail2=0;
        }
        if (cmdIndex0>=CommandLength){ //command too long
            cmdIndex0=0;
            break;
        }
        switch (datax){

            case 0x0D:
                uart0GotCR=1;
            break;
            
            case 0x0A:
              if (uart0GotCR){
                  command0[cmdIndex0-2]='\0';
                  gotCommand=1;
                  uart0GotCR=0;	
                  break;
              }
            break;
        default:
            uart0GotCR=0; //CR should be followed immediately by a LF
            break;
        }
	}
	if (gotCommand){
        char *pch=0;		
				memcpy(temp,&command0[4],cmdIndex0-6);		
				pch = strtok(command0,"=");        
				cmdIndex0=0;
				//memcpy(temp,pch, 80); 	
				for (i=0; i< 5; i++){
						if (strcmp(pch, commandList[i].commandName)==0){
								pch = strtok(NULL, "\n");
								commandList[i].commandFunction(temp);
						}
				}
				/*
				for(x=0;x<CommandLength;x++){
				    command0[x]=0;
				}
				*/
    }
}

uint8_t registerCommand(uint8_t cmdIndex, char* commandName, uint8_t (*funcPtr)(char *)){
	uint8_t result=1;
	strcpy(commandList[cmdIndex].commandName, commandName);
	commandList[cmdIndex].commandFunction=funcPtr;
	return result;
}

void initCommands(void){
	registerCommand(0, "ATX", processATTX);  //Transparent CMD
	registerCommand(1, "ATBuzz", processoBuzzer);  //Buzzer 
  registerCommand(2, "ATLED", processLed); //LED 
	registerCommand(3, "AT1", processATT1);   //TXT1
	registerCommand(4, "AT2", processATT2);   //TXT2
}



uint8_t processATTX(char * cmdStr){

  uint8_t add0d0a[2]={0x0d,0x0a};
  uint16_t ASCII=0;
//BLE_Test=false;
  switch(UAART_RX_Port){
		case UARTRX_MTK:
      Len=0;
			break;
		case UARTRX_BLE:
		//	BLE_Test=true;
	    Len=strlen(cmdStr);	
		  if(LED){
				STM_EVAL_LEDOn(LEDG);  //DEbug
				LED=false;
			}else{
				STM_EVAL_LEDOff(LEDG);  //DEbug
				LED=true;
			}
		
			memcpy(AT_TransparentTemp, ATT1,4);
			memcpy(&AT_TransparentTemp[4], cmdStr,Len);
			memcpy(&AT_TransparentTemp[Len+4], add0d0a,2);
		  			
		  UARTBLE_to_USARTMTK_long(AT_TransparentTemp,Len+2+4);	
			break;
		case UARTRX_Zigbee:
	    //Len2=strlen(cmdStr);	
	   //Len2=cmdStr[0];
		Len2=convertASCII2HexH(&cmdStr[0]);
		Len3=convertASCII2HexL(&cmdStr[1]);
		Len2=(Len2<<4)+Len3;
		/*
		  AT_TransparentTemp[0]=ATT2[0];
			AT_TransparentTemp[1]=ATT2[1];
			AT_TransparentTemp[2]=ATT2[2];
			AT_TransparentTemp[3]=ATT2[3];
		*/
			memcpy(AT_TransparentTemp, ATT2,4);

			memcpy(&AT_TransparentTemp[4], &cmdStr[0],Len2+2);
			memcpy(&AT_TransparentTemp[Len2+4+2], add0d0a,2);
		  
		  UART2_to_USARTMTK_long(AT_TransparentTemp,Len2+2+4+2);
			break;
	}
	/*
	if(!BLE_Test){
      Len=0;
	}
	*/
	/*
	if(UAART_RX_Port==UARTRX_MTK){  //choose Uart Port this is for BLE UART TX
	}else if(UAART_RX_Port==UARTRX_BLE){    //choose Uart Port this is for MTK UART TX
	    Len=strlen(cmdStr);	
		  if(LED){
				STM_EVAL_LEDOn(LEDR);  //DEbug
				LED=false;
			}else{
				STM_EVAL_LEDOff(LEDR);  //DEbug
				LED=true;
			}
		
			memcpy(AT_TransparentTemp, ATT1,4);
			memcpy(&AT_TransparentTemp[4], cmdStr,Len);
			memcpy(&AT_TransparentTemp[Len+4], add0d0a,2);
		  			
		  UARTBLE_to_USARTMTK_long(AT_TransparentTemp,Len+2+4);	
		//STM_EVAL_LEDOff(LEDR);  //DEbug
	}else if(UAART_RX_Port==UARTRX_Zigbee){    //choose Uart Port this is for MTK UART TX
	    Len2=cmdStr[2]+3;
		  AT_TransparentTemp[0]=ATT2[0];
			AT_TransparentTemp[1]=ATT2[1];
			AT_TransparentTemp[2]=ATT2[2];
			AT_TransparentTemp[3]=ATT2[3];
			AT_TransparentTemp[4]=ATT2[4];
		
			//memcpy(AT_TransparentTemp, ATT2,5);
			memcpy(&AT_TransparentTemp[5], cmdStr,Len2);
			memcpy(&AT_TransparentTemp[Len2+5], add0d0a,2);
		  
		  UART2_to_USARTMTK_long(AT_TransparentTemp,Len2+2+5);
	}

	*/

	
}


uint8_t processATT1(char * cmdStr){
	uint8_t Len,y=0;
			
  uint8_t add0d0a[2]={0x0d,0x0a};
	uint8_t AT_TransparentTemp[BUFFERSIZE]={0};
	//STM_EVAL_LEDOn(LEDR);  //DEbug
		
		  if(LED){
				STM_EVAL_LEDOn(LEDG);  //DEbug
				LED=false;
			}else{
				STM_EVAL_LEDOff(LEDG);  //DEbug
				LED=true;
			}
	
	Len=strlen(cmdStr);	

	memcpy(AT_TransparentTemp, cmdStr,Len);
	memcpy(&AT_TransparentTemp[Len], add0d0a,2);
	
	USARTMTK_to_UARTBLE_long(AT_TransparentTemp,Len+2);
					//STM_EVAL_LEDOff(LEDR);  //DEbug
}

uint8_t processATT2(char * cmdStr){
	uint8_t Len,y=0;

  uint8_t add0d0a[2]={0x0d,0x0a};
	uint8_t AT_TransparentTemp[BUFFERSIZE]={0};
		
		  if(LED){
				STM_EVAL_LEDOn(LEDR);  //DEbug
				LED=false;
			}else{
				STM_EVAL_LEDOff(LEDR);  //DEbug
				LED=true;
			}
			
	Len2=convertASCII2HexH(&cmdStr[4]);
	Len3=convertASCII2HexL(&cmdStr[5]);
	Len2=(Len2<<4)+Len3;	

	memcpy(AT_TransparentTemp, cmdStr,4);
	memcpy(&AT_TransparentTemp[4], &cmdStr[6],Len2);
	memcpy(&AT_TransparentTemp[Len2+4], add0d0a,2);
	
	USARTMTK_to_UART2_long(AT_TransparentTemp,Len2+2+4);
			
			/*
	Len=strlen(cmdStr);	

	memcpy(AT_TransparentTemp, cmdStr,Len);
	memcpy(&AT_TransparentTemp[Len], add0d0a,2);
	
	USARTMTK_to_UART2_long(AT_TransparentTemp,Len+2);
			*/
}


uint8_t processoBuzzer(char * cmdStr){
		uint8_t Len,y=0;

		uint8_t add0d0a[2]={0x0d,0x0a};
		uint8_t AT_TransparentTemp[BUFFERSIZE]={0};
		
	  Len=strlen(cmdStr);	
		memcpy(AT_TransparentTemp, ATT1,5);
		memcpy(&AT_TransparentTemp[5], cmdStr,Len);
		memcpy(&AT_TransparentTemp[Len+5], add0d0a,2);
		
		UARTBLE_to_USARTMTK_long(AT_TransparentTemp,Len+2+5);	
		
		sendEvent(evBuzzer);     //evBuzzer event
		sendEvent(evLEDFlash);    //flashing LED event 
}

uint8_t processLed(char * cmdStr){
	  /*char *pch;
	  uint8_t Temp[5]={0};
	  pch=strtok(cmdStr,"=");
	  pch=strtok(NULL,"");
    strcpy(Temp,pch);
	*/
	  if(cmdStr[0]=='G'){
			  if(cmdStr[1]=='1'){
						STM_EVAL_LEDOn(LEDG);
				}else if(cmdStr[1]=='0'){
					  STM_EVAL_LEDOff(LEDG);
				}
		}else if(cmdStr[0]=='R'){
			  if(cmdStr[1]=='1'){
						STM_EVAL_LEDOn(LEDR);
				}else if(cmdStr[1]=='0'){
					  STM_EVAL_LEDOff(LEDR);
				}
		}
		//sendEvent(evLEDFlash);    //flashing LED event 
}
uint8_t convertASCII2HexH(uint8_t *aWord){

	char abyte;

	if ( (aWord[0]>=0x30) & (aWord[0]<=0x39)){
		abyte = aWord[0]-0x30;
	}else{
		if ((aWord[0]>='A')&(aWord[0]<='F')){
			abyte = aWord[0]-'A'+10;
		}
	}
	return abyte;
}

uint8_t convertASCII2HexL(uint8_t *aWord){

	char abyte;

	if ( (aWord[0]>=0x30) & (aWord[0]<=0x39)){
		abyte = aWord[0]-0x30;
	}else{
		if ((aWord[0]>='A')&(aWord[0]<='F')){
			abyte = aWord[0]-'A'+10;
		}
	}
	return abyte;
}
