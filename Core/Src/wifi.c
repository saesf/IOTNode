/*
 * wifi.c
 *
 *  Created on: 19 feb 2021
 *      Author: Raul Rosa
 */


#include "wifi.h"
#include "config.h"
#include "main.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"


extern SPI_HandleTypeDef hspi3;

int wifiReady=0;
int flagReady=0;

int flagConnected=0;
int flagConnectedFinal=0;
int DMATransferCompleted=0;
int DMAReceiveCompleted=0;

char buffer[1000];
char bufferBig[1000];
char dummy[]={0x0a,0x0a};
char ready[]={0x15,0x15,0x0d,0x0a,0x3e,0x20};//ready buffer
int tmpIndex=0;
char commandbuffer[500];
char sendBuffer[256];

int indexdata=0;
int indexsensor=0;

char payloadsensors[1000];

char *payload;
int payloadLen;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hspi);
	DMATransferCompleted=1;
}




int checkWifiEvent(char * input, char *event, int size){

	int i=0;
	for(i=0;i<size;i++){
		if(input[i]!=event[i]){
			return 0;
		}
	}
	return 1;

}


int getWifiModuleReady(){
	while(!wifiReady);
	wifiReady=0;
	HAL_GPIO_WritePin(WIFI_CS_GPIO_Port,WIFI_CS_Pin,GPIO_PIN_RESET);
	do{
		HAL_SPI_TransmitReceive(&hspi3,(uint8_t *)dummy,(uint8_t *)buffer+tmpIndex,2/2,1);
		tmpIndex =(tmpIndex+2)%1000;
	}while(HAL_GPIO_ReadPin(WIFI_INT_GPIO_Port,WIFI_INT_Pin)==1);
	HAL_GPIO_WritePin(WIFI_CS_GPIO_Port,WIFI_CS_Pin,GPIO_PIN_SET);

	if(strncmp(buffer,ready,sizeof(ready))==0){
		return 1;
	}
	tmpIndex=0;
	return 0;

}

void sendCommand(char *command,int size){
	while(!wifiReady);
	wifiReady=0;
	memset(buffer,0,sizeof(buffer));
	memset(commandbuffer,0x0a,sizeof(commandbuffer));
	memcpy(commandbuffer, command, size);
//	ReverseByteOrder((uint16_t *)command, size, (uint16_t *)commandbuffer);
	HAL_GPIO_WritePin(WIFI_CS_GPIO_Port,WIFI_CS_Pin,GPIO_PIN_RESET);
	DMATransferCompleted=0;
	HAL_SPI_Transmit_DMA(&hspi3, (uint8_t *)commandbuffer, size);
	while(DMATransferCompleted==0);
	HAL_GPIO_WritePin(WIFI_CS_GPIO_Port,WIFI_CS_Pin,GPIO_PIN_SET);
	while(!wifiReady);
	memset(buffer,0,sizeof(buffer));
	HAL_GPIO_WritePin(WIFI_CS_GPIO_Port,WIFI_CS_Pin,GPIO_PIN_RESET);
	DMAReceiveCompleted=0;
	HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *)sendBuffer, (uint8_t *)buffer, 256/2);
	while(DMAReceiveCompleted==0);
	HAL_SPI_DMAStop(&hspi3);
	HAL_GPIO_WritePin(WIFI_CS_GPIO_Port,WIFI_CS_Pin,GPIO_PIN_SET);
	memset(bufferBig,0,sizeof(bufferBig));
	memcpy(bufferBig, buffer, 20);
//	ReverseByteOrder((uint16_t *)buffer, 20, (uint16_t *)bufferBig);
}

void SendPayload(char *payload,int size){
	HAL_GPIO_WritePin(WIFI_CS_GPIO_Port,WIFI_CS_Pin,GPIO_PIN_RESET);
	DMATransferCompleted=0;
	HAL_SPI_Transmit_DMA(&hspi3, (uint8_t *)payload, size/2);
}

void GetAcknowldgement()
{
	memset(buffer,0,512);
	HAL_GPIO_WritePin(WIFI_CS_GPIO_Port,WIFI_CS_Pin,GPIO_PIN_RESET);
	DMAReceiveCompleted=0;
	HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *)sendBuffer, (uint8_t *)buffer, 512/2);
}

void ReverseByteOrder(const uint16_t * src, int size, uint16_t * dst)
{
    for(int i = 0; i < size; i++)
//        dst[i] = __builtin_bswap16(src[i]);
    	dst[i] = src[i];
}

void WifiInit(){
	HAL_GPIO_WritePin(WIFI_RESET_GPIO_Port,WIFI_RESET_Pin,GPIO_PIN_RESET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(WIFI_RESET_GPIO_Port,WIFI_RESET_Pin,GPIO_PIN_SET);

	int result;
	do{
		result=getWifiModuleReady();
	}while(!result);

	//let's set the wifi settings
	sendCommand(SSIDcommand,strlen(SSIDcommand));
	sendCommand(PSWcommand,strlen(PSWcommand));
	sendCommand(SECcommad,strlen(SECcommad));
	sendCommand(JoinRetryCountcommad,strlen(JoinRetryCountcommad));
	sendCommand(AutoConnectcommad,strlen(AutoConnectcommad));
	sendCommand(CONNECTcommand,strlen(CONNECTcommand));
	sendCommand(SocketCommand,strlen(SocketCommand));
//	sendCommand(UDPCommand,strlen(UDPCommand));
	sendCommand(TCPcommand,strlen(TCPcommand));
	sendCommand(IPcommand, strlen(IPcommand));
	sendCommand(PORTcommand, strlen(PORTcommand));
	sendCommand(STARTClientcommand, strlen(STARTClientcommand));
	sendCommand(STIMOEOUTcommand, strlen(STIMOEOUTcommand));
	sendCommand(SENDPacketSize, strlen(SENDPacketSize));

//	sendCommand(SetSettingsSpace, strlen(SetSettingsSpace));
//	sendCommand(SaveSettings, strlen(SaveSettings));
	wifiState=1;
}

void WifiFSM()
{
	  if(wifiState==1 && wifiReady==1)//send payload
	  {
		  wifiReady=0;
		  SendPayload(payload,payloadLen);
		  wifiState=2;
	  }
	  else if(wifiState==2 && DMATransferCompleted==1)//wait for send to complete
	  {
		  HAL_GPIO_WritePin(WIFI_CS_GPIO_Port,WIFI_CS_Pin,GPIO_PIN_SET);
//		  wifiState=1;
		  wifiState=1;
	  }
//	  else if(wifiState==3 && wifiReady==1)//retrieve acknowledgement
//	  {
//		  DMATransferCompleted=0;
//		  GetAcknowldgement();
//		  wifiState=4;
//	  }
//	  else if(wifiState==4 && DMAReceiveCompleted==1)//ack received
//	  {
//			HAL_SPI_DMAStop(&hspi3);
//			HAL_GPIO_WritePin(WIFI_CS_GPIO_Port,WIFI_CS_Pin,GPIO_PIN_SET);
////			memset(bufferBig,0,sizeof(bufferBig));
////			ReverseByteOrder((uint16_t *)buffer, 128, (uint16_t *)bufferBig);
//			wifiState=1;
//	  }
}
