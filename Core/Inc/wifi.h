/*
 * wifi.h
 *
 *  Created on: 19 feb 2021
 *      Author: Raul Rosa
 */

#ifndef INC_WIFI_H_
#define INC_WIFI_H_

#include "stdint.h"

#define PAYLOAD_LEN 1460
extern int DMATransferCompleted;
extern int DMAReceiveCompleted;
extern char buffer[1000];
extern char bufferBig[1000];

#define SSIDcommand "C1=" SSID "\r\n"
#define PSWcommand "C2=" PSW "\r\n"                //command to set the password
#define SECcommad "C3=4\r\n"                      //security of the network WPA2 BEWARE, IF YOUR NETWORK IS NOT WPA2 THIS COMMAND SHOULD BE CHANGED
#define JoinRetryCountcommad "CB=10\r\n"
#define AutoConnectcommad "CC=3\r\n"
#define CONNECTcommand "C0\r\n"
#define SAVEcommand "Z1\r\n"

#define SocketCommand "P0=0\r\n"				//UDP
#define UDPCommand "P1=1\r\n"				//UDP
#define TCPcommand "P1=0\r\n" 				//TCP
#define IPcommand "P3=" IPgw "\r\n\n"       //Server IP address (D0 will perform a DNS lookup on and update P3 automatically)
#define PORTcommand "P4=" PORTgw "\r\n\n"	//Server port
#define STARTClientcommand "P6=1\r\n"          //Start the client
#define CLOSEClientcommand "P6=0\r\n"          //Stop client


#define SENDPacketSize "S1=1460\r"
#define STIMOEOUTcommand "S2=10\r\n\n"
#define SENDcommand "S3="
#define READBUFFERcomand "R1=100\r\n"
#define READcommand "R0\r\n"
#define RTimeoutcommand "R2=0\r\n"

#define SetSettingsSpace "Z3=1\r\n"
#define SaveSettings "Z1\r\n"
#define SaveSettings "Z1\r\n"

#define DUMMY_LENGTH 100


extern int payloadLen;

void WifiInit();

void ReverseByteOrder(const uint16_t * src, int size, uint16_t * dst);
void SendPayload(char *payload,int size);
void GetAcknowldgement(void);
void WifiFSM();

#endif /* INC_WIFI_H_ */
