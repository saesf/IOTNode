/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __QUEUE_H
#define __QUEUE_H

#ifdef __cplusplus
 extern "C" {
#endif 

#define BUFFER_Q_SIZE 32


#define DMA_SEND_COMPLETE		1
#define DMA_RECEIVE_COMPLETE	2
#define WIFI_READY				3

typedef struct {
    char buffer[BUFFER_Q_SIZE];
    int head;
    int tail;
    int maxLen;
} circBuf_t;

extern circBuf_t emptyBufferQueue;
extern circBuf_t fullBufferQueue;
extern circBuf_t sensorBufferQueue;

extern void Qinit(circBuf_t *bufferQ);
extern char enqueue(circBuf_t *bufferQ, unsigned char bufferIndex);
extern char dequeue(circBuf_t *bufferQ, unsigned char *bufferIndex);

#ifdef __cplusplus
}
#endif

#endif /* __QUEUE_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
