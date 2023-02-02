#include "queue.h"
#include "stdint.h"
#include "cmsis_gcc.h"

circBuf_t emptyBufferQueue;
circBuf_t fullBufferQueue;
circBuf_t sensorBufferQueue;
uint32_t prim;

void Qinit(circBuf_t *bufferQ){
	bufferQ->head=0;
	bufferQ->tail=0;
	bufferQ->maxLen=BUFFER_Q_SIZE;
}								

char enqueue(circBuf_t *bufferQ, unsigned char bufferIndex)
{
     // next is where head will point to after this write.
    int next = bufferQ->head + 1;
    if (next >= bufferQ->maxLen)
        next = 0;

    if (next == bufferQ->tail) // check if circular buffer is full
        return 0;       // and return with an error.

//    /* Read PRIMASK register, check interrupt status before you disable them */
//	/* Returns 0 if they are enabled, or non-zero if disabled */
//	prim = __get_PRIMASK();
//
//	/* Disable interrupts */
//	__disable_irq();

	bufferQ->buffer[bufferQ->head] = bufferIndex; // Load data and then move
	bufferQ->head = next;            // head to next data offset.

//    /* Enable interrupts back only if they were enabled before we disable it here in this function */
//	if (!prim) {
//		__enable_irq();
//	}

    return 1;  // return success to indicate successful push.
}

char dequeue(circBuf_t *bufferQ, unsigned char *bufferIndex)
{
	int next;
    // if the head isn't ahead of the tail, we don't have any characters
    if (bufferQ->head == bufferQ->tail) // check if circular buffer is empty
        return 0;          // and return with an error

    // next is where tail will point to after this read.
    next = bufferQ->tail + 1;
    if(next >= bufferQ->maxLen)
        next = 0;
//    /* Read PRIMASK register, check interrupt status before you disable them */
//	/* Returns 0 if they are enabled, or non-zero if disabled */
//	prim = __get_PRIMASK();
//
//	/* Disable interrupts */
//	__disable_irq();

	*bufferIndex = bufferQ->buffer[bufferQ->tail]; // Read data and then move

    bufferQ->tail = next;             // tail to next data offset.

//    /* Enable interrupts back only if they were enabled before we disable it here in this function */
//	if (!prim) {
//		__enable_irq();
//	}
    return 1;  // return success to indicate successful push.
}
