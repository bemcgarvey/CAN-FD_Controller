/* 
 * File:   DMA.h
 * Author: Brad
 *
 * Created on January 13, 2018, 2:31 PM
 */

#ifndef DMA_H
#define	DMA_H

#include "MCP2517FD.h"
#include "Buffers.h"

#ifdef	__cplusplus
extern "C" {
#endif

    typedef enum {
        DMA_IDLE, DMA_WAIT_FOR_HEADER, DMA_WAIT_FOR_BODY, DMA_WAIT_FOR_TX_DONE
    } DMAStates;
    extern volatile DMAStates DMAState;
    void InitDMA(void);
    void ReceiveMessageWithTimestampDMA(RxBufferEntry *rxBuffer);
    void SendMessageDMA(TXBuffer *txBuffer, int fifoNumber, bool txNow, uint8_t len);

#ifdef	__cplusplus
}
#endif

#endif	/* DMA_H */

