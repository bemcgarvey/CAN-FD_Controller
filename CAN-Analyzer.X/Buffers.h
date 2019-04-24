/* 
 * File:   Buffers.h
 * Author: Brad
 *
 * Created on January 9, 2018, 6:35 PM
 */

#ifndef BUFFERS_H
#define	BUFFERS_H

#include "MCP2517FD.h"
#include <stdint.h>
#include <stdbool.h>

#define RX_BUFFER_COUNT     60
#define TXQ_BUFFER_COUNT    40
#define TXF_BUFFER_COUNT    40

#ifdef	__cplusplus
extern "C" {
#endif
    typedef struct {
        uint8_t len;
        uint8_t reserved1;
        uint8_t reserved2;
        uint8_t reserved3;
        RXBufferWithTimeStamp msg;
        uint8_t filler[48];
    } RxBufferEntry;
    
    extern RxBufferEntry rxBuffer[RX_BUFFER_COUNT];
    extern int rxTail;
    extern volatile int rxHead;
    
    typedef struct {
        uint8_t len;
        bool sendNow;
        uint8_t reserved1;
        uint8_t reserved2;
        TXBuffer msg;
    } TxBufferEntry;
    
    extern TxBufferEntry txQueueBuffer[TXQ_BUFFER_COUNT];
    extern int txQTail;
    extern int txQHead;
    
    extern TxBufferEntry txFIFOBuffer[TXF_BUFFER_COUNT];
    extern int txFTail;
    extern int txFHead;


#ifdef	__cplusplus
}
#endif

#endif	/* BUFFERS_H */

