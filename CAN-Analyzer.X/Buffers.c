
#include "Buffers.h"

RxBufferEntry rxBuffer[RX_BUFFER_COUNT];
int rxTail = 0;
int volatile rxHead = 0;

TxBufferEntry txQueueBuffer[TXQ_BUFFER_COUNT];
int txQTail = 0;
int txQHead = 0;
    
TxBufferEntry txFIFOBuffer[TXF_BUFFER_COUNT];
int txFTail = 0;
int txFHead = 0;
