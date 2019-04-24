
#include "mcc_generated_files/mcc.h"
#include "MCP2517FD.h"
#include "AppUSB.h"
#include "DMA.h"
#include "Buffers.h"
#include "mcc_generated_files/pin_manager.h"
#include "Status.h"

/*
                         Main application
 */
int main(void) {
    int i;
    // initialize the device
    SYSTEM_Initialize();
    PMDInit();
    InitDMA();
    MCP2517FDWaitForOsc();
    MCP2517FDReset();
    MCP2517FDConfig();
    MCP2517FDSetFilter(0, 0, EID, 0, EID, false, 2);
    BlueLED_SetHigh();
    while (1) {
        if (DMAState == DMA_IDLE) {
            if (PORTBbits.RB2 == 0) {
                ReceiveMessageWithTimestampDMA(&rxBuffer[rxHead]);
            } else if (PORTBbits.RB15 == 0) {
                HandleSystemInterrupt();
            } else if (txQTail != txQHead) {
                SendMessageDMA(&txQueueBuffer[txQTail].msg, 0, 
                        txQueueBuffer[txQTail].sendNow,  
                        txQueueBuffer[txQTail].len);
            } else if (txFTail != txFHead) {
                SendMessageDMA(&txFIFOBuffer[txFTail].msg, 1, true,  
                        txFIFOBuffer[txFTail].len);
            } else {
                CheckBusState();
            }
        }
        AppUSBTasks();
    }
}

/**
 End of File
 */