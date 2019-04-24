
#include <xc.h>
#include "DMA.h"
#include "MCP2517FD.h"
#include "sys/kmem.h"
#include "mcc_generated_files/pin_manager.h"
#include "Buffers.h"
#include "stdbool.h"
#include "Status.h"

#define RxFIFOCON     (C1TXQCON + 12 * 2)
#define RxFIFOOUA     (C1TXQUA + 12 * 2)
#define RxFIFOSTA     (C1TXQSTA + 12 * 2)

volatile DMAStates DMAState;

volatile uint8_t *currentBuffer;
const uint8_t txdummy[64];
uint8_t rxdummy[78];
volatile uint8_t dmaLength;
volatile bool transmitMessageNow;
volatile uint16_t currentFIFOCON;
int currentTxFIFO;

void InitDMA(void) {
    //Clear and disable interrupts Set priority
    IEC3bits.DMA0IE = 0;
    IEC3bits.DMA1IE = 0;
    IEC3bits.DMA3IE = 0;
    
    IPC25bits.DMA2IP = 6;
    IPC25bits.DMA2IS = 0;
    IFS3bits.DMA2IF = 0;
    IEC3bits.DMA2IE = 1;
    //DMA on
    DMACONbits.ON = 1;
    //Setup DMA0
    //    Dummy -> SPI2BUF
    //    12 bytes or payload size
    //    Trigger when buffer not full
    DCH0CONbits.CHAED = 1; //allow events when disabled
    DCH0CONbits.CHPRI = 2; //priority
    DCH0ECONbits.CHSIRQ = _SPI2_TX_VECTOR; //SPI2 Tx
    DCH0ECONbits.SIRQEN = 1;
    DCH0DSA = KVA_TO_PA(&SPI2BUF);
    DCH0DSIZ = 1;
    DCH0CSIZ = 1;
    //Setup DMA1
    //    SSP2BUF -> Dummy
    //    2 bytes
    //    Trigger when buffer not empty
    DCH1CONbits.CHAED = 0; //don't allow events when disabled
    DCH1CONbits.CHPRI = 2; //same priority as tx?
    DCH1ECONbits.CHSIRQ = _SPI2_RX_VECTOR; //SPI2 Rx
    DCH1ECONbits.SIRQEN = 1;
    DCH1DSA = KVA_TO_PA(rxdummy);
    DCH1SSA = KVA_TO_PA(&SPI2BUF);
    DCH1SSIZ = 1;
    DCH1CSIZ = 1;
    DCH1DSIZ = 2;
    //Setup DMA2
    //    SSP2BUF -> buffer
    //    12 bytes or payload size
    //    Trigger when buffer not empty
    //    Chain from DMA1
    DCH2CONbits.CHAED = 0; //don't allow events when disabled
    DCH2CONbits.CHPRI = 2; //priority
    DCH2ECONbits.CHSIRQ = _SPI2_RX_VECTOR; //SPI2 Rx
    DCH2ECONbits.SIRQEN = 1;
    DCH2CONbits.CHCHN = 1;
    DCH2SSA = KVA_TO_PA(&SPI2BUF);
    DCH2SSIZ = 1;
    DCH2CSIZ = 1;
    DCH2INTbits.CHDDIF = 0;
    DCH2INTbits.CHDDIE = 1; //interrupt when destination done
    //Setup DMA3
    //    user tx buffer -> SPI2BUF
    //    8 bytes + payload size
    //    Trigger when buffer not full
    DCH3CONbits.CHAED = 1; //allow events when disabled
    DCH3CONbits.CHPRI = 2; //priority
    DCH3ECONbits.CHSIRQ = _SPI2_TX_VECTOR; //SPI2 Tx
    DCH3ECONbits.SIRQEN = 1;
    DCH3DSA = KVA_TO_PA(&SPI2BUF);
    DCH3DSIZ = 1;
    DCH3CSIZ = 1;
    DMAState = DMA_IDLE;
}

void ReceiveMessageWithTimestampDMA(RxBufferEntry *rxBuffer) {
    uint8_t tx[2];
    uint16_t currentAddress = RAM_BASE + MCP2517FDRead32bitRegister(RxFIFOOUA);
    currentBuffer = (uint8_t *) & rxBuffer->msg;
    tx[0] = (READ << 4) | (currentAddress >> 8 & 0x0f);
    tx[1] = currentAddress;
    DCH0SSIZ = 12;
    DCH0SSA = KVA_TO_PA(txdummy);
    DCH2DSIZ = 12;
    DCH2DSA = KVA_TO_PA(currentBuffer);
    DCH1CONbits.CHEN = 1;
    SS2OUT_SetLow();
    SPI2BUF = tx[0];
    SPI2BUF = tx[1];
    DCH0CONbits.CHEN = 1;
    DMAState = DMA_WAIT_FOR_HEADER;
}

void SendMessageDMA(TXBuffer *txBuffer, int fifoNumber, bool txNow, uint8_t len) {
    uint8_t tx[2];
    if (DMAState != DMA_IDLE) {
        return;
    }
    uint16_t fifoSTA = C1TXQSTA + 12 * fifoNumber;
    uint8_t reg = MCP2517FDRead8bitRegister(fifoSTA);
    if (!(reg & 0x01)) {
        return; //FIFO is full
    }
    transmitMessageNow = txNow;
    currentFIFOCON = C1TXQCON + 12 * fifoNumber;
    currentTxFIFO = fifoNumber;
    uint16_t currentAddress = RAM_BASE + MCP2517FDRead32bitRegister(C1TXQUA + 12 * fifoNumber);
    DCH3SSIZ = len;
    DCH3SSA = KVA_TO_PA(txBuffer);
    DCH2DSIZ = len + 2;
    DCH2DSA = KVA_TO_PA(rxdummy);
    DCH2CONbits.CHEN = 1;
    tx[0] = (WRITE << 4) | (currentAddress >> 8 & 0x0f);
    tx[1] = currentAddress;
    SS2OUT_SetLow();
    SPI2BUF = tx[0];
    SPI2BUF = tx[1];
    DCH3CONbits.CHEN = 1;
    DMAState = DMA_WAIT_FOR_TX_DONE;
}

void __attribute__((vector(_DMA2_VECTOR), interrupt(IPL6SOFT))) DMA2ISR(void) {
    if (DMAState == DMA_WAIT_FOR_HEADER) {
        dmaLength = DLCToLength(currentBuffer[4] & 0x0f);
        if (dmaLength == 0) {
            SS2OUT_SetHigh();
            MCP2517FDWrite8bitRegister(RxFIFOCON + 1, 0x01); //increment FIFO
            rxBuffer[rxHead].len = 12;
            ++rxHead;
            if (rxHead >= RX_BUFFER_COUNT) {
                rxHead = 0;
            }
            if (rxHead == rxTail) {
                status.systemErrors |= ERROR_USB_RX;
            }
            GreenLED_Toggle();
            DMAState = DMA_IDLE;
        } else {
            currentBuffer += 12;
            DCH0SSIZ = dmaLength;
            DCH2DSA = KVA_TO_PA(currentBuffer);
            DCH2DSIZ = dmaLength;
            DCH2CONbits.CHEN = 1;
            DCH0CONbits.CHEN = 1;
            DMAState = DMA_WAIT_FOR_BODY;
        }
    } else if (DMAState == DMA_WAIT_FOR_BODY) {
        SS2OUT_SetHigh();
        MCP2517FDWrite8bitRegister(RxFIFOCON + 1, 0x01); //increment FIFO
        rxBuffer[rxHead].len = 16 + dmaLength;
        ++rxHead;
        if (rxHead >= RX_BUFFER_COUNT) {
            rxHead = 0;
        }
        if (rxHead == rxTail) {
            status.systemErrors |= ERROR_USB_RX;
        }
        GreenLED_Toggle();
        DMAState = DMA_IDLE;
    } else if (DMAState == DMA_WAIT_FOR_TX_DONE) {
        SS2OUT_SetHigh();
        if (transmitMessageNow) {
            MCP2517FDWrite8bitRegister(currentFIFOCON + 1, 0x03); //increment FIFO and tx
        } else {
            MCP2517FDWrite8bitRegister(currentFIFOCON + 1, 0x01); //increment FIFO
        }
        if (currentTxFIFO == 0) {
            ++txQTail;
            if (txQTail >= TXQ_BUFFER_COUNT) {
                txQTail = 0;
            }
        } else {
            ++txFTail;
            if (txFTail >= TXF_BUFFER_COUNT) {
                txFTail = 0;
            }
        }
        BlueLED_Toggle();
        DMAState = DMA_IDLE;
    }
    DCH2INTbits.CHDDIF = 0;
    IFS3bits.DMA2IF = 0;
}
