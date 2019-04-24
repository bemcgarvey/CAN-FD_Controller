
#include "AppUSB.h"
#include "mcc_generated_files/usb/usb.h"
#include <stdbool.h>
#include "mcc_generated_files/pin_manager.h"
#include "Buffers.h"
#include "Status.h"
#include <string.h>
#include "DMA.h"

void HandleHardwareConfig(void);
void HandleFilter(void);
void HandleFIFOLengths(void);

uint8_t ctrlBufferOut[8];
USB_HANDLE evenTxQHandle;
USB_HANDLE oddTxQHandle;
USB_HANDLE evenTxFHandle;
USB_HANDLE oddTxFHandle;

static int txQNextHead = 0;
static int txFNextHead = 0;
static bool secondPacket = false;
static volatile bool needHardwareConfig = false;
static volatile bool needFilter = false;
static volatile bool needFIFOLengths = false;
uint16_t filterNumber = 0;
bool clearFilter = false;

void AppUSBInitialize(void) {
    USBEnableEndpoint(RX_EP, USB_IN_ENABLED | USB_OUT_DISABLED | USB_HANDSHAKE_ENABLED | USB_DISALLOW_SETUP);
    USBEnableEndpoint(TXQ_EP, USB_OUT_ENABLED | USB_IN_DISABLED | USB_HANDSHAKE_ENABLED | USB_DISALLOW_SETUP);
    evenTxQHandle = USBTransferOnePacket(TXQ_EP, OUT_FROM_HOST, ((uint8_t *) & txQueueBuffer[txQHead]), 64);
    oddTxQHandle = USBTransferOnePacket(TXQ_EP, OUT_FROM_HOST, ((uint8_t *) & txQueueBuffer[txQHead]) + 64, 12);
    USBEnableEndpoint(TXF_EP, USB_OUT_ENABLED | USB_IN_DISABLED | USB_HANDSHAKE_ENABLED | USB_DISALLOW_SETUP);
    evenTxFHandle = USBTransferOnePacket(TXF_EP, OUT_FROM_HOST, ((uint8_t *) & txFIFOBuffer[txFHead]), 64);
    oddTxFHandle = USBTransferOnePacket(TXF_EP, OUT_FROM_HOST, ((uint8_t *) & txFIFOBuffer[txFHead]) + 64, 12);
}

void AppUSBTasks(void) {
    if (USBGetDeviceState() < CONFIGURED_STATE) {
        return;
    }
    if (USBIsDeviceSuspended() == true) {
        GreenLED_SetHigh();
        return;
    }
    if (rxTail != rxHead) {
        if (!USBHandleBusy(USBGetNextHandle(RX_EP, IN_TO_HOST))) {
            if (secondPacket) {
                USBTransferOnePacket(RX_EP, IN_TO_HOST, ((uint8_t *) & rxBuffer[rxTail]) + 64, 64);
                secondPacket = false;
                ++rxTail;
            } else {
                USBTransferOnePacket(RX_EP, IN_TO_HOST, (uint8_t *) & rxBuffer[rxTail], 64);
                if (rxBuffer[rxTail].len <= 64) {
                    ++rxTail;
                } else {
                    secondPacket = true;
                }
            }
            if (rxTail >= RX_BUFFER_COUNT) {
                rxTail = 0;
            }
        }
    }
    if (!USBHandleBusy(evenTxQHandle)) {
        txQNextHead = txQHead + 1;
        if (txQNextHead >= TXQ_BUFFER_COUNT) {
            txQNextHead = 0;
        }
        if (txQNextHead == txQTail) {
            status.systemErrors |= ERROR_USB_TX;
        }
        evenTxQHandle = USBTransferOnePacket(TXQ_EP, OUT_FROM_HOST, ((uint8_t *) & txQueueBuffer[txQNextHead]), 64);
    }
    if (!USBHandleBusy(oddTxQHandle)) {
        oddTxQHandle = USBTransferOnePacket(TXQ_EP, OUT_FROM_HOST, ((uint8_t *) & txQueueBuffer[txQNextHead]) + 64, 12);
        txQHead = txQNextHead;
    }
    if (!USBHandleBusy(evenTxFHandle)) {
        txFNextHead = txFHead + 1;
        if (txFNextHead >= TXF_BUFFER_COUNT) {
            txFNextHead = 0;
        }
        if (txFNextHead == txFTail) {
            status.systemErrors |= ERROR_USB_TX;
        }
        evenTxFHandle = USBTransferOnePacket(TXF_EP, OUT_FROM_HOST, ((uint8_t *) & txFIFOBuffer[txFNextHead]), 64);
    }
    if (!USBHandleBusy(oddTxFHandle)) {
        oddTxFHandle = USBTransferOnePacket(TXF_EP, OUT_FROM_HOST, ((uint8_t *) & txFIFOBuffer[txFNextHead]) + 64, 12);
        txFHead = txFNextHead;
    }
    if (needHardwareConfig) {
        HandleHardwareConfig();
    }
    if (needFilter) {
        HandleFilter();
    }
    if (needFIFOLengths) {
        HandleFIFOLengths();
    }
}

void inline __attribute__((always_inline)) GotHardwareConfig(void) {
    needHardwareConfig = true;
}

void HandleHardwareConfig(void) {
    while (DMAState != DMA_IDLE);
    bool success = MCP2517FDChangeMode(CONFIG_MODE);
    if (!success) {
        status.systemErrors |= ERROR_MODE_CHANGE;
        return;
    }
    MCP2517FDSetBauds(ctrlBufferOut[1], ctrlBufferOut[2]);
    success = MCP2517FDChangeMode(ctrlBufferOut[0]);
    if (!success) {
        status.systemErrors |= ERROR_MODE_CHANGE;
    }
    needHardwareConfig = false;
    USBCtrlEPAllowStatusStage();
}

void inline __attribute__((always_inline)) GotFilter(void) {
    needFilter = true;
}

void HandleFilter(void) {
    while (DMAState != DMA_IDLE);
    if (clearFilter) {
        if (filterNumber > 32) {
            MCP2517FDClearAllFilters();
            //Set default filter - all messages
            MCP2517FDSetFilter(0, 0, EID, 0, EID, false, 2);
        } else {
            MCP2517FDClearFilter(filterNumber);
        }
    } else {
        uint32_t id;
        uint32_t mask;
        IDType type = SID_11;
        bool MIDE = false;
        mask = *(uint32_t *) ctrlBufferOut;
        if (mask & 0x40000000) {
            MIDE = true;
        }
        mask &= 0x1fffffff;
        id = *(uint32_t *) & ctrlBufferOut[4];
        if (id & 0x40000000) {
            type = EID;
        }
        id &= 0x1fffffff;
        MCP2517FDSetFilter(filterNumber, id, type, mask, type, MIDE, 2);
    }
    needFilter = false;
    USBCtrlEPAllowStatusStage();
}

void inline __attribute__((always_inline)) GotFIFOLengths(void) {
    needFIFOLengths = true;
}

void HandleFIFOLengths(void) {
    while (DMAState != DMA_IDLE);
    OperatingMode currentMode = status.currentMode;
    bool success = MCP2517FDChangeMode(CONFIG_MODE);
    if (!success) {
        status.systemErrors |= ERROR_MODE_CHANGE;
        return;
    }
    uint8_t reg = MCP2517FDRead8bitRegister(C1CON + 2);
    if (ctrlBufferOut[0] > 0) {
        MCP2517FDConfigFIFO(0, TX_FIFO, P_64, ctrlBufferOut[0], 31, false, false, INT_NONE);
        reg |= 0x10;
    } else {
        reg &= ~0x10;
    }
    MCP2517FDWrite8bitRegister(C1CON + 2, reg);
    hardware.txQLength = ctrlBufferOut[0];
    MCP2517FDConfigFIFO(1, TX_FIFO, P_64, ctrlBufferOut[1], 0, false, false, INT_NONE);
    hardware.txFIFOLength = ctrlBufferOut[1];
    MCP2517FDConfigFIFO(2, RX_FIFO, P_64, ctrlBufferOut[2], 0, false, true, INT_RX_NOT_EMPTY | INT_RX_OVF);
    hardware.rxFIFOLength = ctrlBufferOut[2];
    success = MCP2517FDChangeMode(currentMode);
    if (!success) {
        status.systemErrors |= ERROR_MODE_CHANGE;
    }
    needFIFOLengths = false;
    USBCtrlEPAllowStatusStage();
}