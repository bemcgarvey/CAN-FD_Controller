#include "Status.h"
#include "MCP2517FD.h"

StatusInfo status = {0, 0, 0, 0, 0, 0};
HardwareInfo hardware = {0, 0, 8, 7, 12, 0}; 

void HandleSystemInterrupt(void) {
    uint8_t flags = MCP2517FDRead8bitRegister(C1INT);
    status.errorFlags |= flags;
    if (flags & INT_RX_OVF_ERROR) {
        MCP2517FDClearFIFOErrors(2);
    }
    MCP2517FDClearSystemInterrupts(INT_ALL);
}

void CheckBusState(void) {
    uint32_t reg;
    reg = MCP2517FDRead32bitRegister(C1TREC);
    status.rxErrors = reg;
    reg >>= 8;
    status.txErrors = reg;
    reg >>= 8;
    status.busState = reg;
}