
#include "mcc_generated_files/spi2.h"
#include "mcc_generated_files/pin_manager.h"
#include "MCP2517FD.h"
#include "Buffers.h"
#include "Status.h"

uint32_t MCP2517FDRead32bitRegister(uint16_t address) {
    uint8_t tx[2];
    uint32_t rx;
    tx[0] = (READ << 4) | (address >> 8 & 0x0f);
    tx[1] = address;
    SS2OUT_SetLow();
    SPI2_Exchange8bitBuffer(tx, 2, NULL);
    SPI2_Exchange8bitBuffer(NULL, 4, (uint8_t *) &rx);
    SS2OUT_SetHigh();
    return rx;
}

uint8_t MCP2517FDRead8bitRegister(uint16_t address) {
    uint8_t tx[2];
    uint8_t reg;
    tx[0] = (READ << 4) | (address >> 8 & 0x0f);
    tx[1] = address;
    SS2OUT_SetLow();
    SPI2_Exchange8bitBuffer(tx, 2, NULL);
    reg = SPI2_Exchange8bit(0);
    SS2OUT_SetHigh();
    return reg;
}

void MCP2517FDWrite32bitRegister(uint16_t address, uint32_t value) {
    uint8_t tx[2];
    tx[0] = (WRITE << 4) | (address >> 8 & 0x0f);
    tx[1] = address;
    SS2OUT_SetLow();
    SPI2_Exchange8bitBuffer(tx, 2, NULL);
    SPI2_Exchange8bitBuffer((uint8_t *) & value, 4, NULL);
    SS2OUT_SetHigh();
}

void MCP2517FDWrite8bitRegister(uint16_t address, uint8_t value) {
    uint8_t tx[2];
    tx[0] = (WRITE << 4) | (address >> 8 & 0x0f);
    tx[1] = address;
    SS2OUT_SetLow();
    SPI2_Exchange8bitBuffer(tx, 2, NULL);
    SPI2_Exchange8bit(value);
    SS2OUT_SetHigh();
}

void MCP2517FDReset(void) {
    uint8_t tx[] = {0, 0};
    SS2OUT_SetLow();
    SPI2_Exchange8bitBuffer(tx, 2, NULL);
    SS2OUT_SetHigh();
}

uint8_t MCP2517FDConfig(void) {
    MCP2517FDConfigurePins();
    MCP2517FDChangeMode(CONFIG_MODE);
    MCP2517FDSetBauds(N_1M, D_2M);
    uint8_t reg = 0x10;
    MCP2517FDWrite8bitRegister(C1CON + 2, reg); //enable tx queue and disable tef
    MCP2517FDConfigFIFO(0, TX_FIFO, P_64, 8, 31, false, false, INT_NONE);
    MCP2517FDConfigFIFO(1, TX_FIFO, P_64, 7, 0, false, false, INT_NONE);
    MCP2517FDConfigFIFO(2, RX_FIFO, P_64, 12, 0, false, true, INT_RX_NOT_EMPTY | INT_RX_OVF);
    //enable interrupts
    MCP2517FDEnableSystemInterrupts(INT_RX | INT_INVALID_MESSAGE | INT_CAN_BUS_ERROR
            | INT_SYSTEM_ERROR | INT_RX_OVF_ERROR);
    MCP2517FDChangeMode(NORMAL_MODE);
    MCP2517FDWrite32bitRegister(C1TSCON, 0x00010027);
    return 0;
}

void MCP2517FDWaitForOsc(void) {
    uint8_t rdy;
    do {
        rdy = MCP2517FDRead8bitRegister(OSC + 1);
    } while ((rdy & 0x04) == 0);
}

bool MCP2517FDChangeMode(OperatingMode mode) {
    uint8_t reg;
    unsigned int backupCount; 
    mode &= 0x07;
    reg = MCP2517FDRead8bitRegister(C1CON + 3);
    reg &= 0b11111000;
    reg |= mode;
    MCP2517FDWrite8bitRegister(C1CON + 3, reg);
    backupCount = _CP0_GET_COUNT();
    _CP0_SET_COUNT(0);
    do {
        reg = MCP2517FDRead8bitRegister(C1CON + 2);
        reg >>= 5;
        if (_CP0_GET_COUNT() > 48000000) {
            break;
        }
    } while (reg != mode);
    _CP0_SET_COUNT(backupCount + _CP0_GET_COUNT());
    if (reg != mode) {
        status.currentMode = reg;
        return false;
    }
    status.currentMode = mode;
    return true;
}

OperatingMode MCP2517FDGetCurrentMode(void) {
    uint8_t reg;
    reg = MCP2517FDRead8bitRegister(C1CON + 2);
    reg >>= 5;
    return reg;
}

void MCP2517FDSetNominalBaud(uint8_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw) {
    uint32_t reg = 0;
    reg = brp;
    reg <<= 8;
    reg |= tseg1;
    reg <<= 8;
    reg |= (tseg2 & 0x7f);
    reg <<= 8;
    reg |= (sjw & 0x7f);
    MCP2517FDWrite32bitRegister(C1NBTCFG, reg);
}

void MCP2517FDSetDataBaud(uint8_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw, uint8_t tdco) {
    uint32_t reg = 0;
    reg = brp;
    brp <<= 8;
    reg |= (tseg1 & 0x1f);
    reg <<= 8;
    reg |= (tseg2 & 0x0f);
    reg <<= 8;
    reg |= (sjw & 0x0f);
    MCP2517FDWrite32bitRegister(C1DBTCFG, reg);
    MCP2517FDWrite8bitRegister(C1TDC + 1, (tdco & 0x7f));
    MCP2517FDWrite8bitRegister(C1TDC + 2, 0x02);
}

void MCP2517FDConfigFIFO(uint8_t fifoNumber, FifoType type, PayloadSize payloadSize, uint8_t fifoSize, uint8_t priority, bool rtr, bool timeStamp, uint8_t interrupts) {
    uint16_t address = C1TXQCON + 12 * fifoNumber;
    uint8_t reg;
    fifoSize -= 1;
    reg = (payloadSize << 5) | fifoSize;
    MCP2517FDWrite8bitRegister(address + 3, reg);
    reg = 0x60 | (priority & 0x1f);
    MCP2517FDWrite8bitRegister(address + 2, reg);
    reg = 0;
    if (type == TX_FIFO) {
        reg |= 0x80;
    }
    if (timeStamp) {
        reg |= 0x20;
    }
    if (rtr) {
        reg |= 0x40;
    }
    reg |= (interrupts & 0x1f);
    MCP2517FDWrite8bitRegister(address, reg);
}

bool MCP2517FDTransmitMessage(uint8_t fifoNumber, TXBuffer *txBuffer, bool txNow) {
    uint16_t fifoCON = C1TXQCON + 12 * (fifoNumber);
    uint16_t fifoSTA = C1TXQSTA + 12 * (fifoNumber);
    uint16_t fifoOUA = C1TXQUA + 12 * (fifoNumber);
    uint8_t reg;
    reg = MCP2517FDRead8bitRegister(fifoSTA);
    if (!(reg & 0x01)) {
        return false; //FIFO is full
    }
    uint16_t address = RAM_BASE + MCP2517FDRead32bitRegister(fifoOUA);
    uint16_t size;
    if (txBuffer->DLC == 0) {
        size = 0;
    } else if (txBuffer->DLC <= 4) {
        size = 4;
    } else if (txBuffer->DLC <= 8) {
        size = 8;
    } else {
        switch (txBuffer->DLC) {
            case 9: size = 12;
                break;
            case 10: size = 16;
                break;
            case 11: size = 20;
                break;
            case 12: size = 24;
                break;
            case 13: size = 32;
                break;
            case 14: size = 48;
                break;
            case 15: size = 64;
                break;
        }
    }
    size += 8;
    MCP2517FDWriteRam(address, (uint8_t *) txBuffer, size);
    reg = MCP2517FDRead8bitRegister(fifoCON + 1);
    if (txNow) {
        reg |= 0x03;
    } else {
        reg |= 0x01;
    }
    MCP2517FDWrite8bitRegister(fifoCON + 1, reg); //increment fifo and request transmit
    return true;
}

void MCP2517FDWriteRam(uint16_t address, uint8_t *buffer, uint16_t bytes) {
    uint8_t tx[2];
    tx[0] = (WRITE << 4) | (address >> 8 & 0x0f);
    tx[1] = address;
    SS2OUT_SetLow();
    SPI2_Exchange8bitBuffer(tx, 2, NULL);
    SPI2_Exchange8bitBuffer(buffer, bytes, NULL);
    SS2OUT_SetHigh();
}

void MCP2517FDReadRam(uint16_t address, uint8_t *buffer, uint16_t bytes) {
    uint8_t tx[2];
    tx[0] = (READ << 4) | (address >> 8 & 0x0f);
    tx[1] = address;
    SS2OUT_SetLow();
    SPI2_Exchange8bitBuffer(tx, 2, NULL);
    SPI2_Exchange8bitBuffer(NULL, bytes, buffer);
    SS2OUT_SetHigh();
}

uint32_t FormatID(uint32_t id, IDType type) {
    uint32_t nid = 0;
    if (type == SID_11) {
        nid = id & 0x7ff;
    } else if (type == SID_12) {
        nid = (id & 0x7ff) >> 1;
        nid |= (id & 0x1) << 29;
    } else {
        nid = (id >> 18) & 0x7ff;
        nid |= (id & 0x3ffff) << 11;
    }
    return nid;
}

void MCP2517FDSetFilter(uint8_t filterNumber, uint32_t filterID, IDType filterIDType, uint32_t maskID, IDType maskIDType, bool MIDE, uint8_t fifoNumber) {
    uint16_t address = C1FLTOBJ0 + 8 * filterNumber;
    uint32_t reg;
    uint16_t enable = C1FLTCON0 + (filterNumber / 4) * 4;
    enable += filterNumber % 4;
    MCP2517FDWrite8bitRegister(enable, 0); //Turn off filter if on
    reg = FormatID(filterID, filterIDType);
    if (filterIDType == EID) {
        reg |= 0x40000000;
    }
    MCP2517FDWrite32bitRegister(address, reg);
    reg = FormatID(maskID, maskIDType);
    if (MIDE) {
        reg |= 0x40000000;
    }
    address = C1MASK0 + 8 * filterNumber;
    MCP2517FDWrite32bitRegister(address, reg);
    fifoNumber &= 0x1f;
    fifoNumber |= 0x80;
    MCP2517FDWrite8bitRegister(enable, fifoNumber);
}

void MCP2517FDClearFilter(uint8_t filterNumber) {
    uint16_t enable = C1FLTCON0 + (filterNumber / 4) * 4;
    enable += filterNumber % 4;
    MCP2517FDWrite8bitRegister(enable, 0);    
}

void MCP2517FDClearAllFilters(void) {
    uint8_t i;
    for (i = 0; i < 32; ++i) {
        MCP2517FDClearFilter(i);
    }
}

bool MCP2517FDReceiveMessage(uint8_t fifoNumber, RXBuffer *rxBuffer) {
    uint16_t fifoCON = C1TXQCON + 12 * (fifoNumber);
    uint16_t fifoSTA = C1TXQSTA + 12 * (fifoNumber);
    uint16_t fifoOUA = C1TXQUA + 12 * (fifoNumber);
    uint8_t reg;
    reg = MCP2517FDRead8bitRegister(fifoSTA);
    if (!(reg & 0x01)) {
        return false; //FIFO is empty
    }
    uint16_t address = RAM_BASE + MCP2517FDRead32bitRegister(fifoOUA);
    uint8_t *buff = (uint8_t *) rxBuffer;
    MCP2517FDReadRam(address, buff, 8);
    uint16_t size;
    if (rxBuffer->DLC == 0) {
        size = 0;
    } else if (rxBuffer->DLC <= 4) {
        size = 4;
    } else if (rxBuffer->DLC <= 8) {
        size = 8;
    } else {
        switch (rxBuffer->DLC) {
            case 9: size = 12;
                break;
            case 10: size = 16;
                break;
            case 11: size = 20;
                break;
            case 12: size = 24;
                break;
            case 13: size = 32;
                break;
            case 14: size = 48;
                break;
            case 15: size = 64;
                break;
        }
    }
    buff += 8;
    address += 8;
    MCP2517FDReadRam(address, buff, size);
    MCP2517FDWrite8bitRegister(fifoCON + 1, 0x01); //increment FIFO
    return true;
}

int MCP2517FDReceiveMessageWithTimestamp(uint8_t fifoNumber, RXBufferWithTimeStamp *rxBuffer) {
    uint16_t fifoCON = C1TXQCON + 12 * (fifoNumber);
    uint16_t fifoSTA = C1TXQSTA + 12 * (fifoNumber);
    uint16_t fifoOUA = C1TXQUA + 12 * (fifoNumber);
    uint8_t reg;
    reg = MCP2517FDRead8bitRegister(fifoSTA);
    if (!(reg & 0x01)) {
        return 0; //FIFO is empty
    }
    uint16_t address = RAM_BASE + MCP2517FDRead32bitRegister(fifoOUA);
    uint8_t *buff = (uint8_t *) rxBuffer;
    MCP2517FDReadRam(address, buff, 12);
    uint16_t size;
    if (rxBuffer->DLC == 0) {
        size = 0;
    } else if (rxBuffer->DLC <= 4) {
        size = 4;
    } else if (rxBuffer->DLC <= 8) {
        size = 8;
    } else {
        switch (rxBuffer->DLC) {
            case 9: size = 12;
                break;
            case 10: size = 16;
                break;
            case 11: size = 20;
                break;
            case 12: size = 24;
                break;
            case 13: size = 32;
                break;
            case 14: size = 48;
                break;
            case 15: size = 64;
                break;
        }
    }
    buff += 12;
    address += 12;
    MCP2517FDReadRam(address, (uint8_t *) buff, size);
    MCP2517FDWrite8bitRegister(fifoCON + 1, 0x01); //increment FIFO
    return size + 12;
}

void MCP2517FDEnableSystemInterrupts(uint16_t flags) {
    uint32_t reg;
    reg = MCP2517FDRead32bitRegister(C1INT);
    reg |= (((uint32_t) flags) << 16);
    MCP2517FDWrite32bitRegister(C1INT, reg);
}

void MCP2517FDDisableSystemInterrupts(uint16_t flags) {
    uint32_t reg;
    reg = MCP2517FDRead32bitRegister(C1INT);
    reg &= (~(uint32_t)flags) << 16;
    MCP2517FDWrite32bitRegister(C1INT, reg);
}

void MCP2517FDClearSystemInterrupts(uint16_t flags) {
    uint32_t reg;
    reg = MCP2517FDRead32bitRegister(C1INT);
    reg &= ~(uint32_t)flags;
    MCP2517FDWrite32bitRegister(C1INT, reg);
}

void MCP2517FDConfigurePins(void) {
    uint32_t reg = 0x00000300;
    MCP2517FDWrite32bitRegister(IOCON, reg);
}

inline uint8_t DLCToLength(uint8_t dlc) {
    static const uint8_t sizes[] = {0, 4, 4, 4, 4, 8, 8, 8, 8, 12, 16, 20, 24, 32, 48, 64};
    return sizes[dlc];
}

//TODO Add some more nominal baud rates?
bool MCP2517FDSetBauds(NominalBaudRates nominalBaud, DataBaudRates dataBaud) {
    static const uint8_t nBauds[][4] = {
        {3, 158, 39, 39},
        {1, 158, 39, 39},
        {0, 254, 63, 63},
        {0, 126, 31, 31},
        {0, 62, 15, 15},
        {0, 38, 9, 9},
        {0, 30, 7, 7}};
    static const uint8_t dBauds[][5] = {
        {1, 30, 7, 7, 62},
        {0, 30, 7, 7, 31},
        {0, 14, 3, 3, 15},
        {0, 4, 1, 1, 5},
        {0, 2, 0, 0, 3}};
    
    if (MCP2517FDGetCurrentMode() != CONFIG_MODE) {
        return false;
    }
    MCP2517FDSetNominalBaud(nBauds[nominalBaud][0], nBauds[nominalBaud][1], nBauds[nominalBaud][2], nBauds[nominalBaud][3]);
    MCP2517FDSetDataBaud(dBauds[dataBaud][0], dBauds[dataBaud][1], dBauds[dataBaud][2], dBauds[dataBaud][3], dBauds[dataBaud][4]);
    hardware.nominalBaud = nominalBaud;
    hardware.dataBaud = dataBaud;
    return true;
}

void MCP2517FDClearFIFOErrors(uint8_t fifoNumber) {
    uint16_t fifoSTA = C1TXQSTA + 12 * (fifoNumber);
    MCP2517FDWrite8bitRegister(fifoSTA, 0);
}

static OperatingMode preSleepMode;

void MCP2517FDSleep(void) {
    preSleepMode = status.currentMode;
    MCP2517FDChangeMode(SLEEP_MODE);
    uint8_t reg;
    do {
        reg = MCP2517FDRead8bitRegister(OSC);
    } while (reg & 0x04 == 0);
}

void MCP2517FDWakeFromSleep(void) {
    uint8_t reg;
    reg = MCP2517FDRead8bitRegister(OSC);
    reg &= 0xfb;
    MCP2517FDWrite8bitRegister(OSC, reg);
    MCP2517FDWaitForOsc();
    MCP2517FDChangeMode(preSleepMode);
}