/* 
 * File:   MCP2517FD.h
 * Author: Brad
 *
 * Created on December 28, 2017, 8:35 AM
 */

#ifndef MCP2517FD_H
#define	MCP2517FD_H

#include <stdint.h>
#include <stdbool.h>

typedef union {
    struct __attribute__((packed)) {
        uint32_t id;
        unsigned int DLC : 4;
        unsigned int IDE : 1;
        unsigned int RTR : 1;
        unsigned int BRS : 1;
        unsigned int FDF : 1;
        unsigned int ESI : 1;
        unsigned int SEQ : 7;
        uint16_t empty;
        uint8_t data[64];
    };
    uint8_t bytes[72];
} TXBuffer;

typedef union {
    struct __attribute__((packed)) {
        uint32_t id;
        unsigned int DLC : 4;
        unsigned int IDE : 1;
        unsigned int RTR : 1;
        unsigned int BRS : 1;
        unsigned int FDF : 1;
        unsigned int ESI : 1;
        unsigned int : 2;
        unsigned int FILHIT : 5;
        uint16_t empty;
        uint8_t data[64];
    };
    uint8_t bytes[72];
} RXBuffer;

typedef union {
    struct __attribute__((packed)) {
        uint32_t id;
        unsigned int DLC : 4;
        unsigned int IDE : 1;
        unsigned int RTR : 1;
        unsigned int BRS : 1;
        unsigned int FDF : 1;
        unsigned int ESI : 1;
        unsigned int : 2;
        unsigned int FILHIT : 5;
        uint16_t empty;
        uint32_t timeStamp;
        uint8_t data[64];
    };
    uint8_t bytes[76];
} RXBufferWithTimeStamp;

//commands
#define RESET       0b0000
#define READ        0b0011
#define WRITE       0b0010
#define READ_CRC    0b1011
#define WRITE_CRC   0b1010
#define WRITE_SAFE  0b1100

//register addresses
#define OSC         0xe00
#define C1CON       0x000
#define C1NBTCFG    0x004
#define C1DBTCFG    0x008
#define C1TDC       0x00c
#define C1FIFOCON1  0x05c
#define C1FIFOSTA1  0x060
#define C1FIFOOUA1  0x064
#define C1FLTOBJ0   0x1f0
#define C1MASK0     0x1f4
#define C1FLTCON0   0x1d0
#define C1TXQCON    0x050
#define C1TXQSTA    0x054
#define C1TXQUA     0x058
#define C1INT       0x01c
#define IOCON       0xe04
#define C1TSCON     0x014
#define C1TREC      0x034

//Ram base address
#define RAM_BASE   0x400

//Operating modes

typedef enum {
    NORMAL_MODE = 0, SLEEP_MODE = 1, INT_LOOPBACK_MODE = 2,
    LISTEN_ONLY_MODE = 3, CONFIG_MODE = 4, EXT_LOOPBACK_MODE = 5, CAN_2_0_MODE = 6,
    RESTRICTED_MODE = 7
} OperatingMode;

typedef enum {
    TX_FIFO = 1, RX_FIFO = 0
} FifoType;

typedef enum {
    P_8 = 0, P_12 = 1, P_16 = 2, P_20 = 3, P_24 = 4, P_32 = 5,
    P_48 = 6, P_64 = 7
} PayloadSize;

typedef enum {
    SID_11, SID_12, EID
} IDType;

//FIFO interrupt flag values.  OR together to set
#define INT_RX_FULL         0x04
#define INT_RX_HALF_FULL    0x02
#define INT_RX_NOT_EMPTY    0x01
#define INT_TX_NOT_FULL     0x01
#define INT_TX_HALF_FULL    0x02
#define INT_TX_EMPTY        0x04
#define INT_RX_OVF          0x08
#define INT_TX_EXHAUSTED    0x10

//System interrupt flags.  OR together
#define INT_INVALID_MESSAGE     0x8000
#define INT_WAKE_UP             0x4000
#define INT_CAN_BUS_ERROR       0x2000
#define INT_SYSTEM_ERROR        0x1000
#define INT_RX_OVF_ERROR        0x0800
#define INT_TX_ATTEMPT          0x0400
#define INT_SPI_CRC_ERROR       0x0200
#define INT_ECC_ERROR           0x0100
#define INT_TEF                 0x0010
#define INT_MODE_CHANGE         0x0008
#define INT_TIME_BASE           0x0004
#define INT_RX                  0x0002
#define INT_TX                  0x0001
#define INT_NONE                0
#define INT_ALL                 0xff1f

//Baud rates

typedef enum {
    N_50K = 0, N_100K, N_125K, N_250K, N_500K, N_800K, N_1M
} NominalBaudRates;

typedef enum {
    D_500K = 0, D_1M, D_2M, D_5M, D_8M
} DataBaudRates;

#ifdef	__cplusplus
extern "C" {
#endif
    uint32_t MCP2517FDRead32bitRegister(uint16_t address);
    uint8_t MCP2517FDRead8bitRegister(uint16_t address);
    void MCP2517FDWrite32bitRegister(uint16_t address, uint32_t value);
    void MCP2517FDWrite8bitRegister(uint16_t address, uint8_t value);
    void MCP2517FDReset(void);
    uint8_t MCP2517FDConfig(void);
    void MCP2517FDWaitForOsc(void);
    bool MCP2517FDChangeMode(OperatingMode mode);
    void MCP2517FDSetNominalBaud(uint8_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw);
    void MCP2517FDSetDataBaud(uint8_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw, uint8_t tdco);
    void MCP2517FDConfigFIFO(uint8_t fifoNumber, FifoType type, PayloadSize payloadSize, uint8_t fifoSize, uint8_t priority, bool rtr, bool timeStamp, uint8_t interrupts);
    bool MCP2517FDTransmitMessage(uint8_t fifoNumber, TXBuffer *txBuffer, bool txNow);
    void MCP2517FDWriteRam(uint16_t address, uint8_t *buffer, uint16_t bytes);
    void MCP2517FDReadRam(uint16_t address, uint8_t *buffer, uint16_t bytes);
    uint32_t FormatID(uint32_t id, IDType type);
    void MCP2517FDSetFilter(uint8_t filterNumber, uint32_t filterID, IDType filterIDType, uint32_t maskID, IDType maskIDType, bool MIDE, uint8_t fifoNumber);
    bool MCP2517FDReceiveMessage(uint8_t fifoNumber, RXBuffer *rxBuffer);
    int MCP2517FDReceiveMessageWithTimestamp(uint8_t fifoNumber, RXBufferWithTimeStamp *rxBuffer);
    void MCP2517FDEnableSystemInterrupts(uint16_t flags);
    void MCP2517FDDisableSystemInterrupts(uint16_t flags);
    void MCP2517FDClearSystemInterrupts(uint16_t flags);
    void MCP2517FDConfigurePins(void);
    uint8_t DLCToLength(uint8_t dlc);
    bool MCP2517FDSetBauds(NominalBaudRates nominalBaud, DataBaudRates dataBaud);
    OperatingMode MCP2517FDGetCurrentMode(void);
    void MCP2517FDClearFIFOErrors(uint8_t fifoNumber);
    void MCP2517FDClearFilter(uint8_t filterNumber);
    void MCP2517FDClearAllFilters(void);
    void MCP2517FDSleep(void);
    void MCP2517FDWakeFromSleep(void);
#ifdef	__cplusplus
}
#endif

#endif	/* MCP2517FD_H */

