/* 
 * File:   Status.h
 * Author: Brad
 *
 * Created on January 9, 2018, 6:42 PM
 */

#ifndef STATUS_H
#define	STATUS_H

#include <stdint.h>

//MCP2517FD Error flags
#define ERROR_IVM           0x80    //Invalid message
#define ERROR_CER           0x20    //CAN bus error
#define ERROR_SER           0x10    //System error
#define ERROR_RXOV          0x08    //Rx overflow error

//System errors
#define ERROR_USB_RX        0x40    //USB Rx buffer overrun
#define ERROR_USB_TX        0x04    //USB Tx buffer overrun
#define ERROR_MODE_CHANGE   0x01    //Timout during mode change

#ifdef	__cplusplus
extern "C" {
#endif
    
    typedef struct {
        uint8_t errorFlags;
        uint8_t busState;
        uint8_t rxErrors;
        uint8_t txErrors;
        uint8_t currentMode;
        uint8_t systemErrors;
        uint8_t reserved2;
        uint8_t reserved3;
    } StatusInfo;
      
    typedef struct {
        uint8_t nominalBaud;
        uint8_t dataBaud;
        uint8_t txQLength;
        uint8_t txFIFOLength;
        uint8_t rxFIFOLength;
        uint8_t reserved1;
        uint8_t reserved2;
        uint8_t reserved3;
    } HardwareInfo;
        
    extern StatusInfo status;
    extern HardwareInfo hardware;

    void HandleSystemInterrupt(void);
    void CheckBusState(void);
    
#ifdef	__cplusplus
}
#endif

#endif	/* STATUS_H */

