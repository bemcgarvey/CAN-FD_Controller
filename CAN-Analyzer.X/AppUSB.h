/* 
 * File:   AppUSB.h
 * Author: Brad
 *
 * Created on January 3, 2018, 5:23 PM
 */

#ifndef APPUSB_H
#define	APPUSB_H

#include <stdint.h>
#include <stdbool.h>

#define RX_EP       3
#define TXQ_EP      1
#define TXF_EP      2

#define SET_HARDWARE_CONFIG         0xA1
#define SET_FILTER                  0xA2
#define GET_STATUS                  0xA3
#define CLEAR_ERRORS                0xA4
#define GET_HARDWARE_INFO           0xA5
#define SET_FIFO_LENGTHS            0xA6
#define CLEAR_FILTER                0xA7
#define WAKE_FROM_SLEEP             0xA8
#define GET_VERSION                 0xA9

#ifdef	__cplusplus
extern "C" {
#endif

    void AppUSBInitialize(void);
    void AppUSBTasks(void);
    void GotHardwareConfig(void);
    void GotFilter(void);
    void GotFIFOLengths(void);
    
    extern uint8_t ctrlBufferOut[8];
    extern uint16_t filterNumber;
    extern bool clearFilter;

#ifdef	__cplusplus
}
#endif

#endif	/* APPUSB_H */

