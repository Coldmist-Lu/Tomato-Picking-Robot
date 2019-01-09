#ifndef _USART_H_
#define _USART_H_
#include "stm32f10x.h"
/** 
 * @defgroup SERIALPORT
 * @brief SERIALPORT driver modules
 * @{
 */
 
#define SERIALPORT_MASTER
//#define SERIALPORT_SLAVE
/**
 * @}
 */
#if (!defined SERIALPORT_MASTER && !defined SERIALPORT_SLAVE) || (defined SERIALPORT_MASTER && defined SERIALPORT_SLAVE)
    #error Which identity are you using?
#endif
 
/**
 * @brief Commands enumeration
 */
typedef enum
{
    #ifdef SERIALPORT_SLAVE
    SERIALPORT_CommandSlowDown,//slow down the car
    SERIALPORT_CommandRestoreSpeed,//restore the speed before slow down
    #endif
    #ifdef SERIALPORT_MASTER
    SERIALPORT_CommandCameraStart,//turn on the camera(s)
    SERIALPORT_CommandCameraStop,//turn off the camera(s)
    #endif
    SERIALPORT_CommandUnknown,//unknown command
    SERIALPORT_CommandNull//no command receieved
}SERIALPORT_CommandTypedef;

void SERIALPORT_Init(void);
#ifdef SERIALPORT_MASTER
void SERIALPORT_SlowDown(int16_t speed);
void SERIALPORT_RestoreSpeed(void);
#endif
#ifdef SERIALPORT_SLAVE
void SERIALPORT_CameraStart(void);
void SERIALPORT_CameraStop(void);
#endif
void SERIALPORT_RestartDma(void);
SERIALPORT_CommandTypedef SERIALPORT_ReceiveCommand(int16_t *parameter);
/**
 * @}
 */
#endif
