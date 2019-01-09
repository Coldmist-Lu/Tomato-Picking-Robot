#ifndef __TRANSMIT_H
#define __TRANSMIT_H

#include "stm32f10x.h"
// 接收数据返回标志
#define START_FLAG				1
#define STOP_FLAG					2
#define UNKNOWN						0
// 发送数据表示
#define NORMAL_SPEED			Sending_Data(1600)
#define SLOW_SPEED				Sending_Data(300)
#define STOP_CAR					Sending_Data(20)
#define REVERSE_CAR				Sending_Data(-300)
// 接收函数调用
//#define CONDITION_CHECK		(Receving_Data())

uint8_t Receving_Data( void );
void Sending_Data( int16_t speed );

#endif  /* __TRANSMIT_H */

