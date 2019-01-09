#ifndef __TASK_FUNCTIONS_H
#define __TASK_FUNCTIONS_H

#include "stm32f10x.h"
#include "./ov7725/bsp_ov7725.h"
#include "./lcd/bsp_ili9341_lcd.h"
#include "./led/bsp_led.h"   
#include "./usart/bsp_usart.h"
#include "./key/bsp_key.h"  
#include "./systick/bsp_SysTick.h"
#include "./GeneralTim/bsp_GeneralTim.h"
#include "./adc/bsp_adc.h"
#include "./serialport/serialport.h"

#define RPCNT 400
#define DETECT_SUCC_CNT 3
#define DETECT_CNT 15

/* 此处声明所有main中需要的函数 */
void Delay(__IO uint32_t nCount);
void LCD_SCREEN_CONFIG(void);
void OV7725_MODE_CONFIG(void);
void OV7725_Initializer(void);
void Hand_Init(void);
void Get_PingPang(void);
void Left_Task(uint16_t H);
void Right_Task(uint16_t H);

#endif /* __TASK_FUNCTIONS_H */
