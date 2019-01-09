/* 番茄采摘机器人摄像头识别、机械手采摘、激光测距模块 */
/* 2018.12.5 */
#include "stm32f10x.h"
#include "./ov7725/bsp_ov7725.h"
#include "./lcd/bsp_ili9341_lcd.h"
#include "./led/bsp_led.h"   
#include "./usart/bsp_usart.h"
#include "./key/bsp_key.h"  
#include "./systick/bsp_SysTick.h"
#include "./GeneralTim/bsp_GeneralTim.h"
#include "./adc/bsp_adc.h"
#include "./task_functions.h"
#include "./Transmit.h"


/* OV7725临时变量 */
extern uint8_t Ov7725_vsync;	/* 摄像头捕获临时变量 */
unsigned int Task_Delay[NumOfTask]; 
extern OV7725_MODE_PARAM cam_mode;

/* ADC临时变量：ADC1转换的电压值通过MDA方式传到SRAM */
/* 左5 右3 */
extern __IO uint16_t ADC_ConvertedValue[NOFCHANEL];

/* 通讯变量 */
struct Receive_Interrupt Receive_IT;

int main(void) 
{	
	uint16_t Tree_Detect_cnt = 0;
	Receive_IT.cnt = 0;
	Receive_IT.flag = DISABLE;
	/* 摄像头识别所需变量 */
	uint16_t H = 0; 											//色相赋值
	uint16_t Dist, Left_Dist, Right_Dist;	//激光测得距离。不需赋值，由ADC得到
	//const uint16_t allOne = 0xffff;				//最大值。
	//for(uint8_t i = 0; i < 6; ++i) Signals[i] = 0;	//接收信号清零
//	uint8_t tmp;													
	uint8_t LeftLight_Enable, RightLight_Enable;
	/* 显示屏初始化 */
	LCD_SCREEN_CONFIG();	
	/* 串口、LED、按键、系统定时器初始化 */
	//USART_Config();
	uint8_t RIT;
	uart1_init(115200);
	LED_GPIO_Config();
	Key_GPIO_Config();
	SysTick_Init();
	/* 摄像头初始化 */
	OV7725_Initializer();
	/* 定时器初始化 */
	GENERAL_TIM_Init();

	/* 爪子舵机初始化 */
	Hand_Init();
	/* 激光测距机初始化 */
	ADCx_Init();
	Dist = Distance_Initialize(40); /* 激光测距采用PC1、PC4口，分别对应左侧和右侧 */		
	
	LeftLight_Enable = 0;		// 测试默认启动
	RightLight_Enable = 0;
	uint8_t road_counter = 0;
	while(1) { 
		/* 激光捕获 */
		if(LeftLight_Enable == 0) {
			Left_Dist = 0;
		} else {
			Left_Dist = ADC_ConvertedValue[1];
		}
		if(RightLight_Enable == 0) {
			Right_Dist = 0;
		} else {
			Right_Dist = ADC_ConvertedValue[0];
		}
		
		RIT = Receving_Data();		
		if(RIT != UNKNOWN) {
			if(RIT == START_FLAG) {
				road_counter++;
				LeftLight_Enable = 1;
				if ( road_counter == 1 || road_counter == 6) RightLight_Enable = 0; else RightLight_Enable = 1;
			} else {
				LeftLight_Enable = 0;
				RightLight_Enable = 0;
			}
		}
		
		if(Left_Dist < Dist && Right_Dist < Dist) { //未识别到树 
			Tree_Detect_cnt = 0;
			continue;
		}
		
		if(Tree_Detect_cnt++ < 10) {
			continue;
		}
		
		Delay(0xfffff);
		SLOW_SPEED;

		if(Left_Dist > Dist) {
			Left_Task(H);
			Delay(0x18ffff);
			LeftLight_Enable = 0;
		} else {
			Right_Task(H);
			Delay(0x1fffff);
			RightLight_Enable = 0;
		}
//		if(LeftLight_Enable + RightLight_Enable == 1) {
//			REVERSE_CAR;
//			Delay(0xafffff);
//		}
		NORMAL_SPEED;
		Tree_Detect_cnt = 0;
	}
}




/*********************************************END OF FILE**********************/

