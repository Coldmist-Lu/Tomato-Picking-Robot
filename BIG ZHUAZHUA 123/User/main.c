/* ���Ѳ�ժ����������ͷʶ�𡢻�е�ֲ�ժ��������ģ�� */
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


/* OV7725��ʱ���� */
extern uint8_t Ov7725_vsync;	/* ����ͷ������ʱ���� */
unsigned int Task_Delay[NumOfTask]; 
extern OV7725_MODE_PARAM cam_mode;

/* ADC��ʱ������ADC1ת���ĵ�ѹֵͨ��MDA��ʽ����SRAM */
/* ��5 ��3 */
extern __IO uint16_t ADC_ConvertedValue[NOFCHANEL];

/* ͨѶ���� */
struct Receive_Interrupt Receive_IT;

int main(void) 
{	
	uint16_t Tree_Detect_cnt = 0;
	Receive_IT.cnt = 0;
	Receive_IT.flag = DISABLE;
	/* ����ͷʶ��������� */
	uint16_t H = 0; 											//ɫ�ำֵ
	uint16_t Dist, Left_Dist, Right_Dist;	//�����þ��롣���踳ֵ����ADC�õ�
	//const uint16_t allOne = 0xffff;				//���ֵ��
	//for(uint8_t i = 0; i < 6; ++i) Signals[i] = 0;	//�����ź�����
//	uint8_t tmp;													
	uint8_t LeftLight_Enable, RightLight_Enable;
	/* ��ʾ����ʼ�� */
	LCD_SCREEN_CONFIG();	
	/* ���ڡ�LED��������ϵͳ��ʱ����ʼ�� */
	//USART_Config();
	uint8_t RIT;
	uart1_init(115200);
	LED_GPIO_Config();
	Key_GPIO_Config();
	SysTick_Init();
	/* ����ͷ��ʼ�� */
	OV7725_Initializer();
	/* ��ʱ����ʼ�� */
	GENERAL_TIM_Init();

	/* צ�Ӷ����ʼ�� */
	Hand_Init();
	/* ���������ʼ�� */
	ADCx_Init();
	Dist = Distance_Initialize(40); /* ���������PC1��PC4�ڣ��ֱ��Ӧ�����Ҳ� */		
	
	LeftLight_Enable = 0;		// ����Ĭ������
	RightLight_Enable = 0;
	uint8_t road_counter = 0;
	while(1) { 
		/* ���Ⲷ�� */
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
		
		if(Left_Dist < Dist && Right_Dist < Dist) { //δʶ���� 
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

