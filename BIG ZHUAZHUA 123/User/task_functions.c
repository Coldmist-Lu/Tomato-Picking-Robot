#include "task_functions.h"
#include "./Transmit.h"	

/* ��ʱ�������ṩcnt����ʱ */
void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
} 

/* Һ����ʼ������ */
void LCD_SCREEN_CONFIG(void) {
	ILI9341_Init();
	ILI9341_GramScan ( 3 );	
	LCD_SetFont(&Font8x16);
	LCD_SetColors(RED,BLACK);
  ILI9341_Clear(0,0,LCD_X_LENGTH,LCD_Y_LENGTH);	/* ��������ʾȫ�� */
}

/* ����ͷ���ú������ɵ��� */
extern OV7725_MODE_PARAM cam_mode;

void OV7725_MODE_CONFIG(void) {
	/*��������ͷ����������ģʽ*/
	OV7725_Special_Effect(cam_mode.effect);
	/*����ģʽ*/
	OV7725_Light_Mode(cam_mode.light_mode);
	/*���Ͷ�*/
	OV7725_Color_Saturation(cam_mode.saturation);
	/*���ն�*/
	OV7725_Brightness(cam_mode.brightness);
	/*�Աȶ�*/
	OV7725_Contrast(cam_mode.contrast);
	/*����Ч��*/
	OV7725_Special_Effect(cam_mode.effect);
	
	/*����ͼ�������ģʽ��С*/
	OV7725_Window_Set(cam_mode.cam_sx,
														cam_mode.cam_sy,
														cam_mode.cam_width,
														cam_mode.cam_height,
														cam_mode.QVGA_VGA);

	/* ����Һ��ɨ��ģʽ */
	ILI9341_GramScan( cam_mode.lcd_scan );
	
	ILI9341_DispStringLine_EN(LINE(2),"OV7725 initialize success!");
}

/* ����ͷ��ʼ������ */
extern uint8_t Ov7725_vsync;

void OV7725_Initializer(void) {	
	/* ����ͷ��Һ����У׼���� */	
	uint8_t retry = 0;
	/* ov7725 gpio ��ʼ�� */
	OV7725_GPIO_Config();
	/* ov7725 �Ĵ���Ĭ�����ó�ʼ�� */
	while(OV7725_Init() != SUCCESS)
	{
		retry++;
		if(retry>5)
		{
			ILI9341_DispStringLine_EN(LINE(2),"No OV7725 module detected!");
			while(1);
		}
	}
	OV7725_MODE_CONFIG();
	
	Ov7725_vsync = 0;
}

/* ��ʼ����е��PWM */
/*
	CH3:  צ��    60��		180��
	CH4:	����    50��   120��   220��
*/
void Hand_Init(void) {
	GENERAL_TIM_CH1_CHANGE(180);
	GENERAL_TIM_CH2_CHANGE(180);
	GENERAL_TIM_CH3_CHANGE(180); //צ���ſ�
	GENERAL_TIM_CH4_CHANGE(120); //��������
}


/* ץ���� */
void Get_PingPang(void) {
	GENERAL_TIM_CH3_CHANGE(180);	//��צ
	GENERAL_TIM_CH1_CHANGE(180);	//��צ
	GENERAL_TIM_CH2_CHANGE(180);	//��צ
	
	Delay(0xaffff);
	
	GENERAL_TIM_CH3_CHANGE(60);		//��צ
	GENERAL_TIM_CH2_CHANGE(60);		//��צ
	GENERAL_TIM_CH1_CHANGE(60);		//��צ
	
	Delay(0x3fffff);

	GENERAL_TIM_CH4_CHANGE(120);		//����
	
	Delay(0x3fffff);
	
	GENERAL_TIM_CH3_CHANGE(180);		//��צ
	GENERAL_TIM_CH2_CHANGE(180);		//��צ
	GENERAL_TIM_CH1_CHANGE(180);		//��צ
}

/* ����ʶ���� */
void Left_Task(uint16_t H) {
	uint8_t tmpres = 0;//���ͼ������ʱ���
	uint8_t k = 0, cnt = 0;			//�����ʱ������
	uint8_t Left_turn = 155; //������ת�Ƕ�
	
	GENERAL_TIM_CH4_CHANGE(Left_turn);//����ת��һ���Ƕ�
	Delay(0x1fffff);
	
	while(1) {//��ʼ��
		if((k >= 3) || (cnt > 10)) break;
	/*���յ���ͼ�������ʾ*/
		if( Ov7725_vsync == 2 )
		{
			cnt++;
			FIFO_PREPARE;  			/*FIFO׼��*/					
			tmpres = ImagDisp1(cam_mode.lcd_sx,
							 cam_mode.lcd_sy,
							 cam_mode.cam_width,
							 cam_mode.cam_height,
							 H, RPCNT);			/*�ɼ�����ʾ*/
			Ov7725_vsync = 0;
			if(cnt >= 3 && tmpres) k++;
		}
	}
	
	if(k < 3) {
		REVERSE_CAR;
		Delay(0x7fffff);
		GENERAL_TIM_CH4_CHANGE(120); //δʶ�𣬻���
		return;
	}
	while(Left_turn <= 211) {
		while(1) {//��ʼ��
		/*���յ���ͼ�������ʾ*/
			if( Ov7725_vsync == 2 )
			{
				FIFO_PREPARE;  			/*FIFO׼��*/					
				tmpres = ImagDisp2(cam_mode.lcd_sx,
								 cam_mode.lcd_sy,
								 cam_mode.cam_width,
								 cam_mode.cam_height,
								 H);			/*�ɼ�����ʾ*/
				Ov7725_vsync = 0;
				break;
			}
		}
		if(tmpres == 3){
			Left_turn += 7;
			if(Left_turn == 211) {
				Left_turn -= 7;
				GENERAL_TIM_CH4_CHANGE(Left_turn);
				break;
			} else {
				GENERAL_TIM_CH4_CHANGE(Left_turn);
			}
			
		}
	}
	
	uint8_t flag = 0,check_counter = 0;
	flag = tmpres;
	if ( flag == 3 ) SLOW_BACKWARD; else if ( flag == 2 ) SLOW_FORWARD;
	while (flag == tmpres && flag != 1)
	{
		while(1) {//��ʼ��
		/*���յ���ͼ�������ʾ*/
			if( Ov7725_vsync == 2 )
			{
				FIFO_PREPARE;  			/*FIFO׼��*/					
				tmpres = ImagDisp2(cam_mode.lcd_sx,
								 cam_mode.lcd_sy,
								 cam_mode.cam_width,
								 cam_mode.cam_height,
								 H);			/*�ɼ�����ʾ*/
				Ov7725_vsync = 0;
				break;
			}
		}
		check_counter++;
		if (check_counter == 5) break;
	}
	
	STOP_CAR;

	Get_PingPang();
	REVERSE_CAR;
	Delay(0x7fffff);
	
	return;
}

/* ����ʶ���� */
void Right_Task(uint16_t H) {
	uint8_t tmpres = 0;//���ͼ������ʱ���
	uint8_t k = 0, cnt = 0;			//�����ʱ������
	uint8_t Left_turn = 85; //������ת�Ƕ�
	
	GENERAL_TIM_CH4_CHANGE(Left_turn);//����ת��һ���Ƕ�
	Delay(0x1fffff);
	
	while(1) {//��ʼ��
		if((k >= 3) || (cnt > 10)) break;
	/*���յ���ͼ�������ʾ*/
		if( Ov7725_vsync == 2 )
		{
			cnt++;
			FIFO_PREPARE;  			/*FIFO׼��*/					
			tmpres = ImagDisp1(cam_mode.lcd_sx,
							 cam_mode.lcd_sy,
							 cam_mode.cam_width,
							 cam_mode.cam_height,
							 H, RPCNT);			/*�ɼ�����ʾ*/
			Ov7725_vsync = 0;
			if(cnt >= 3 && tmpres) k++;
		}
	}
	if(k < 3) {
		REVERSE_CAR;
		Delay(0x7fffff);
		GENERAL_TIM_CH4_CHANGE(120); //δʶ�𣬻���
		return;
	}
	while(Left_turn >= 50) {
		while(1) {//��ʼ��
		/*���յ���ͼ�������ʾ*/
			if( Ov7725_vsync == 2 )
			{
				FIFO_PREPARE;  			/*FIFO׼��*/					
				tmpres = ImagDisp2(cam_mode.lcd_sx,
								 cam_mode.lcd_sy,
								 cam_mode.cam_width,
								 cam_mode.cam_height,
								 H);			/*�ɼ�����ʾ*/
				Ov7725_vsync = 0;
				break;
			}
		}
		if(tmpres == 2){
			Left_turn -= 7;
			if(Left_turn == 50) {
				Left_turn += 7;
				GENERAL_TIM_CH4_CHANGE(Left_turn);
				break;
			} else {
				GENERAL_TIM_CH4_CHANGE(Left_turn);
			}
			
		}
	}
	uint8_t flag = 0,check_counter = 0;
	flag = tmpres;
	if ( flag == 2 ) SLOW_BACKWARD; else if ( flag == 3 ) SLOW_FORWARD;
	while (flag == tmpres && flag != 1)
	{
		while(1) {//��ʼ��
		/*���յ���ͼ�������ʾ*/
			if( Ov7725_vsync == 2 )
			{
				FIFO_PREPARE;  			/*FIFO׼��*/					
				tmpres = ImagDisp2(cam_mode.lcd_sx,
								 cam_mode.lcd_sy,
								 cam_mode.cam_width,
								 cam_mode.cam_height,
								 H);			/*�ɼ�����ʾ*/
				Ov7725_vsync = 0;
				break;
			}
		}
		check_counter++;
		if (check_counter == 5) break;
	}
	
	STOP_CAR;
	Get_PingPang();
	
	REVERSE_CAR;
	Delay(0x7fffff);
	return;
}
