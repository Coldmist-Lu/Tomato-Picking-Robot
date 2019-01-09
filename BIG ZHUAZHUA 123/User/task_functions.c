#include "task_functions.h"
#include "./Transmit.h"	

/* 延时函数：提供cnt的延时 */
void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
} 

/* 液晶初始化函数 */
void LCD_SCREEN_CONFIG(void) {
	ILI9341_Init();
	ILI9341_GramScan ( 3 );	
	LCD_SetFont(&Font8x16);
	LCD_SetColors(RED,BLACK);
  ILI9341_Clear(0,0,LCD_X_LENGTH,LCD_Y_LENGTH);	/* 清屏，显示全黑 */
}

/* 摄像头配置函数（可调） */
extern OV7725_MODE_PARAM cam_mode;

void OV7725_MODE_CONFIG(void) {
	/*根据摄像头参数组配置模式*/
	OV7725_Special_Effect(cam_mode.effect);
	/*光照模式*/
	OV7725_Light_Mode(cam_mode.light_mode);
	/*饱和度*/
	OV7725_Color_Saturation(cam_mode.saturation);
	/*光照度*/
	OV7725_Brightness(cam_mode.brightness);
	/*对比度*/
	OV7725_Contrast(cam_mode.contrast);
	/*特殊效果*/
	OV7725_Special_Effect(cam_mode.effect);
	
	/*设置图像采样及模式大小*/
	OV7725_Window_Set(cam_mode.cam_sx,
														cam_mode.cam_sy,
														cam_mode.cam_width,
														cam_mode.cam_height,
														cam_mode.QVGA_VGA);

	/* 设置液晶扫描模式 */
	ILI9341_GramScan( cam_mode.lcd_scan );
	
	ILI9341_DispStringLine_EN(LINE(2),"OV7725 initialize success!");
}

/* 摄像头初始化函数 */
extern uint8_t Ov7725_vsync;

void OV7725_Initializer(void) {	
	/* 摄像头和液晶屏校准变量 */	
	uint8_t retry = 0;
	/* ov7725 gpio 初始化 */
	OV7725_GPIO_Config();
	/* ov7725 寄存器默认配置初始化 */
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

/* 初始化机械手PWM */
/*
	CH3:  爪子    60闭		180张
	CH4:	底座    50右   120中   220左
*/
void Hand_Init(void) {
	GENERAL_TIM_CH1_CHANGE(180);
	GENERAL_TIM_CH2_CHANGE(180);
	GENERAL_TIM_CH3_CHANGE(180); //爪子张开
	GENERAL_TIM_CH4_CHANGE(120); //底座回中
}


/* 抓球动作 */
void Get_PingPang(void) {
	GENERAL_TIM_CH3_CHANGE(180);	//开爪
	GENERAL_TIM_CH1_CHANGE(180);	//开爪
	GENERAL_TIM_CH2_CHANGE(180);	//开爪
	
	Delay(0xaffff);
	
	GENERAL_TIM_CH3_CHANGE(60);		//合爪
	GENERAL_TIM_CH2_CHANGE(60);		//合爪
	GENERAL_TIM_CH1_CHANGE(60);		//合爪
	
	Delay(0x3fffff);

	GENERAL_TIM_CH4_CHANGE(120);		//回中
	
	Delay(0x3fffff);
	
	GENERAL_TIM_CH3_CHANGE(180);		//开爪
	GENERAL_TIM_CH2_CHANGE(180);		//开爪
	GENERAL_TIM_CH1_CHANGE(180);		//开爪
}

/* 向左识别函数 */
void Left_Task(uint16_t H) {
	uint8_t tmpres = 0;//存放图像传输临时结果
	uint8_t k = 0, cnt = 0;			//存放临时次数。
	uint8_t Left_turn = 155; //控制左转角度
	
	GENERAL_TIM_CH4_CHANGE(Left_turn);//向左转至一个角度
	Delay(0x1fffff);
	
	while(1) {//初始化
		if((k >= 3) || (cnt > 10)) break;
	/*接收到新图像进行显示*/
		if( Ov7725_vsync == 2 )
		{
			cnt++;
			FIFO_PREPARE;  			/*FIFO准备*/					
			tmpres = ImagDisp1(cam_mode.lcd_sx,
							 cam_mode.lcd_sy,
							 cam_mode.cam_width,
							 cam_mode.cam_height,
							 H, RPCNT);			/*采集并显示*/
			Ov7725_vsync = 0;
			if(cnt >= 3 && tmpres) k++;
		}
	}
	
	if(k < 3) {
		REVERSE_CAR;
		Delay(0x7fffff);
		GENERAL_TIM_CH4_CHANGE(120); //未识别，回中
		return;
	}
	while(Left_turn <= 211) {
		while(1) {//初始化
		/*接收到新图像进行显示*/
			if( Ov7725_vsync == 2 )
			{
				FIFO_PREPARE;  			/*FIFO准备*/					
				tmpres = ImagDisp2(cam_mode.lcd_sx,
								 cam_mode.lcd_sy,
								 cam_mode.cam_width,
								 cam_mode.cam_height,
								 H);			/*采集并显示*/
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
		while(1) {//初始化
		/*接收到新图像进行显示*/
			if( Ov7725_vsync == 2 )
			{
				FIFO_PREPARE;  			/*FIFO准备*/					
				tmpres = ImagDisp2(cam_mode.lcd_sx,
								 cam_mode.lcd_sy,
								 cam_mode.cam_width,
								 cam_mode.cam_height,
								 H);			/*采集并显示*/
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

/* 向右识别函数 */
void Right_Task(uint16_t H) {
	uint8_t tmpres = 0;//存放图像传输临时结果
	uint8_t k = 0, cnt = 0;			//存放临时次数。
	uint8_t Left_turn = 85; //控制左转角度
	
	GENERAL_TIM_CH4_CHANGE(Left_turn);//向左转至一个角度
	Delay(0x1fffff);
	
	while(1) {//初始化
		if((k >= 3) || (cnt > 10)) break;
	/*接收到新图像进行显示*/
		if( Ov7725_vsync == 2 )
		{
			cnt++;
			FIFO_PREPARE;  			/*FIFO准备*/					
			tmpres = ImagDisp1(cam_mode.lcd_sx,
							 cam_mode.lcd_sy,
							 cam_mode.cam_width,
							 cam_mode.cam_height,
							 H, RPCNT);			/*采集并显示*/
			Ov7725_vsync = 0;
			if(cnt >= 3 && tmpres) k++;
		}
	}
	if(k < 3) {
		REVERSE_CAR;
		Delay(0x7fffff);
		GENERAL_TIM_CH4_CHANGE(120); //未识别，回中
		return;
	}
	while(Left_turn >= 50) {
		while(1) {//初始化
		/*接收到新图像进行显示*/
			if( Ov7725_vsync == 2 )
			{
				FIFO_PREPARE;  			/*FIFO准备*/					
				tmpres = ImagDisp2(cam_mode.lcd_sx,
								 cam_mode.lcd_sy,
								 cam_mode.cam_width,
								 cam_mode.cam_height,
								 H);			/*采集并显示*/
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
		while(1) {//初始化
		/*接收到新图像进行显示*/
			if( Ov7725_vsync == 2 )
			{
				FIFO_PREPARE;  			/*FIFO准备*/					
				tmpres = ImagDisp2(cam_mode.lcd_sx,
								 cam_mode.lcd_sy,
								 cam_mode.cam_width,
								 cam_mode.cam_height,
								 H);			/*采集并显示*/
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
