#include "bsp_usart.h"

void uart1_init(uint32_t Baud)
{
		USART_InitTypeDef  USART_InitStructrue;
		GPIO_InitTypeDef GPIO_InitStructrue;
		NVIC_InitTypeDef NVIC_InitStructrue;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1,ENABLE);				//使能用到的GPIOA时钟和APB2复用时钟
	
		GPIO_InitStructrue.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStructrue.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructrue.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA,&GPIO_InitStructrue);
		//配置PA9（串口2 TXD）为输出复用推挽
	
		GPIO_InitStructrue.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructrue.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA,&GPIO_InitStructrue);
		//配置PA10（串口2 RXD）为输入复用浮空
	
		USART_InitStructrue.USART_BaudRate = Baud;									//波特率
		USART_InitStructrue.USART_WordLength = USART_WordLength_8b;	//8位
		USART_InitStructrue.USART_StopBits = USART_StopBits_1;			//1个停止位
		USART_InitStructrue.USART_Parity = USART_Parity_No;					//不校验
		USART_InitStructrue.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//不进行流控制
		USART_InitStructrue.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;										//模式为发送和接收双向
		//根据USART_InitStructrue已填写的参数初始化串口1
		USART_Init(USART1,&USART_InitStructrue);
		//使能串口1
		USART_Cmd(USART1,ENABLE);
		//清除接收中断，解决第一个字节发送不出去的问题
		USART_GetFlagStatus(USART1,USART_FLAG_TC);
		
		NVIC_InitStructrue.NVIC_IRQChannel = USART1_IRQn;						//USART2中断号
		NVIC_InitStructrue.NVIC_IRQChannelSubPriority = 0;					//子优先级设置为0
		NVIC_InitStructrue.NVIC_IRQChannelCmd = ENABLE;							//IRQ通道使能
		NVIC_Init(&NVIC_InitStructrue);															//根据NVIC_InitStructrue中指定的参数初始化外设NVIC寄存器
		USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);								//接收中断使能
		
}

 /**
  * @brief  配置嵌套向量中断控制器NVIC
  * @param  无
  * @retval 无
  */
//static void NVIC_Configuration(void)
//{
//  NVIC_InitTypeDef NVIC_InitStructure;
//  
//  /* 嵌套向量中断控制器组选择 */
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//  
//  /* 配置USART为中断源 */
//  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
//  /* 抢断优先级*/
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  /* 子优先级 */
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  /* 使能中断 */
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  /* 初始化配置NVIC */
//  NVIC_Init(&NVIC_InitStructure);
//}

 /**
  * @brief  USART GPIO 配置,工作参数配置
  * @param  无
  * @retval 无
  */
//void USART_Config(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;

//	// 打开串口GPIO的时钟
//	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);
//	
//	// 打开串口外设的时钟
//	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

//	// 将USART Tx的GPIO配置为推挽复用模式
//	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

//  // 将USART Rx的GPIO配置为浮空输入模式
//	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
//	
//	// 配置串口的工作参数
//	// 配置波特率
//	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
//	// 配置 针数据字长
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//	// 配置停止位
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;
//	// 配置校验位
//	USART_InitStructure.USART_Parity = USART_Parity_No ;
//	// 配置硬件流控制
//	USART_InitStructure.USART_HardwareFlowControl = 
//	USART_HardwareFlowControl_None;
//	// 配置工作模式，收发一起
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//	// 完成串口的初始化配置
//	USART_Init(DEBUG_USARTx, &USART_InitStructure);
//	
//	// 串口中断优先级配置
//	NVIC_Configuration();
//	
//	// 使能串口接收中断
//	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);	
//	
//	// 使能串口
//	USART_Cmd(DEBUG_USARTx, ENABLE);	    
//}

/*****************  发送一个字节 **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* 发送一个字节数据到USART */
	USART_SendData(pUSARTx,ch);
		
	/* 等待发送数据寄存器为空 */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/****************** 发送8位的数组 ************************/

/*****************  发送字符串 **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
  do 
  {
      Usart_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* 等待发送完成 */
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
  {}
}


///重定向c库函数printf到串口，重定向后可使用printf函数
//int fputc(int ch, FILE *f)
//{
//		/* 发送一个字节数据到串口 */
//		USART_SendData(DEBUG_USARTx, (uint8_t) ch);
//		
//		/* 等待发送完毕 */
//		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET);		
//	
//		return (ch);
//}

///重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
//int fgetc(FILE *f)
//{
//		/* 等待串口输入数据 */
//		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE) == RESET);

//		return (int)USART_ReceiveData(DEBUG_USARTx);
//}
extern struct Receive_Interrupt Receive_IT;

void USART1_IRQHandler(void)
{
		uint8_t data;
		if(USART_GetITStatus(DEBUG_USARTx, USART_IT_RXNE)!=RESET)					//是否接收中断
		{
				data = USART_ReceiveData(DEBUG_USARTx);
				switch(data) {
					case '<':	//开始传输
						Receive_IT.cnt = 0;
						Receive_IT.flag = DISABLE;
						break;
					case '>':	//结束传输
						Receive_IT.flag = ENABLE;
						break;
					default:	//中途
						Receive_IT.text[Receive_IT.cnt++] = data;
						Receive_IT.flag = DISABLE;
				}
		}
}

