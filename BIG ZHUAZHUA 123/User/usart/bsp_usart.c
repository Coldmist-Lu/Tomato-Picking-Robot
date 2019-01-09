#include "bsp_usart.h"

void uart1_init(uint32_t Baud)
{
		USART_InitTypeDef  USART_InitStructrue;
		GPIO_InitTypeDef GPIO_InitStructrue;
		NVIC_InitTypeDef NVIC_InitStructrue;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1,ENABLE);				//ʹ���õ���GPIOAʱ�Ӻ�APB2����ʱ��
	
		GPIO_InitStructrue.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStructrue.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructrue.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA,&GPIO_InitStructrue);
		//����PA9������2 TXD��Ϊ�����������
	
		GPIO_InitStructrue.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructrue.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA,&GPIO_InitStructrue);
		//����PA10������2 RXD��Ϊ���븴�ø���
	
		USART_InitStructrue.USART_BaudRate = Baud;									//������
		USART_InitStructrue.USART_WordLength = USART_WordLength_8b;	//8λ
		USART_InitStructrue.USART_StopBits = USART_StopBits_1;			//1��ֹͣλ
		USART_InitStructrue.USART_Parity = USART_Parity_No;					//��У��
		USART_InitStructrue.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//������������
		USART_InitStructrue.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;										//ģʽΪ���ͺͽ���˫��
		//����USART_InitStructrue����д�Ĳ�����ʼ������1
		USART_Init(USART1,&USART_InitStructrue);
		//ʹ�ܴ���1
		USART_Cmd(USART1,ENABLE);
		//��������жϣ������һ���ֽڷ��Ͳ���ȥ������
		USART_GetFlagStatus(USART1,USART_FLAG_TC);
		
		NVIC_InitStructrue.NVIC_IRQChannel = USART1_IRQn;						//USART2�жϺ�
		NVIC_InitStructrue.NVIC_IRQChannelSubPriority = 0;					//�����ȼ�����Ϊ0
		NVIC_InitStructrue.NVIC_IRQChannelCmd = ENABLE;							//IRQͨ��ʹ��
		NVIC_Init(&NVIC_InitStructrue);															//����NVIC_InitStructrue��ָ���Ĳ�����ʼ������NVIC�Ĵ���
		USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);								//�����ж�ʹ��
		
}

 /**
  * @brief  ����Ƕ�������жϿ�����NVIC
  * @param  ��
  * @retval ��
  */
//static void NVIC_Configuration(void)
//{
//  NVIC_InitTypeDef NVIC_InitStructure;
//  
//  /* Ƕ�������жϿ�������ѡ�� */
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//  
//  /* ����USARTΪ�ж�Դ */
//  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
//  /* �������ȼ�*/
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  /* �����ȼ� */
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  /* ʹ���ж� */
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  /* ��ʼ������NVIC */
//  NVIC_Init(&NVIC_InitStructure);
//}

 /**
  * @brief  USART GPIO ����,������������
  * @param  ��
  * @retval ��
  */
//void USART_Config(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;

//	// �򿪴���GPIO��ʱ��
//	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);
//	
//	// �򿪴��������ʱ��
//	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

//	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
//	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

//  // ��USART Rx��GPIO����Ϊ��������ģʽ
//	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
//	
//	// ���ô��ڵĹ�������
//	// ���ò�����
//	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
//	// ���� �������ֳ�
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//	// ����ֹͣλ
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;
//	// ����У��λ
//	USART_InitStructure.USART_Parity = USART_Parity_No ;
//	// ����Ӳ��������
//	USART_InitStructure.USART_HardwareFlowControl = 
//	USART_HardwareFlowControl_None;
//	// ���ù���ģʽ���շ�һ��
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//	// ��ɴ��ڵĳ�ʼ������
//	USART_Init(DEBUG_USARTx, &USART_InitStructure);
//	
//	// �����ж����ȼ�����
//	NVIC_Configuration();
//	
//	// ʹ�ܴ��ڽ����ж�
//	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);	
//	
//	// ʹ�ܴ���
//	USART_Cmd(DEBUG_USARTx, ENABLE);	    
//}

/*****************  ����һ���ֽ� **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* ����һ���ֽ����ݵ�USART */
	USART_SendData(pUSARTx,ch);
		
	/* �ȴ��������ݼĴ���Ϊ�� */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/****************** ����8λ������ ************************/

/*****************  �����ַ��� **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
  do 
  {
      Usart_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* �ȴ�������� */
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
  {}
}


///�ض���c�⺯��printf�����ڣ��ض�����ʹ��printf����
//int fputc(int ch, FILE *f)
//{
//		/* ����һ���ֽ����ݵ����� */
//		USART_SendData(DEBUG_USARTx, (uint8_t) ch);
//		
//		/* �ȴ�������� */
//		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET);		
//	
//		return (ch);
//}

///�ض���c�⺯��scanf�����ڣ���д����ʹ��scanf��getchar�Ⱥ���
//int fgetc(FILE *f)
//{
//		/* �ȴ������������� */
//		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE) == RESET);

//		return (int)USART_ReceiveData(DEBUG_USARTx);
//}
extern struct Receive_Interrupt Receive_IT;

void USART1_IRQHandler(void)
{
		uint8_t data;
		if(USART_GetITStatus(DEBUG_USARTx, USART_IT_RXNE)!=RESET)					//�Ƿ�����ж�
		{
				data = USART_ReceiveData(DEBUG_USARTx);
				switch(data) {
					case '<':	//��ʼ����
						Receive_IT.cnt = 0;
						Receive_IT.flag = DISABLE;
						break;
					case '>':	//��������
						Receive_IT.flag = ENABLE;
						break;
					default:	//��;
						Receive_IT.text[Receive_IT.cnt++] = data;
						Receive_IT.flag = DISABLE;
				}
		}
}

