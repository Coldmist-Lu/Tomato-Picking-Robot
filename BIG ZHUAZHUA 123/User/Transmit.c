#include "Transmit.h"
#include "./usart/bsp_usart.h"

extern struct Receive_Interrupt Receive_IT;

void Sending_Data( int16_t speed ) {
	uint8_t text[4] = {'S', 'L', 0, 0};
	text[2] = ((speed & 0xff00) >> 8);
	text[3] = (speed & 0x00ff);
	
	for(uint8_t i = 0; i < 4; ++i) {
		USART_SendData(DEBUG_USARTx, text[i]);
		while(USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TC)==RESET);
	}
}

uint8_t Receving_Data( void ) {
		if(Receive_IT.flag == ENABLE){
		if(Receive_IT.text[0] == 'C' && Receive_IT.text[1] == 'A') {
			Receive_IT.flag = DISABLE;
			return START_FLAG;
		}
		if(Receive_IT.text[0] == 'C' && Receive_IT.text[1] == 'S') {
			Receive_IT.flag = DISABLE;
			return STOP_FLAG;
		}
	}
	return UNKNOWN;
}


