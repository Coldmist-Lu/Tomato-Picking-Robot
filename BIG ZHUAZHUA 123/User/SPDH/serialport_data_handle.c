#include "serialport_data_handle.h"

extern _usart_recv_dev usartrecvdev;								//���ڽ������ݻ���

extern Recive_command_dev ReciveCommandDev;				//���������
extern Send_command_dev	 SendCommandDev;					//���������


/*�������ݴ���*/
//����:command: ����		data:   ����
//����:���������ˢ�·��������͵�ǰ�ٶ�
void SendCommandHandle(u8 command,int16_t data)
{
			uint8_t i;
			uint8_t text[4];
			switch(command)
			{
				case SlowDown: *text = 'S';*(text+1) = 'L';*(text+2)=(u8)(data>>8);*(text+3)=(u8)(data);break;
				case RestoreSpeed: *text = 'R';*(text+1) = 'S';*(text+2)=(u8)(data>>8);*(text+3)=(u8)(data);break;
			}
			for(i=0;i<4;i++)
			{
					USART_SendData(USART1,text[i]);
					while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
			}
			SendCommandDev.Send_Command = command;
			SendCommandDev.Send_Speed = data;
}


/*�������ݴ���*/
//���ã�ˢ�½��������
void RecvCommandHandle()
{
		if(ReciveCommandDev.command_input_pass==USART_IN)
		{
				if(usartrecvdev.recv_end_flag==ENABLE)
				{
						if(*(usartrecvdev.recv_text)=='C'&&*(usartrecvdev.recv_text+1)=='A')
						{
								ReciveCommandDev.recive_command = ScanStart;
								SendCommandDev.Send_Speed = 1600;
						}
						if(*usartrecvdev.recv_text=='C'&&*(usartrecvdev.recv_text+1)=='S')
						{
								ReciveCommandDev.recive_command = ScanStop;
						}
						usartrecvdev.recv_end_flag = DISABLE;
				}
		}
		
}

