#ifndef _SERIAL_DATA_HANDLE_H
#define _SERIAL_DATA_HANDLE_H


//���������
typedef struct
{
		u8 recive_command;							//������������
		u8 command_input_pass;					//����ע��ͨ��
}Recive_command_dev;


//���������
typedef struct
{
		u8 Send_Command;								//��¼��һ�η��͵�����
		int16_t Send_Speed;									//��¼��ǰ�ٶ�
}Send_command_dev;

void SendCommandHandle(u8 command,int16_t data);
void RecvCommandHandle(void);

#endif

