#ifndef _SERIAL_DATA_HANDLE_H
#define _SERIAL_DATA_HANDLE_H


//接收命令缓存
typedef struct
{
		u8 recive_command;							//接收命令内容
		u8 command_input_pass;					//命令注入通道
}Recive_command_dev;


//发送命令缓存
typedef struct
{
		u8 Send_Command;								//记录上一次发送的命令
		int16_t Send_Speed;									//记录当前速度
}Send_command_dev;

void SendCommandHandle(u8 command,int16_t data);
void RecvCommandHandle(void);

#endif

