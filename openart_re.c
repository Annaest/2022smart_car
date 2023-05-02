#include "openart_re.h"
#include "stdint.h"
#include "self_headfile.h"

#include "stdio.h"

uint8_t object_type[max_pot] = {0};
uint8_t reply_state;
uint8_t find_flag = 0;
uint8_t two_con = 0; //二次识别次序控制
void openart_xy(uint8_t data)
{
	//通信协议
	uint8_t head[2] = {0xeb,0x90};
	uint8_t tail[2] = {0x0d,0x0a};
	//数据队列
	static uint8_t frame[100];
	static uint8_t last_data = 0;
	static uint8_t count = 0;
	static uint8_t receive_type = 0;
	static uint32_t start_time = 0;
	if(data == head[1] && last_data == head[0])
	{
		receive_type = 1;	
	}
	//receive_type == 1 接收坐标
	else if(receive_type == 1)
	{
		frame[count++] = data;
		
	}
	//接收到帧尾或超时结束
	if(receive_type == 1 && data == tail[1] && last_data == tail[0])
	{
		//检查
		if(count%2!=0)
		{
			//printf("receice_error_404\r\n");
			return;
		}

		//数据写入坐标数组
		for(uint8_t i=0,j=0;i<count-2;i=i+4,j++)
		{
			
			//x
			pos_XY[j][0] = frame[i]*100+frame[i+1];
			//y
			pos_XY[j][1] = frame[i+2]*100+frame[i+3];
		
		}
		
		//lcd_showuint16(1,1,get_time_ms() - start_time);
		//全部重置
		start_time = 0;
		receive_type = 0;
		last_data = 0;
		for(int i=0;i<count;i++)  frame[i] = 0;
		count = 0;

	}
    last_data = data;
} 

void openart_error(uint8_t data)
{
	//通信协议
	uint8_t head1[2] = {0xec,0x91};
	uint8_t tail[2] = {0x0d,0x0a};
	//数据队列
	static uint8_t error[6];
	static uint8_t last_data = 0;
	static uint8_t count = 0;
	static uint8_t receive_type = 0;
	static uint8_t num = 0;
	if(data == head1[1] && last_data == head1[0])
	{
		receive_type = 1;	
	}
	else if(receive_type == 1)
	{
		error[count++] = data;
		
	}
	//接收到帧尾或超时结束
	if(receive_type == 1 && data == tail[1] && last_data == tail[0])
	{
		//检查
		if(count%2!=0)
		{
			//printf("receice_error_404\r\n");
			return;
		}
		//数据写入数组
		//x
		img_error[0] = (float)(error[0]%10*100+error[1]);
		if(error[0] >= 10) img_error[0] = -img_error[0];
		//y
		img_error[1] = (float)(error[2]%10*100+error[3]);
		if(error[2] >= 10) img_error[1] = -img_error[1];
		
		//全部重置
		receive_type = 0;
		last_data = 0;
		for(int i=0;i<count;i++)  error[i] = 0;
		count = 0;
		//刷新接收error
		error_flag = 1;
		//看到标志
		find_flag = 1; 
	}
    last_data = data;
}

void state_send()
{
	//通信协议
	uint8_t head = 0xec;
	uart_putchar(USART_3,head);
	uart_putchar(USART_3,work_state);
	//uart_putchar(USART_3,tail);
	uart_putchar(USART_4,head);
	uart_putchar(USART_4,work_state);
	//uart_putchar(USART_4,tail);
}

void openart1_receive(uint8_t data){
	//通信协议
	uint8_t head[2] = {0xed,0x92};
	static uint8_t count = 0;
	static uint8_t last_data = 0;
	static uint8_t receive_type = 0;
	if(last_data == head[0] && data == head[1]){
		receive_type = 1;
		
	}
	else if(receive_type == 1){
		work_state = data;
		receive_type = 0;
		last_data = 0;
	}
	last_data = data;
}

void openart2_receive(uint8_t data){
	//通信协议
	uint8_t head[2] = {0xed,0x92};
	static uint8_t last_data = 0;
	static uint8_t receive_type = 0;
	if(last_data == head[0] && data == head[1]){
		receive_type = 1;
	
	}
	else if(receive_type == 1){
		vofa_data[4] = data;
		//回传的状态
		reply_state = data;
		receive_type = 0;
		last_data = 0;
	}
	last_data = data;
}

void openart_type(uint8_t data)
{
	//通信协议
	uint8_t head[2] = {0xef,0x93};
	static uint8_t last_data = 0;
	static uint8_t receive_type = 0;
	if(last_data == head[0] && data == head[1]){
		receive_type = 1;
	}
	else if(receive_type == 1){
		object_type[two_con] = data;
		receive_type = 0;
		last_data = 0;
	}
	last_data = data;
}

float xkpid[3] = {0.05f,0.06f,0.f};
float ykpid[3] = {0.05f,0.06f,0.f};
//接近图片，前摄像头
void approach_img()
{

	static float pid_value[2] = {0};

	//异常值舍去
	if(abs(img_error[0]) > 40 || abs(img_error[1]) > 40)  return;

	img_error[0] -= IMU.encc_x/54.76f * pit0_time/1000.0f;
	img_error[1] -= IMU.encc_y/54.76f * pit0_time/1000.0f;
	vel_x = nolinar_error(1.7,img_error[0]);
	vel_y = nolinar_error(1.8,img_error[1]);

}