#include "exit.h"
#include "headfile.h"
#include "motor_drive.h"
#include "fsl_lpuart.h"

uint8_t ch = '0';
//中断服务函数
void LPUART8_IRQHandler(void) 
	{ 
		
		if((LPUART8->STAT)&kLPUART_RxDataRegFullFlag) //接收中断 
			{ 
				uint8_t receive_data; 
				receive_data = LPUART8->DATA;
				uart_putstr(USART_8,"receive data");
				uart_putchar(USART_8,receive_data);
				int8_t vel = 20;
				static uint8_t flag = 0;
				
				switch(receive_data - '0')
				{
					case 0:
						move_control(0,0,0);
						break;
					case 1:
						move_control(vel,0,0);
						break;
					case 2:
						move_control(-vel,0,0);
						break;
					case 3:
						move_control(0,vel,0);
						break;
					case 4:
						move_control(0,-vel,0);
						break;
					case 5:
						move_control(0,0,vel);
						break;
					case 6:
						move_control(0,0,-vel);
						break;
				}
				if(flag == receive_data - '0')  move_control(0,0,0);
				flag = receive_data - '0';
				
			}
			
			__DSB(); //数据同步屏蔽指令 
	}

void PIT_IRQHandler()
{
	if((PIT_GetStatusFlags(PIT,kPIT_Chnl_0)&kPIT_TimerFlag)==kPIT_TimerFlag)
	{
		
	}
}