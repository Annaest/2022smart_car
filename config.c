#include "config.h"
#include "headfile.h"
#include "string.h"
#include "self_headfile.h"
#include "fsl_gpt.h"
#include "zf_systick.h"
#include "string.h"
//使用GPT1记录系统时间
#define clcok_gpt GPT1
//设置分频系数
#define clock_gpt_div 75


void config_all()
{
	 
	//----------------------------------------------------------------
    DisableGlobalIRQ();
    board_init();   //务必保留，本函数用于初始化MPU 时钟 调试串口
	systick_delay_ms(300);	//延时300ms，等待主板其他外设上电成功
	//---------------------------------------------------------------
	//时钟初始化
	clock_init();
	
	//imu初始化
	icm20602_init_spi();
	//imu963ra_init ();
	//systick_delay_ms(1000);
	//卡尔曼滤波初始化
	//陀螺仪x,y,z
	//kalmanFilter_init(kf+3,-25 ,0.1 ,0.1 ,0.2667);   
	//kalmanFilter_init(kf+4,-4 ,0.1 ,0.1 ,0.1167);
	//IMU参数初始化
	imu_init();
	kalmanFilter_init(kf+5, 0 ,0.5 ,100 ,0.0625);
	
	
	//电机初始化
	config_motor();
	//IO口初始化
	config_gpio();
	//屏幕初始化
    //ips200_init(); 
	lcd_init();    
	//舵机初始化
	pwm_init(PWM4_MODULE2_CHA_C30 ,333,119*200);
	//编码器初始化
	//初始化摄像头 使用CSI接口
	//mt9v03x_csi_init();	
    //如果屏幕一直显示初始化信息，请检查摄像头接线
    //如果使用主板，一直卡在while(!uart_receive_flag)，请检查是否电池连接OK?
    //如果图像只采集一次，请检查场信号(VSY)是否连接OK?
	config_enc();
	//PIT定时器中断初始化
	pit_init();
	pit_interrupt_ms(PIT_CH0, pit0_time);
	pit_interrupt_ms(PIT_CH1, pit1_time);
	//pit_interrupt_ms(PIT_CH2, pit2_time);
	pit_interrupt_ms(PIT_CH3, pit3_time);
    NVIC_SetPriority(PIT_IRQn,0);
  
	
	
	//串口初始化
	config_uart();
	EnableGlobalIRQ(0);
	//DisableGlobalIRQ();
	
}
//IO口初始化
void config_gpio()
{
	//按键初始化
	config_key();
	//初始化蜂鸣器引脚
    gpio_init(BEEP_PIN,GPO,0,GPIO_PIN_CONFIG);
	gpio_init(B14, GPO, 1, GPIO_PIN_CONFIG);
	gpio_init(B15, GPO, 1, GPIO_PIN_CONFIG);
	gpio_init(B21, GPO, 1, GPIO_PIN_CONFIG);
	gpio_init(B23, GPO, 0, GPIO_PIN_CONFIG);
}
//电机初始化
void config_motor()
{
	gpio_init(DIR_1, GPO, 0, GPIO_PIN_CONFIG); 		//单片机端口D0 初始化DIR_1			GPIO
	gpio_init(DIR_2, GPO, 0, GPIO_PIN_CONFIG); 		//单片机端口D1 初始化DIR_2			GPIO
	pwm_init(PWM_1, 17000, 0);      				//单片机端口D2 初始化PWM_1周期10K 占空比0
	pwm_init(PWM_2, 17000, 0);     					//单片机端口D3 初始化PWM_2周期10K 占空比0
    
    gpio_init(DIR_3, GPO, 0, GPIO_PIN_CONFIG);      //单片机端口D0 初始化DIR_1          GPIO
    gpio_init(DIR_4, GPO, 0, GPIO_PIN_CONFIG);      //单片机端口D1 初始化DIR_2          GPIO
    pwm_init(PWM_3, 17000, 0);                      //单片机端口D2 初始化PWM_1周期10K 占空比0
    pwm_init(PWM_4, 17000, 0);                      //单片机端口D3 初始化PWM_2周期10K 占空比0
}
//编码器初始化
void config_enc()
{
	//初始化 QTIMER_1 A相使用QTIMER1_TIMER0_C0 B相使用QTIMER1_TIMER1_C1
    qtimer_quad_init(QTIMER_1,QTIMER1_TIMER0_C0,QTIMER1_TIMER1_C1);
    //初始化 QTIMER_1 A相使用QTIMER1_TIMER2_C2 B相使用QTIMER1_TIMER3_C24
    qtimer_quad_init(QTIMER_1,QTIMER1_TIMER2_C2,QTIMER1_TIMER3_C24);
    qtimer_quad_init(QTIMER_2,QTIMER2_TIMER0_C3,QTIMER2_TIMER3_C25);
    qtimer_quad_init(QTIMER_3,QTIMER3_TIMER2_B18,QTIMER3_TIMER3_B19);
	qtimer_quad_clear(QTIMER_1, QTIMER1_TIMER0_C0);
	qtimer_quad_clear(QTIMER_1,QTIMER1_TIMER2_C2);
	qtimer_quad_clear(QTIMER_2,QTIMER2_TIMER0_C3);
	qtimer_quad_clear(QTIMER_3,QTIMER3_TIMER2_B18);

}
//串口初始化
void config_uart()
{
	
	//串口1
	uart_init(USART_1,1000000,UART1_TX_B12,UART1_RX_B13);
	uart_rx_irq(USART_1,1);
	//uart_tx_irq(USART_1,1);
	EnableIRQ(LPUART1_IRQn);
	NVIC_SetPriority(USART_1,0);
	//串口3
	uart_init(USART_3,9600,UART3_TX_C8,UART3_RX_C9);
	uart_rx_irq(USART_3,1);
	//uart_tx_irq(USART_3,1);
	EnableIRQ(LPUART3_IRQn);
	NVIC_SetPriority(USART_3,2);
	//串口4
	//OpenArt 和 rt1064通信的波特率不要设置太高，否则造成rt1064接收寄存器溢出，程序卡死
	//经多次实验，19200,9600较好
	uart_init(USART_4,9600,UART4_TX_C16,UART4_RX_C17);
	uart_rx_irq(USART_4,1);
	//uart_tx_irq(USART_4,1);
	EnableIRQ(LPUART4_IRQn);
	NVIC_SetPriority(USART_4,2);
	
	//无线转串口（串口8）
    seekfree_wireless_init();
	//uart_rx_irq(USART_8,1);
	//uart_tx_irq(USART_8,1);
	//EnableIRQ(LPUART8_IRQn);
	NVIC_SetPriority(USART_8,3);
	

}

void print(int16_t a)
{
	
}

void delayms(uint16_t ms)
{
	volatile uint32_t i = 0;
	while(ms--)
	{
		for (i = 0; i < 30000; ++i)
		{
			__asm("NOP"); /* delay */
		}
	}	
}



FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
    /* 这里 printf 使用 LPUART1 可以自行修改 */
	uart_putchar(USART_1, (uint8_t)ch);	
	return ch;
}


void vofa_print(float *data,uint8_t ch_num)
{
	
	uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    uart_putbuff(USART_8, (uint8_t*)data,sizeof(float)*ch_num);	
	for(int i=0;i<4;i++) uart_putchar(USART_8, tail[i]);	
}
void config_key()
{
	gpio_init(KEY1, GPI, 0, GPIO_PIN_CONFIG);
	gpio_init(KEY2, GPI, 0, GPIO_PIN_CONFIG);
	gpio_init(KEY3, GPI, 0, GPIO_PIN_CONFIG);
	gpio_init(KEY4, GPI, 1, GPIO_PIN_CONFIG);
}

void clock_init()
{
	gpt_config_t gptConfig;
    
	GPT_GetDefaultConfig(&gptConfig);           //获取默认配置
	GPT_Init(clcok_gpt, &gptConfig);            //GPT初始化 便于打开时钟
	GPT_Deinit(clcok_gpt);                      //GPT反初始化
	GPT_Init(clcok_gpt, &gptConfig);            //GPT初始化
	GPT_SetClockDivider(clcok_gpt, clock_gpt_div);    //设置分频系数
	GPT_StartTimer(clcok_gpt);
}
uint32_t get_time_ms()
{
	
	return (uint32_t)(GPT1->CNT/1000);
}

uint32_t get_time_us()
{
	
	return (uint32_t)GPT1->CNT;
}
/*
 *@description		:初始化GPT1定时器
 *@param			:无
 *@return			:无
 */
void gpt1_init(void)
{
	GPT1->CR = 0;	/*清零*/
	GPT1->CR |= (1 << 15);	/*软件复位*/
	while((GPT1->CR >> 15) & 0x01);	/*等待软件复位成功*/
	GPT1->CR |= (1 << 6);	/*
							*运行模式：restart模式
							*时钟源：ipg_clk=66MHz
							*定时器关闭时保留计数值
							*/
	GPT1->PR = (66-1);	/*设置分频器为66分频*/
	GPT1->OCR[0] = 0XFFFFFFFF;	/*设置输出比较通道一的寄存器值*/
	GPT1->CR |= (1 << 0);	/*使能GPT定时器*/
	GPT_StartTimer(clcok_gpt);
}



//-------------------------------------------------------------------------------------------------------------------
void send_msg(uint16 pos_x, uint16 pos_y, uint8_t type)
{
	uint8_t msg[18] = {0};
	uint16_t now_time =  get_time_ms();
	uint16_t second = now_time/1000;
	uint16_t msecond = now_time%1000;
    //时间信息
    msg[0] = second/100 + '0';
    msg[1] = second%100/10 + '0';
    msg[2] = second%10 + '0';
    msg[3] = '.';
    msg[4] = msecond/100 + '0';
    msg[5] = msecond%100/10 + '0';
    msg[6] = msecond%10 + '0';
	
    pos_x /= 20;
	pos_y /= 20;
     //坐标X信息
    msg[7] = ' ';
    msg[8] = pos_x/10 + '0';
    msg[9] = pos_x%10 + '0';
    
    //坐标Y信息
    msg[10] = ' ';
    msg[11] = pos_y/10 + '0';
    msg[12] = pos_y%10 + '0';
    
    //类别 大类信息
    msg[13] = ' ';
    msg[14] = (uint8)type/10 + '0';
    
    //类别 小类信息
    msg[15] = ' ';
    msg[16] = (uint8)type%10 + '0';
    
    msg[17] = '\n';
    uart_putbuff(USART_8, msg, 18);
}
//屏幕显示
void lcdshow_sys(uint8_t obj_num)
{
	uint16_t up = 0,down = 8,x_center = 50,left = 0,right = left + 80; //布局
	//每列数目
	uint8_t potnum = 8;
	char chinese[3][5][5] = {{"dog","hor","cat","pig","cow"},
							{"org","app","dur","gra","ban"},
							{"tra","shi","pla","car","bus"}};
	
	uint8_t posx = (pos_XY[obj_num][0]+10)/20;
	uint8_t posy = (pos_XY[obj_num][1]+10)/20;
	char type[5] = ""; 
	strcpy(type,chinese[object_type[obj_num]/10-1][object_type[obj_num]%10-1]);
	if(obj_num < potnum)
	{
		lcd_showint32(left,up+obj_num,posx,2);
		lcd_showint32(left+25,up+obj_num,posy,2);
		lcd_showstr(left+50,up+obj_num,type);
	
	}
	else
	{
		lcd_showint32(right,up+obj_num-potnum,posx,2);
		lcd_showint32(right+25,up+obj_num-potnum,posy,2);
		lcd_showstr(right+50,up+obj_num-potnum,type);
		
	}

	
}