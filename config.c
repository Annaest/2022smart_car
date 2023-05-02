#include "config.h"
#include "headfile.h"
#include "string.h"
#include "self_headfile.h"
#include "fsl_gpt.h"
#include "zf_systick.h"
#include "string.h"
//ʹ��GPT1��¼ϵͳʱ��
#define clcok_gpt GPT1
//���÷�Ƶϵ��
#define clock_gpt_div 75


void config_all()
{
	 
	//----------------------------------------------------------------
    DisableGlobalIRQ();
    board_init();   //��ر��������������ڳ�ʼ��MPU ʱ�� ���Դ���
	systick_delay_ms(300);	//��ʱ300ms���ȴ��������������ϵ�ɹ�
	//---------------------------------------------------------------
	//ʱ�ӳ�ʼ��
	clock_init();
	
	//imu��ʼ��
	icm20602_init_spi();
	//imu963ra_init ();
	//systick_delay_ms(1000);
	//�������˲���ʼ��
	//������x,y,z
	//kalmanFilter_init(kf+3,-25 ,0.1 ,0.1 ,0.2667);   
	//kalmanFilter_init(kf+4,-4 ,0.1 ,0.1 ,0.1167);
	//IMU������ʼ��
	imu_init();
	kalmanFilter_init(kf+5, 0 ,0.5 ,100 ,0.0625);
	
	
	//�����ʼ��
	config_motor();
	//IO�ڳ�ʼ��
	config_gpio();
	//��Ļ��ʼ��
    //ips200_init(); 
	lcd_init();    
	//�����ʼ��
	pwm_init(PWM4_MODULE2_CHA_C30 ,333,119*200);
	//��������ʼ��
	//��ʼ������ͷ ʹ��CSI�ӿ�
	//mt9v03x_csi_init();	
    //�����Ļһֱ��ʾ��ʼ����Ϣ����������ͷ����
    //���ʹ�����壬һֱ����while(!uart_receive_flag)�������Ƿ�������OK?
    //���ͼ��ֻ�ɼ�һ�Σ����鳡�ź�(VSY)�Ƿ�����OK?
	config_enc();
	//PIT��ʱ���жϳ�ʼ��
	pit_init();
	pit_interrupt_ms(PIT_CH0, pit0_time);
	pit_interrupt_ms(PIT_CH1, pit1_time);
	//pit_interrupt_ms(PIT_CH2, pit2_time);
	pit_interrupt_ms(PIT_CH3, pit3_time);
    NVIC_SetPriority(PIT_IRQn,0);
  
	
	
	//���ڳ�ʼ��
	config_uart();
	EnableGlobalIRQ(0);
	//DisableGlobalIRQ();
	
}
//IO�ڳ�ʼ��
void config_gpio()
{
	//������ʼ��
	config_key();
	//��ʼ������������
    gpio_init(BEEP_PIN,GPO,0,GPIO_PIN_CONFIG);
	gpio_init(B14, GPO, 1, GPIO_PIN_CONFIG);
	gpio_init(B15, GPO, 1, GPIO_PIN_CONFIG);
	gpio_init(B21, GPO, 1, GPIO_PIN_CONFIG);
	gpio_init(B23, GPO, 0, GPIO_PIN_CONFIG);
}
//�����ʼ��
void config_motor()
{
	gpio_init(DIR_1, GPO, 0, GPIO_PIN_CONFIG); 		//��Ƭ���˿�D0 ��ʼ��DIR_1			GPIO
	gpio_init(DIR_2, GPO, 0, GPIO_PIN_CONFIG); 		//��Ƭ���˿�D1 ��ʼ��DIR_2			GPIO
	pwm_init(PWM_1, 17000, 0);      				//��Ƭ���˿�D2 ��ʼ��PWM_1����10K ռ�ձ�0
	pwm_init(PWM_2, 17000, 0);     					//��Ƭ���˿�D3 ��ʼ��PWM_2����10K ռ�ձ�0
    
    gpio_init(DIR_3, GPO, 0, GPIO_PIN_CONFIG);      //��Ƭ���˿�D0 ��ʼ��DIR_1          GPIO
    gpio_init(DIR_4, GPO, 0, GPIO_PIN_CONFIG);      //��Ƭ���˿�D1 ��ʼ��DIR_2          GPIO
    pwm_init(PWM_3, 17000, 0);                      //��Ƭ���˿�D2 ��ʼ��PWM_1����10K ռ�ձ�0
    pwm_init(PWM_4, 17000, 0);                      //��Ƭ���˿�D3 ��ʼ��PWM_2����10K ռ�ձ�0
}
//��������ʼ��
void config_enc()
{
	//��ʼ�� QTIMER_1 A��ʹ��QTIMER1_TIMER0_C0 B��ʹ��QTIMER1_TIMER1_C1
    qtimer_quad_init(QTIMER_1,QTIMER1_TIMER0_C0,QTIMER1_TIMER1_C1);
    //��ʼ�� QTIMER_1 A��ʹ��QTIMER1_TIMER2_C2 B��ʹ��QTIMER1_TIMER3_C24
    qtimer_quad_init(QTIMER_1,QTIMER1_TIMER2_C2,QTIMER1_TIMER3_C24);
    qtimer_quad_init(QTIMER_2,QTIMER2_TIMER0_C3,QTIMER2_TIMER3_C25);
    qtimer_quad_init(QTIMER_3,QTIMER3_TIMER2_B18,QTIMER3_TIMER3_B19);
	qtimer_quad_clear(QTIMER_1, QTIMER1_TIMER0_C0);
	qtimer_quad_clear(QTIMER_1,QTIMER1_TIMER2_C2);
	qtimer_quad_clear(QTIMER_2,QTIMER2_TIMER0_C3);
	qtimer_quad_clear(QTIMER_3,QTIMER3_TIMER2_B18);

}
//���ڳ�ʼ��
void config_uart()
{
	
	//����1
	uart_init(USART_1,1000000,UART1_TX_B12,UART1_RX_B13);
	uart_rx_irq(USART_1,1);
	//uart_tx_irq(USART_1,1);
	EnableIRQ(LPUART1_IRQn);
	NVIC_SetPriority(USART_1,0);
	//����3
	uart_init(USART_3,9600,UART3_TX_C8,UART3_RX_C9);
	uart_rx_irq(USART_3,1);
	//uart_tx_irq(USART_3,1);
	EnableIRQ(LPUART3_IRQn);
	NVIC_SetPriority(USART_3,2);
	//����4
	//OpenArt �� rt1064ͨ�ŵĲ����ʲ�Ҫ����̫�ߣ��������rt1064���ռĴ��������������
	//�����ʵ�飬19200,9600�Ϻ�
	uart_init(USART_4,9600,UART4_TX_C16,UART4_RX_C17);
	uart_rx_irq(USART_4,1);
	//uart_tx_irq(USART_4,1);
	EnableIRQ(LPUART4_IRQn);
	NVIC_SetPriority(USART_4,2);
	
	//����ת���ڣ�����8��
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
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
    /* ���� printf ʹ�� LPUART1 ���������޸� */
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
    
	GPT_GetDefaultConfig(&gptConfig);           //��ȡĬ������
	GPT_Init(clcok_gpt, &gptConfig);            //GPT��ʼ�� ���ڴ�ʱ��
	GPT_Deinit(clcok_gpt);                      //GPT����ʼ��
	GPT_Init(clcok_gpt, &gptConfig);            //GPT��ʼ��
	GPT_SetClockDivider(clcok_gpt, clock_gpt_div);    //���÷�Ƶϵ��
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
 *@description		:��ʼ��GPT1��ʱ��
 *@param			:��
 *@return			:��
 */
void gpt1_init(void)
{
	GPT1->CR = 0;	/*����*/
	GPT1->CR |= (1 << 15);	/*�����λ*/
	while((GPT1->CR >> 15) & 0x01);	/*�ȴ������λ�ɹ�*/
	GPT1->CR |= (1 << 6);	/*
							*����ģʽ��restartģʽ
							*ʱ��Դ��ipg_clk=66MHz
							*��ʱ���ر�ʱ��������ֵ
							*/
	GPT1->PR = (66-1);	/*���÷�Ƶ��Ϊ66��Ƶ*/
	GPT1->OCR[0] = 0XFFFFFFFF;	/*��������Ƚ�ͨ��һ�ļĴ���ֵ*/
	GPT1->CR |= (1 << 0);	/*ʹ��GPT��ʱ��*/
	GPT_StartTimer(clcok_gpt);
}



//-------------------------------------------------------------------------------------------------------------------
void send_msg(uint16 pos_x, uint16 pos_y, uint8_t type)
{
	uint8_t msg[18] = {0};
	uint16_t now_time =  get_time_ms();
	uint16_t second = now_time/1000;
	uint16_t msecond = now_time%1000;
    //ʱ����Ϣ
    msg[0] = second/100 + '0';
    msg[1] = second%100/10 + '0';
    msg[2] = second%10 + '0';
    msg[3] = '.';
    msg[4] = msecond/100 + '0';
    msg[5] = msecond%100/10 + '0';
    msg[6] = msecond%10 + '0';
	
    pos_x /= 20;
	pos_y /= 20;
     //����X��Ϣ
    msg[7] = ' ';
    msg[8] = pos_x/10 + '0';
    msg[9] = pos_x%10 + '0';
    
    //����Y��Ϣ
    msg[10] = ' ';
    msg[11] = pos_y/10 + '0';
    msg[12] = pos_y%10 + '0';
    
    //��� ������Ϣ
    msg[13] = ' ';
    msg[14] = (uint8)type/10 + '0';
    
    //��� С����Ϣ
    msg[15] = ' ';
    msg[16] = (uint8)type%10 + '0';
    
    msg[17] = '\n';
    uart_putbuff(USART_8, msg, 18);
}
//��Ļ��ʾ
void lcdshow_sys(uint8_t obj_num)
{
	uint16_t up = 0,down = 8,x_center = 50,left = 0,right = left + 80; //����
	//ÿ����Ŀ
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