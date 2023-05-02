#include "serve.h"
#include "self_headfile.h"
int vV11=3500;
int aA11=250;

uint16 duty = 119*200;

#define S_MOTOR_PIN   PWM4_MODULE2_CHA_C30       //����������

void arrivel_n_1(void)//n=1������ʱ0.5s��n=2������ʱ0.85s��n=3������ʱ0.5s
{
  int pa1=119*200,pb1=2696;//��ʼλ��
  int pa2=164.5*200,pb2=3180;//����
  int pa3=152*200,pb3=3389;//̧��һ���
	//1
	//writepos(1, pb1, vV11, aA11);//2696:2100+596
	//pwm_duty(S_MOTOR_PIN,pa1);//161*200
	//systick_delay_ms(1000);
	//2
	gpio_set(B23,1);//�д���
	writepos(1, pb2, vV11, aA11);//2696:2100+596
	pwm_duty(S_MOTOR_PIN,pa2);//161*200
	systick_delay_ms(400);
	//3
	writepos(1, pb3, vV11, aA11);//2935:2339+596
	pwm_duty(S_MOTOR_PIN,pa3);//152*200
}

void arrivel_n_2(int n)
{
	int pa4,pb4;
	if(n==3)
	{
		pa4=119*200;pb4=1360;
	}
	else if(n==1)
	{
		pa4=111*200;pb4=1530;
	}
	else if(n==2)
	{
		pa4=103*200;pb4=1704;
	}
		//4
		writepos(1, pb4, vV11, aA11);
	
		systick_delay_ms(1);
		writepos(1, pb4, vV11, aA11);
		systick_delay_ms(1);
		writepos(1, pb4, vV11, aA11);
		pwm_duty(S_MOTOR_PIN,pa4);
		writepos(1, pb4, vV11, aA11);
		systick_delay_ms(1);
		writepos(1, pb4, vV11, aA11);
}


void putdown(void)//����
{
	gpio_set(B23,0);
	//writepos(3, 2460, vV11, aA11);
}
void arrivel_n_3(void)//�ӵ�n������Ĵ�ֱλ�õ���ʼλ�ã�����1���ſ��Խ�����һ��ץȡ��
{
	systick_delay_ms(100);
	int pa5=116*200,pb5=3150;;//��ʼλ��
	//5
	writepos(1, pb5, vV11, aA11);
	pwm_duty(S_MOTOR_PIN,pa5);
}

void writepos(int i,int p,int v,int a)
{
	int ID = 0x00;
	ID = i;
	int A = 0x00;
	A = a;
	int pos_low = 0x00;
	pos_low = p % 256;
	int pos_high = 0x00;
	pos_high = p / 256;
	int v_low = 0x00;
	v_low = v % 256;
	int v_high = 0x00;
	v_high = v / 256;
	int cor = 0x00;
	int cor2 = 0x00;
	cor2 = (0xFF + 0xFF + ID + 0x0A + 0x03 + 0x29 + A + pos_low + pos_high + 0x00 + 0x00 + 0x00 + v_low + v_high) % 256;
	cor = 0xFF - cor2 - 2;
	uart_putchar(USART_1,0xFF);
	uart_putchar(USART_1,0xFF);
	uart_putchar(USART_1,ID);//ID
	uart_putchar(USART_1,0x0A);
	uart_putchar(USART_1,0x03);
	uart_putchar(USART_1,0x29);
	uart_putchar(USART_1,A);//A
	systick_delay_ms(5);
	uart_putchar(USART_1,pos_low);//pos_low
	uart_putchar(USART_1,pos_high);//pos_high
	uart_putchar(USART_1,0x00);
	uart_putchar(USART_1,0x00);
	uart_putchar(USART_1,v_low);//v_low
	uart_putchar(USART_1,v_high);//v_high
	uart_putchar(USART_1,cor);//correct
}