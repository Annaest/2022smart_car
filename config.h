#ifndef __CONFIG_H__
#define __CONFIG_H__
#include "zf_pwm.h"
#include "stdint.h"
//电机
#define DIR_1 D1
#define DIR_2 D0
#define PWM_1 PWM2_MODULE3_CHB_D3
#define PWM_2 PWM2_MODULE3_CHA_D2 

#define DIR_3 D15
#define DIR_4 D14
#define PWM_3 PWM1_MODULE0_CHB_D13
#define PWM_4 PWM1_MODULE0_CHA_D12

#define wprint(data) uart_putchar(USART_8, data);

//定义按键引脚
#define KEY1    C31
#define KEY2    C27
#define KEY3    C26
#define KEY4    C4
//定义拨码开关引脚
#define SW1     D4
#define SW2     D27


extern uint8_t work_state;
#define BEEP_PIN   B11       //定义蜂鸣器引脚
void config_motor();
void config_gpio();
void config_enc();
void config_uart();
void vofa_print(float *data,uint8_t ch_num);
void delayms(uint16_t ms);
extern float vofa_data[7];
void test();
void config_key();
void clock_init();
uint32_t get_time_ms();
uint32_t get_time_us();
void gpt1_init();
void config_all();
bool go_next(int16_t next_x,int16_t next_y);
void state_control(uint8_t state);  //状态控制
void wireless_data();      //无线数据
void lcdshow_sys(uint8_t obj_num);        //屏幕显示系统
void send_msg(uint16 img_pos_x, uint16 img_pos_y, uint8_t type);
#endif