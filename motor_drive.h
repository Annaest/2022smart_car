#ifndef __MOTOR_DRIVE__
#define __MOTOR_DRIVE__
#include "stdint.h"
#include "zf_pwm.h"

#define car_length      250
#define car_width       208
#define WHEEL_RADIUS    26
#define PI              3.14
#define MOTOR_TO_CENTER 536

//ǰ���ٶ�,�����ٶȣ���ת�ٶ�
extern int32_t vel_x,vel_y,vel_w;
void move_control();
void get_vel();
void uart_car(uint8_t receice_data);
void event_stop();
//�����
struct MOTOR_
{
	int32_t exp_vel;   //�����ٶ�
	int32_t rel_vel;   //ʵ���ٶ�
	int32_t rel_vel_p; //��һ��ʵ���ٶ�
	PWMCH_enum pwmch;  //pwm�˿�
	PIN_enum pin;      //��ƽ�˿�
	bool dir;          //����
	
	
};
extern struct MOTOR_ motor[4];
extern struct MOTOR_ *motor_p;
void set_motor(struct MOTOR_ *motor,PWMCH_enum pwmch,PIN_enum pin,int32_t exp_vel);
void diffvel();


#endif