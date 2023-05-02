#include "headfile.h"
#include "self_headfile.h"
void move_control()
{
	int16_t k_rate = 500;
	int32_t final_vel_1 = 0;
	int32_t final_vel_2 = 0;
	int32_t final_vel_3 = 0;
	int32_t final_vel_4 = 0;
	int32_t angle_out = 0;
	//int16_t output[4] = {0,0,0,0};
	//pid初始化
	static uint8_t i = 0;
	if(i==0)
	{
		set_pid(p+0);
		set_pid(p+1);
		set_pid(p+2);
		set_pid(p+3);
		i = 1;
	}
	//计算每一个电机的速度
	motor[0].exp_vel = (vel_x - vel_y + vel_w) * k_rate;
	motor[1].exp_vel = (vel_x + vel_y - vel_w) * k_rate;
	motor[2].exp_vel = (vel_x + vel_y + vel_w) * k_rate;
	motor[3].exp_vel = (vel_x - vel_y - vel_w) * k_rate;
	//速度与方向解耦合
	//电机速度闭环计算
	pid_calculate(p+0,enc[0].vel,motor[0].exp_vel);
	pid_calculate(p+1,enc[1].vel,motor[1].exp_vel);
	pid_calculate(p+2,enc[2].vel,motor[2].exp_vel);
	pid_calculate(p+3,enc[3].vel,motor[3].exp_vel);
	//差速补偿
	//if(vel_x != 0 && vel_y == 0)   diffvel();
	//方向闭环控制
	angle_out = turn_Yaw()*k_rate;
	//滑移控制
	
	//传入电机最终速度
	final_vel_1 = motor[0].exp_vel + pid[0].output + angle_out;
	final_vel_2 = motor[1].exp_vel + pid[1].output - angle_out;
	final_vel_3 = motor[2].exp_vel + pid[2].output + angle_out;
	final_vel_4 = motor[3].exp_vel + pid[3].output - angle_out;
	set_motor(motor_p+0,PWM_1,DIR_1,final_vel_1);
	set_motor(motor_p+1,PWM_2,DIR_2,final_vel_2);
	set_motor(motor_p+2,PWM_3,DIR_3,final_vel_3);
	set_motor(motor_p+3,PWM_4,DIR_4,final_vel_4);

}
void set_motor(struct MOTOR_ *motor,PWMCH_enum pwmch,PIN_enum pin,int32_t rel_vel)
{
	//两次的速度差限制
//	motor->rel_vel = (rel_vel - motor->rel_vel_p)<100?rel_vel:motor->rel_vel_p+50;
//	motor->rel_vel = (rel_vel - motor->rel_vel_p)>-100?rel_vel:motor->rel_vel_p-50;
//	motor->rel_vel_p = motor->rel_vel;
	float k = 1.0f;
	motor->rel_vel = rel_vel*k + motor->rel_vel_p*(1-k);
//	motor->rel_vel = rel_vel;
	motor->pwmch = pwmch;
	motor->pin = pin;
	//限幅
	uint16_t max_vel = 40000;
	if(motor->rel_vel >= max_vel)        motor->rel_vel = max_vel;
	else if(motor->rel_vel <= -max_vel)  motor->rel_vel = -max_vel;
	motor->rel_vel_p = motor->rel_vel;
	//控制电机方向
	if(motor->rel_vel >= 0)          motor->dir = 0;
	else if(motor->rel_vel < 0) 
	{		
		motor->dir = 1;
		motor->rel_vel = motor->rel_vel * -1;
	}
	
	gpio_set(motor->pin,motor->dir);
	pwm_duty(motor->pwmch,(uint32_t)motor->rel_vel);
}


void uart_car(uint8_t receive_data)
{
	
	uint8_t speed = 0;
	static uint8_t flag = 0;
	static uint8_t data[7] = {'n','0','0','0','0','h'},i=1;
	if(receive_data == 'n') flag = 1;
	if(receive_data == 'h') flag = 2;
	if(flag == 1 && receive_data >= '0' && receive_data <= '9')
	{
		data[i] = receive_data;
		i++;
	}
	if(flag == 2)
	{
		i=1;
		flag = 0;
		speed = data[4] - '0' + (data[3] - '0')*10;
		switch(data[2] - '0' + (data[1] - '0')*10)
		{
			//uart_putchar(USART_8, (uint8_t)vel_x);	
			case 0:
				vel_x = 0;
				vel_y = 0;
				vel_w = 0;
				break;
			case 1:
				vel_x = speed;
				vel_y = 0;
				vel_w = 0;
				break;
			case 2:
				vel_x = -speed;
				vel_y = 0;
				vel_w = 0;
				break;
			case 3:
				vel_x = 0;
				vel_y = speed;
				vel_w = 0;
				break;
			case 4:
				vel_x = 0;
				vel_y = -speed;
				vel_w = 0;
				break;
			case 5:
				vel_x = 0;
				vel_y = 0;
				vel_w = speed;
				break;
			case 6:
				vel_x = 0;
				vel_y = 0;
				vel_w = -speed;
				break;
		}
	}
				
}

//差速补偿
void diffvel()
{
	float kp = 0.5,ki = 0,kd = 0.8;
	int16_t error1=0,error2=0,output1=0,output2=0;
	static int16_t error1_p=0,error2_p=0;
	error1 = enc[0].vel - enc[1].vel;
	output1 = kp*error1 + kd*(error1 - error1_p);
	motor[0].rel_vel = (motor[0].rel_vel + motor[1].rel_vel)/2 - output1;
	motor[1].rel_vel = (motor[0].rel_vel + motor[1].rel_vel)/2 + output1;
	error2 = enc[2].vel - enc[3].vel;
	output1 = kp*error2 + kd*(error2 - error2_p);
	motor[2].rel_vel = (motor[2].rel_vel + motor[3].rel_vel)/2 - output2;
	motor[3].rel_vel = (motor[2].rel_vel + motor[3].rel_vel)/2 + output2;
	error1_p = error1;
	error2_p = error2;
}

void event_stop()
{
	DisableGlobalIRQ();
	while(1)
	{
		vel_x = 0;
		vel_y = 0;
		vel_w = 0;
		enc_deal();
		move_control();
		printf("stop\r\n");
	}

}