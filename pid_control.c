#include "headfile.h"
#include "stdio.h"
#include "self_headfile.h"
#define motor_vel_0 10000     //电机速度等级
#define motor_vel_1 20000
#define motor_vel_2 30000
#define motor_vel_3 40000


//增量式Pid计算
void pid_calculate(struct PID_ *pid,int32_t enc_vel,int32_t exp_vel)
{
	
	int32_t output_delta=0;
	float kpid[3];
	//误差计算
	int32_t error = exp_vel - enc_vel;
	//printf("enc_vel:%d\r\n",enc_vel);
	int16_t output = 0;
	//速度阶梯计算
	//-10000----10000
	if(enc_vel < motor_vel_0 && enc_vel > -motor_vel_0)
	{
		kpid[0] = pid->kpid0[0];
		kpid[1] = pid->kpid0[1];
		kpid[2] = pid->kpid0[2];
	}
	//-20000----20000
	else if(enc_vel < motor_vel_1 && enc_vel > -motor_vel_1)
	{
		kpid[0] = pid->kpid1[0];
		kpid[1] = pid->kpid1[1];
		kpid[2] = pid->kpid1[2];
	}
	//-30000---30000
	else if(enc_vel < motor_vel_2 && enc_vel > -motor_vel_2)
	{
		kpid[0] = pid->kpid2[0];
		kpid[1] = pid->kpid2[1];
		kpid[2] = pid->kpid2[2];
	}
	else
	{
		kpid[0] = pid->kpid3[0];
		kpid[1] = pid->kpid3[1];
		kpid[2] = pid->kpid3[2];
	}
	if(1)
	{
	//外环
		output = pid->out_kp * error + pid->out_kd * (error - pid->out_error_p1);
		pid->out_error_p1 = error;
		error = output;
	}
	//内环
	output_delta = kpid[0]*(error - pid->error_p1) + 
			 kpid[1]*error + 
		     kpid[2]*(error - 2*pid->error_p1 + pid->error_p2);
	pid->error_p2 = pid->error_p1;
	pid->error_p1 = error;
	
	//积分输出
	pid->output += output_delta;
	//积分限幅
	uint8_t k = 15;
	if((error >=0 && pid->output >= k*error) || (error < 0 && pid->output <= k*error)) 
	{		
		pid->output = k*error;
	}
	pid->output = pid->output<15000?pid->output:15000;
	pid->output = pid->output>-15000?pid->output:-15000;
	
	//printf("pid->output:%d\r\n",pid->output);

	
}


       
void set_pid(struct PID_ *pid)
{
//	float kpid0[3] = {0.65,0.15,0.1}; 
//	float kpid1[3] = {0.85,0.35,0.5};
//	float kpid2[3] = {1.15,0.55,0};
//	float kpid3[3] = {0,0,0};
//	pid->out_kp = 1.3;
//	pid->out_kd = 0.3;
	float kpid0[3] = {0.9f,0.45f,0.5f}; 
	float kpid1[3] = {1.3f,0.30f,1.3f};
	float kpid2[3] = {1.7f,0.20f,2.2f};
	float kpid3[3] = {0,0,0};
	pid->out_kp = 1.25f;
	pid->out_kd = 0.35f;
//	float kpid0[3] = {0.55,0.25,0.1}; 
//	float kpid1[3] = {0.75,0.35,0.5};
//	float kpid2[3] = {1.05,0.55,0};
//	float kpid3[3] = {0,0,0};
//	pid->out_kp = 1.3;
//	pid->out_kd = 0.5;
	pid->error_p1 = 0;
	pid->error_p2 = 0;
	pid->output = 0;
	//------------
	pid->kpid0[0] = kpid0[0];
	pid->kpid0[1] = kpid0[1];
	pid->kpid0[2] = kpid0[2];
	//------------
	pid->kpid1[0] = kpid1[0];
	pid->kpid1[1] = kpid1[1];
	pid->kpid1[2] = kpid1[2];
	//-----------
	pid->kpid2[0] = kpid2[0];
	pid->kpid2[1] = kpid2[1];
	pid->kpid2[2] = kpid2[2];
	//-----------
	pid->kpid3[0] = kpid3[0];
	pid->kpid3[1] = kpid3[1];
	pid->kpid3[2] = kpid3[2];
	
}

void adj_pid(struct PID_ *pid,uint8_t receive_data)
{
	//printf(":%d",receive_data);
	//记录小数点后几位数；小数点出现标志；kp,ki,kd标志
	static uint8_t dot_num = 0,d_flag = 0,k_flag = '0',kpid_value[10];
	static uint8_t i = 0;
	//存储pid参数
	static float kp = 0,ki = 0,kd = 0;
	//调速度等级
	float *pid_level[4];
	pid_level[0] = (pid+0)->kpid0;
	pid_level[1] = (pid+1)->kpid0;
	pid_level[2] = (pid+2)->kpid0;
	pid_level[3] = (pid+3)->kpid0;
	//调pid某个参数的标志
	if(receive_data == 'p')  k_flag = 'p';
	else if(receive_data == 'i')  k_flag = 'i';
	else if(receive_data == 'd')  k_flag = 'd';
	//-------------------
	if((k_flag == 'p' || k_flag == 'i' || k_flag == 'd') && receive_data >= '0' && receive_data <= '9') 
	{		
		kpid_value[i] = receive_data - '0';
		i++;
	}
	else if((k_flag == 'p' || k_flag == 'i' || k_flag == 'd') && receive_data == '.') 
	{
		kpid_value[i] = '.';
		i++;
	}
	else if(receive_data == 'e')
	{
		if(k_flag == 'p')
		{
			for(int j=0;j<i;j++)
			{
				//printf("-%d-",kpid_value[j]);
				if(d_flag == 1) dot_num++;
				if(kpid_value[j] == '.')  d_flag = 1;
				else  kp = kp*10 + kpid_value[j];
				
			}
			*(pid_level[0]+0) = kp/(float)pow(10,dot_num);
			*(pid_level[1]+0) = kp/(float)pow(10,dot_num);
			*(pid_level[2]+0) = kp/(float)pow(10,dot_num);
			*(pid_level[3]+0) = kp/(float)pow(10,dot_num);
		}
		else if(k_flag == 'i')
		{
			for(int j=0;j<i;j++)
			{
				if(d_flag == 1) dot_num++;
				if(kpid_value[j] == '.')  d_flag = 1;
				else  ki = ki*10 + kpid_value[j];
				
			}
			*(pid_level[0]+1) = ki/(float)pow(10,dot_num);
			*(pid_level[1]+1) = ki/(float)pow(10,dot_num);
			*(pid_level[2]+1) = ki/(float)pow(10,dot_num);
			*(pid_level[3]+1) = ki/(float)pow(10,dot_num);
		}
		else if(k_flag == 'd')
		{
			for(int j=0;j<i;j++)
			{
				if(d_flag == 1) dot_num++;
				if(kpid_value[j] == '.')  d_flag = 1;
				else  kd = kd*10 + kpid_value[j];
			}
			*(pid_level[0]+2) = kd/(float)pow(10,dot_num);
			*(pid_level[1]+2) = kd/(float)pow(10,dot_num);
			*(pid_level[2]+2) = kd/(float)pow(10,dot_num);
			*(pid_level[3]+2) = kd/(float)pow(10,dot_num);
		}
		printf("--------------------\r\n%f %f %f\r\n----------------",*(pid_level[0]+0),*(pid_level[0]+1),*(pid_level[0]+2));
		
		//重置
		k_flag = '0';
		dot_num = 0;
		d_flag = 0;
		kp = 0;
		ki = 0;
		kd = 0;
		i=0;
		for(int j=0;j<10;j++) kpid_value[i] = '0';
		
	}
		
}

float nolinar_error(float k,float error){
	//非线性
	int8_t k1 = 1;
	if(error < 0)
	{		
		k1= -k1;
		k = -k;
	}
	error = log(error*k1+1)*k;
	return error;
}
