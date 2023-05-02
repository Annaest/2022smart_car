#ifndef __PID_CONTROL__
#define __PID_CONTROL__
#include "stdint.h"

void *vel_pid_control(int32_t *set_vel);

//pid类
struct PID_
{
	
	//内环分段分速
	float kpid0[3];    //0-10000
	float kpid1[3];    //10000-20000
	float kpid2[3];    //20000-40000
	float kpid3[3];    //40000-50000
	//历史error
	int16_t error_p1;
	int16_t error_p2;
	int32_t output;
	//外环
	float out_kp;
	float out_kd;
	int16_t out_error_p1;
	
};
extern struct PID_ pid[4];
extern struct PID_ *p;
void set_pid(struct PID_ *pid);
void pid_calculate(struct PID_ *pid,int32_t enc_vel,int32_t exp_vel);
void adj_pid(struct PID_ *pid,uint8_t receive_data);
float nolinar_error(float k,float error);
#endif