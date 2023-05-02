#ifndef __IMU_H__
#define __IMU_H__
#include "stdint.h"
#include "headfile.h"

bool line_dis(float cm);
void side_dis(float cm);
extern uint8_t pos_num;
extern uint8_t pos_sum; 
extern uint8_t fin_flag;
//转一个固定的角度
void turn_angle(float angle);
//直线跑一个固定的距离
bool run_straight(int8_t velocity,uint16_t cm);
//全向移动
void run_free();
//转到指定场地坐标系方向
int16_t turn_Yaw();
//编码器和加速度计计算车体瞬时速度
void xy_vel();
//转向完成标志
extern bool turn_fin;
void acc_deal();
//方向控制
int16_t dir_control();
//计算偏航角
void cal_rollAngle();
//车身当前位置
void pos_now();
typedef struct 
{
	//加速度计计算加速度
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	//编码器计算加速度
	float aenc_x;
	float aenc_y;
	//编码器计算速度
	float encc_x;
	float encc_y;
	//角速度
	float gyro_x;
	float gyro_y;
	float gyro_z;
	//角度
	float Pitch;
	float Yaw;
	float Roll;
	float exp_dir;   //期望方向
	float enc_update;
	//位置
	float now_x;
	float now_y;
	//自检
	float shift_value;  //陀螺仪零偏
	float acc_x_value;  //加速度计x轴零偏
	float acc_y_value;  //加速度计y轴零偏
	//角度环pid
	float kpid_in[3];
	float kpid_out[3];
	float kpid_out1[3];
	//需要跑的距离
	float rem_x;
	float rem_y;

}IMU_;
extern IMU_ IMU;
//IMU初始化
void imu_init();
	
//欧拉角
typedef struct{
	float euler_ang_le[3];
}Vector3f;

//四元数
typedef struct{
	float q0;
	float q1;
	float q2;
	float q3;
}quat_;
#endif
