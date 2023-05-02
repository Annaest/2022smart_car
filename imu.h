#ifndef __IMU_H__
#define __IMU_H__
#include "stdint.h"
#include "headfile.h"

bool line_dis(float cm);
void side_dis(float cm);
extern uint8_t pos_num;
extern uint8_t pos_sum; 
extern uint8_t fin_flag;
//תһ���̶��ĽǶ�
void turn_angle(float angle);
//ֱ����һ���̶��ľ���
bool run_straight(int8_t velocity,uint16_t cm);
//ȫ���ƶ�
void run_free();
//ת��ָ����������ϵ����
int16_t turn_Yaw();
//�������ͼ��ٶȼƼ��㳵��˲ʱ�ٶ�
void xy_vel();
//ת����ɱ�־
extern bool turn_fin;
void acc_deal();
//�������
int16_t dir_control();
//����ƫ����
void cal_rollAngle();
//����ǰλ��
void pos_now();
typedef struct 
{
	//���ٶȼƼ�����ٶ�
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	//������������ٶ�
	float aenc_x;
	float aenc_y;
	//�����������ٶ�
	float encc_x;
	float encc_y;
	//���ٶ�
	float gyro_x;
	float gyro_y;
	float gyro_z;
	//�Ƕ�
	float Pitch;
	float Yaw;
	float Roll;
	float exp_dir;   //��������
	float enc_update;
	//λ��
	float now_x;
	float now_y;
	//�Լ�
	float shift_value;  //��������ƫ
	float acc_x_value;  //���ٶȼ�x����ƫ
	float acc_y_value;  //���ٶȼ�y����ƫ
	//�ǶȻ�pid
	float kpid_in[3];
	float kpid_out[3];
	float kpid_out1[3];
	//��Ҫ�ܵľ���
	float rem_x;
	float rem_y;

}IMU_;
extern IMU_ IMU;
//IMU��ʼ��
void imu_init();
	
//ŷ����
typedef struct{
	float euler_ang_le[3];
}Vector3f;

//��Ԫ��
typedef struct{
	float q0;
	float q1;
	float q2;
	float q3;
}quat_;
#endif
