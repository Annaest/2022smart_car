#include "kalman.h"
#include "stdint.h"
#include "self_headfile.h"
#include "headfile.h"
/**
 *@function: - 卡尔曼滤波器初始化
 *@kalmanFilter：卡尔曼滤波器结构体
 *@init_x：待测量的初始值
 *@init_p：后验状态估计值误差协方差的初始值
 */
void kalmanFilter_init(KalmanStructTypedef *kalmanFilter, float init_x, float init_p,float predict_q,float newMeasured_q)
{
    kalmanFilter->x = init_x;//待测量的初始值，如有中值一般设成中值
    kalmanFilter->p = init_p;//后验状态估计值误差协方差的初始值（不要为0问题不大）
    kalmanFilter->A = 1;
    kalmanFilter->H = 1;
    kalmanFilter->q = predict_q;//预测（过程）噪声方差 影响收敛速率，可以根据实际需求给出
	float ave = 0;
	float fangcha = 0;
	float temp_gyro_p = 0;
	float temp_gyro = 0;
	float alpha = 0.1;
	//计算方差
//	for(uint16_t i = 0;i<100;i++)
//	{
//		get_icm20602_gyro_spi();
//		//一阶互补滤波
//		temp_gyro = icm_gyro_z*alpha + (1-alpha)*temp_gyro_p;
//		temp_gyro_p = temp_gyro;
//		ave += temp_gyro;
//		systick_delay_ms(5);
//	}
//    ave /= 100.f;
//	for(uint16_t i = 0;i<100;i++)
//	{
//		get_icm20602_gyro_spi();
//		//一阶互补滤波
//		temp_gyro = icm_gyro_z*alpha + (1-alpha)*temp_gyro_p;
//		temp_gyro_p = temp_gyro;
//		fangcha += temp_gyro*temp_gyro - ave*ave;
//		systick_delay_ms(5);
//		
//	}
//	vofa_data[6] = (float)fangcha;
    kalmanFilter->r = 200.0f;//测量（观测）噪声方差R，可以通过实验手段获得
}

/**
 *@function: - 卡尔曼滤波器
 *@kalmanFilter:卡尔曼结构体
 *@newMeasured；测量值
 *返回滤波后的值
 */
float kalmanFilter_filter(KalmanStructTypedef *kalmanFilter, float newMeasured)
{

    /* Predict */
    kalmanFilter->x = kalmanFilter->A * kalmanFilter->x;//x的先验估计由上一个时间点的后验估计值和输入信息给出
    kalmanFilter->p = kalmanFilter->A * kalmanFilter->A * kalmanFilter->p + kalmanFilter->q;  /*计算先验均方差 p(n|n-1)=A^2*p(n-1|n-1)+q */

    /* Correct */
    kalmanFilter->gain = kalmanFilter->p * kalmanFilter->H / (kalmanFilter->p * kalmanFilter->H * kalmanFilter->H + kalmanFilter->r);
    kalmanFilter->x = kalmanFilter->x + kalmanFilter->gain * (newMeasured - kalmanFilter->H * kalmanFilter->x);//利用残余的信息改善对x(t)的估计，给出后验估计，这个值也就是输出
    kalmanFilter->p = (1 - kalmanFilter->gain * kalmanFilter->H) * kalmanFilter->p;//%计算后验均方差

    return kalmanFilter->x;//得到现时刻的最优估计
}