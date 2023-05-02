#include "kalman.h"
#include "stdint.h"
#include "self_headfile.h"
#include "headfile.h"
/**
 *@function: - �������˲�����ʼ��
 *@kalmanFilter���������˲����ṹ��
 *@init_x���������ĳ�ʼֵ
 *@init_p������״̬����ֵ���Э����ĳ�ʼֵ
 */
void kalmanFilter_init(KalmanStructTypedef *kalmanFilter, float init_x, float init_p,float predict_q,float newMeasured_q)
{
    kalmanFilter->x = init_x;//�������ĳ�ʼֵ��������ֵһ�������ֵ
    kalmanFilter->p = init_p;//����״̬����ֵ���Э����ĳ�ʼֵ����ҪΪ0���ⲻ��
    kalmanFilter->A = 1;
    kalmanFilter->H = 1;
    kalmanFilter->q = predict_q;//Ԥ�⣨���̣��������� Ӱ���������ʣ����Ը���ʵ���������
	float ave = 0;
	float fangcha = 0;
	float temp_gyro_p = 0;
	float temp_gyro = 0;
	float alpha = 0.1;
	//���㷽��
//	for(uint16_t i = 0;i<100;i++)
//	{
//		get_icm20602_gyro_spi();
//		//һ�׻����˲�
//		temp_gyro = icm_gyro_z*alpha + (1-alpha)*temp_gyro_p;
//		temp_gyro_p = temp_gyro;
//		ave += temp_gyro;
//		systick_delay_ms(5);
//	}
//    ave /= 100.f;
//	for(uint16_t i = 0;i<100;i++)
//	{
//		get_icm20602_gyro_spi();
//		//һ�׻����˲�
//		temp_gyro = icm_gyro_z*alpha + (1-alpha)*temp_gyro_p;
//		temp_gyro_p = temp_gyro;
//		fangcha += temp_gyro*temp_gyro - ave*ave;
//		systick_delay_ms(5);
//		
//	}
//	vofa_data[6] = (float)fangcha;
    kalmanFilter->r = 200.0f;//�������۲⣩��������R������ͨ��ʵ���ֶλ��
}

/**
 *@function: - �������˲���
 *@kalmanFilter:�������ṹ��
 *@newMeasured������ֵ
 *�����˲����ֵ
 */
float kalmanFilter_filter(KalmanStructTypedef *kalmanFilter, float newMeasured)
{

    /* Predict */
    kalmanFilter->x = kalmanFilter->A * kalmanFilter->x;//x�������������һ��ʱ���ĺ������ֵ��������Ϣ����
    kalmanFilter->p = kalmanFilter->A * kalmanFilter->A * kalmanFilter->p + kalmanFilter->q;  /*������������� p(n|n-1)=A^2*p(n-1|n-1)+q */

    /* Correct */
    kalmanFilter->gain = kalmanFilter->p * kalmanFilter->H / (kalmanFilter->p * kalmanFilter->H * kalmanFilter->H + kalmanFilter->r);
    kalmanFilter->x = kalmanFilter->x + kalmanFilter->gain * (newMeasured - kalmanFilter->H * kalmanFilter->x);//���ò������Ϣ���ƶ�x(t)�Ĺ��ƣ�����������ƣ����ֵҲ�������
    kalmanFilter->p = (1 - kalmanFilter->gain * kalmanFilter->H) * kalmanFilter->p;//%������������

    return kalmanFilter->x;//�õ���ʱ�̵����Ź���
}