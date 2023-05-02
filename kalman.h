#ifndef _kalman_H_
#define _kalman_H_
//�����������˲�
typedef struct {
    float x;  // ϵͳ��״̬��
    float A;  // x(n)=A*x(n-1)+u(n),u(n)~N(0,q)
    float H;  // z(n)=H*x(n)+w(n),w(n)~N(0,r)
    float q;  // Ԥ���������Э����
    float r;  // ������������Э����
    float p;  // �������Э����
    float gain;//����������
}KalmanStructTypedef;
extern KalmanStructTypedef kalmanFilter[6];
extern KalmanStructTypedef *kf;

void kalmanFilter_init(KalmanStructTypedef *kalmanFilter, float init_x, float init_p,float predict_q,float newMeasured_q);
float kalmanFilter_filter(KalmanStructTypedef *kalmanFilter, float newMeasured);
#endif