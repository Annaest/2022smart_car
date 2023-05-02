#include "imu.h"
#include "headfile.h"
#include "self_headfile.h"
#include "math.h"

IMU_ IMU;
#define pi 3.14
bool turn_fin = 0;
uint8_t fin_flag = 0;   //����һ��Ŀ���ı�־
//תһ���̶��ĽǶ�
void turn_angle(float angle)
{
	if(angle > 0)
	{
		for(int16_t i = 0;i<angle;i++)
		{
			IMU.exp_dir -= 1;
			systick_delay_ms(5);
		}
	}
	else if(angle < 0)
	{
		for(int16_t i = 0;i<-angle;i++)
		{
			IMU.exp_dir += 1;
			systick_delay_ms(5);
		}
	}
	
}
//��һ��ֱ�߾���
bool run_straight(int8_t velocity,uint16_t dis_cm)
{
	static float start_dis = 0;
	static bool flag = 0;
	//�ĸ�������ƽ������
	int16_t now_dis = enc[0].dis;
	//���ϸ���������Ѿ��ܵľ���
	int16_t over_dis = 0;   
	//��ʣ�¶��پ���Ҫ��
	int16_t rem_dis = 0;
	static bool stop_flag = 0;
	//��¼��ʼλ��
	if(flag == 0)
	{
		start_dis = enc[0].dis;
	}
	over_dis = abs(now_dis - start_dis);
	//ʣ��ľ���
	rem_dis = dis_cm - over_dis;
	//printf("dis:%f\r\n",rem_dis);
	//û��������
	if(rem_dis > 0)
	{
		flag = 1;
		vel_x = velocity;
		
	}
	//�ܵ�
	else if(rem_dis <= 0)
	{
		vel_x = 0;
		stop_flag = 1;
	}
	if(stop_flag == 1 && abs(enc[0].vel) <= 10 && abs(enc[1].vel) <= 10
					&& abs(enc[2].vel) <= 10 && abs(enc[3].vel) <= 10){
		flag = 0;
		stop_flag = 0;
	}
	return flag;
}
void xy_vel()
{
	//���������㳵��˲ʱ�ٶ�
	static float encc_x_p = 0,encc_y_p = 0;
	float encc_x;
	float encc_y;
	float k1 = 0.5;
	float k2 = 0.5;
	static float acc_x_p = 0;
	static float acc_y_p = 0;
	float acc_x = 0;
	float acc_y = 0;
	//������ٶ�
	encc_x = (enc[0].vel + enc[1].vel + enc[2].vel + enc[3].vel)/4.0;
	encc_y = (-enc[0].vel + enc[1].vel + enc[2].vel - enc[3].vel)/4.0;
	//����˲ʱ�ٶ�
	encc_x = (encc_x + encc_x_p)/2.f;
	encc_y = (encc_y + encc_y_p)/2.f;
	//һ�׻����˲������ٶ�
	IMU.encc_x = encc_x*k1 + (1-k1)*encc_x_p;
	IMU.encc_y = encc_y*k1 + (1-k1)*encc_y_p;
	//������ٶ�
	IMU.aenc_x = IMU.encc_x - encc_x_p;
	IMU.aenc_y = IMU.encc_y - encc_y_p;
	//��¼��һ��
	encc_x_p = IMU.encc_x;
	encc_y_p = IMU.encc_y;
	//���ٶȼƼ��㳵��˲ʱ�ٶ�
	get_icm20602_accdata_spi();
	acc_x = -(icm_acc_x - IMU.acc_x_value)/10.f;
	acc_y = -(icm_acc_y - IMU.acc_y_value)/10.f;
	IMU.acc_x = acc_x*k2 + (1-k2)*acc_x_p;
	IMU.acc_y = acc_y*k2 + (1-k2)*acc_y_p;;
	acc_x_p = acc_x;
	acc_y_p = acc_y;
}

//ȫ���ƶ�
void run_free()
{
	
	static float vel_x_p = 0;
	static float vel_y_p = 0;
	static float sum_x = 0;
	static float sum_y = 0;
	float yu_x = 7.f;
	float yu_y = 7.f;
	float velx_k = 6.f;
	float vely_k = 5.5f;
	float k1 = 1/velx_k*0.5;
	float k2 = 1/vely_k*0.5;
//	float velx_k = 8.f;
//	float vely_k = 7.f;
//	float k1 = 1;
//	float k2 = 1;
	//������ǿ
//	if(pos_num == 0)
//	{
//		vely_k = 6.f;
//	}
	//a�ٶȹ滮
	if(pos_num >= 10 && work_state != 5)
	{
		velx_k = 6.5f;
		vely_k = 6.5f;
		yu_x = 7.f;
		yu_y = 7.f;
	}
	if(work_state == 4)
	{
		velx_k = 10.f;
		vely_k = 8.f;
		yu_x = 7.f;
		yu_y = 7.f;
	}
	if(fabs(IMU.rem_x) - fabs(sum_x) > 0)
	{		
		sum_x = IMU.rem_x;
	}
	if(fabs(IMU.rem_y) - fabs(sum_y) > 0)
	{
		sum_y = IMU.rem_y;
	}
	IMU.rem_x -= IMU.encc_x/54.76f * pit0_time/1000.0f;
	IMU.rem_y -= IMU.encc_y/54.76f * pit0_time/1000.0f;
	vel_x = nolinar_error(velx_k,IMU.rem_x)*k1 + (1-k1)*vel_x_p;
	vel_x_p = vel_x;
	vel_y = nolinar_error(vely_k,IMU.rem_y)*k2 + (1-k2)*vel_y_p;
	vel_y_p = vel_y;
	
//	if(fabs(IMU.rem_x) > fabs(sum_x)/2.f)
//	{
//		vel_x = nolinar_error(velx_k,sum_x - IMU.rem_x)*k1 + (1-k1)*vel_x_p;
//	}
//	else
//	{
//		vel_x = nolinar_error(velx_k,IMU.rem_x)*k1 + (1-k1)*vel_x_p;
//	}
//	vel_x_p = vel_x;
//	if(fabs(IMU.rem_y) > fabs(sum_y)/2.f)
//	{
//		vel_y = nolinar_error(vely_k,sum_y - IMU.rem_y)*k2 + (1-k2)*vel_y_p;
//	}
//	else
//	{
//		vel_y = nolinar_error(vely_k,IMU.rem_y)*k2 + (1-k2)*vel_y_p;
//	}
//	vel_y_p = vel_y;

	if(fabs(IMU.rem_x) <= yu_x && fabs(IMU.rem_y) <= yu_y && fin_flag == 0)
	{
		IMU.rem_x = 0;
		IMU.rem_y = 0;
		vel_x = 0;
		vel_y = 0;
		fin_flag = 1;
		sum_x = 0;
		sum_y = 0;
		return;
	}


}

//�������
int16_t turn_Yaw()
{
	//�⻷λ��ʽ
	float *kpid_out = IMU.kpid_out;
	//�ڻ�����ʽ
	float *kpid_in = IMU.kpid_in;
	static float error_p_out = 0.f;
	static float error_p_in1 = 0.f;
	static float error_p_in2 = 0.f;
	static float pid_value = 0.f;
	float error = 0.f;
	float dpid = 0.f;
	float I = 0;
	//����������ת����
	IMU.exp_dir = ((int16_t)(IMU.exp_dir*10) % 3600)/10.0f;
	error = IMU.Yaw - IMU.exp_dir;
	if(fabs(360 - fabs(IMU.Yaw - IMU.exp_dir)) >= fabs(IMU.Yaw - IMU.exp_dir))
		error = IMU.Yaw - IMU.exp_dir;
	else{
		error = -error/fabs(error) * (360 - fabs(IMU.Yaw - IMU.exp_dir));
	}
	
	//�⻷λ��ʽ
	int16_t output = kpid_out[0] * error + kpid_out[2] * (error - error_p_out);
	error_p_out = error;
	error = output;
	
//	I += error;
//	//�����޷�
//	uint8_t k = 5;
//	if((error >=0 && I >= k*error) || (error < 0 && I <= k*error)) 
//	{		
//		I = k*error;
//	}
//	I = I>10?10:I;
//	I = I<-10?-10:I;
//	//�ڻ�λ��ʽ
//	pid_value = kpid_in[0] * error + kpid_in[1] * I + kpid_in[2] * (error - error_p_in1);
//	error_p_in1 = error;

	//�ڻ�����ʽ
	dpid = kpid_in[0] * (error - error_p_in1) + 
				 kpid_in[1] * error + 
				 kpid_in[2] * (error - 2*error_p_in1 + error_p_in2);
	error_p_in2 = error_p_in1;
	error_p_in1 = error;
	//���ٻ���

	if(fabs(error) <= 10.f)
	{
		pid_value += dpid;
	}
	else if(fabs(error) <= 15.f)
	{
		pid_value += 0.6*dpid;
	}
	else
	{
		pid_value += 0.1*dpid;
	}
	//�����޷�
	uint8_t k = 6;
	if((error >=0 && pid_value >= k*error) || (error < 0 && pid_value <= k*error)) 
	{		
		pid_value = k*error;
	}
	pid_value = pid_value<=100?pid_value:100;
	pid_value = pid_value>=-100?pid_value:-100;
	return pid_value;
}

int32_t* slide_dec(int32_t* pid_slide)
{
	int16_t error_x = 0;
	int16_t error_y = 0;
	float pid_x[3] = {0,0,0};
	float pid_y[3] = {0,0,0};
	static int16_t error_x_p1 = 0;
	static int16_t error_y_p1 = 0;
	static int16_t error_x_p2 = 0;
	static int16_t error_y_p2 = 0;
	error_x = IMU.acc_x - IMU.aenc_x;
	error_y = IMU.acc_y - IMU.aenc_y;
	*pid_slide += pid_x[0]*(error_x - error_x_p1) + pid_x[1]*error_x
					+ pid_x[2]*(error_x - 2*error_x_p1 + error_x_p2);
	*(pid_slide+1) += pid_y[0]*(error_y - error_y_p1) + pid_y[1]*error_y
					+ pid_y[2]*(error_y - 2*error_y_p1 + error_y_p2);
	
	return pid_slide;
}
/*
* һ���������������Ԫ��
* ��Ԫ����ʼֵ: q0 = 1, q1 = q2 = q3 = 0
* Wx Wy WzΪ�����ʵ�λΪ����
* Wx = gx*(PI/180) WY = gy*(PI/180) Wz = gz*(PI/180)
* q0(t+dt) = q0(t) + 1/2dt * (-Wx*q1 - Wy*q2 - Wz*q3)
* q1(t+dt) = q1(t) + 1/2dt * ( Wx*q0 - Wy*q3 + Wz*q2)
* q2(t+dt) = q2(t) + 1/2dt * ( Wx*q3 + Wy*q0 - Wz*q1)
* q3(t+dt) = q3(t) + 1/2dt * (-Wx*q2 + Wy*q1 + Wz*q0)
*/
quat_ gyro_2_quaternion(IMU_ IMU,float dt)
{
    float half_dt = 0.5*dt*0.001; // ��λΪ��
	
	static quat_ quat = {1,0,0,0};
	float Wx = IMU.gyro_x*pi/180;
	float Wy = IMU.gyro_y*pi/180;
	float Wz = IMU.gyro_z*pi/180;
    quat.q0 += half_dt * (- Wx* quat.q1 - Wy*quat.q2 - Wz*quat.q3);
    quat.q1 += half_dt * ( Wx * quat.q0 - Wy*quat.q3 + Wz*quat.q2);
    quat.q2 += half_dt * ( Wx * quat.q3 + Wy*quat.q0 - Wz*quat.q1);
    quat.q3 += half_dt * (-Wx * quat.q2 + Wy*quat.q1 + Wz*quat.q0);
	
    float normal = sqrt(quat.q0*quat.q0 + quat.q1*quat.q1 + quat.q2*quat.q2 + quat.q3*quat.q3);
    quat.q0 /= normal;
    quat.q1 /= normal;
    quat.q2 /= normal;
    quat.q3 /= normal;
	
    //printf("Quaternion is %.3f %.3f %.3f %.3f\n", quat.q0, quat.q1, quat.q2, quat.q3);
	return quat;
}

/*
* ��Ԫ��תŷ����
* 
* 
* 
*/
float quaternion_2_euler_angle(quat_ quat)
{
	float q0 = quat.q0;
	float q1 = quat.q1;
	float q2 = quat.q2;
	float q3 = quat.q3;
#if 1
    // NED����������ϵ�µ�Z-Y-X��ת
    float roll = atan2f(2.0f*(q0*q1 + q2*q3), (q0*q0 - q1*q1 - q2*q2 + q3*q3));
    //float pitch = safe_asin(2.0f*(q0*q2 - q1*q3));
    float yaw = atan2f(2.0f*(q0*q3 + q1*q2) , (q0*q0 + q1*q1 - q2*q2 - q3*q3));
#else
    // ENU����������ϵ�µ�Z-X-Y��ת
    float roll = -atan2f(2.0f*(q1*q3 - q0*q2), (q0*q0 - q1*q1 - q2*q2 + q3*q3));
    //float pitch = safe_asin(2.0f*(q0*q1 + q2*q3));
    float yaw = atan2f(2.0f*(q1*q2 - q0*q3), (q0*q0 - q1*q1 + q2*q2 - q3*q3));
#endif
    
    return yaw;
}


void acc_deal()
{
	get_icm20602_accdata_spi();
	
	//printf("%f\r\n",icm_acc_z * 9.80665f * 0.001);
}
//IMU��ʼ��
void imu_init()
{
	IMU.acc_x = 0;
	IMU.acc_y = 0;
	IMU.acc_z = 0;
	//���ٶ�
	IMU.gyro_x = 0;
	IMU.gyro_y = 0;
	IMU.gyro_z = 0;
	//�Ƕ�
	IMU.Pitch = 0;
	IMU.Yaw = 0;
	IMU.Roll = 0;
	//��������
	IMU.exp_dir = 0;
	//����λ��
	IMU.now_x = 0;
	IMU.now_y = 0;
	//�ǶȻ�pid����
//	float kpid_out[3] = {2.0f,0.0f,4.7f};   //С��P
//	float kpid_in[3] = {3.5f,1.5f,4.9f};
	float kpid_out[3] = {1.f,0.f,2.8f};   //С��P
	float kpid_in[3] = {0.6f,1.f,1.f};
	IMU.kpid_in[0] = kpid_in[0];
	IMU.kpid_in[1] = kpid_in[1];
	IMU.kpid_in[2] = kpid_in[2];
	IMU.kpid_out[0] = kpid_out[0];
	IMU.kpid_out[1] = kpid_out[1];
	IMU.kpid_out[2] = kpid_out[2];
	//imu�Լ�
	for(uint16_t i = 0;i<1000;i++)
	{
		get_icm20602_gyro_spi();
		get_icm20602_accdata_spi();
		IMU.shift_value += icm_gyro_z;
		IMU.acc_x_value += icm_acc_x;
		IMU.acc_y_value += icm_acc_y;
		systick_delay_ms(2);
		
	}
    IMU.shift_value /= 1000.f;
	//vofa_data[6] = (float)IMU.shift_value;
	IMU.shift_value += 0.27f;
	//
	IMU.acc_x_value /= 1000.f;
	IMU.acc_y_value /= 1000.f;

}
//����ƫ����
void cal_rollAngle()
{
	//ȡֵ���ʱ��
	static uint32_t dt_ = 0;
	int32_t dt;
	static int16_t gyro_z_p = 0;
	//��������ȡֵ���
	dt = get_time_ms() - dt_;
	dt_ = get_time_ms();

	get_icm20602_gyro_spi();

	IMU.gyro_z = -(icm_gyro_z - IMU.shift_value);     //M:21.85   H:1.769
	//vofa_data[6] = (float)IMU.gyro_z;
	
	//��ͨ�˲�
	float alpha = 0.8;
	IMU.gyro_z = IMU.gyro_z*alpha + (1-alpha)*gyro_z_p;
	gyro_z_p = IMU.gyro_z;
	//IMU.gyro_z = kalmanFilter_filter(kf+5,IMU.gyro_z);

	IMU.gyro_z /= 16.4;
	
	//���ֳ��Ƕ�
	IMU.Yaw +=  IMU.gyro_z*dt*0.001;
	if(IMU.Yaw >= 360) 	IMU.Yaw = 0;
	if(IMU.Yaw < 0 )   IMU.Yaw = 360 + IMU.Yaw;
	
	//��Ԫ����Ƕ�
//	quat_ quat = gyro_2_quaternion(IMU,dt);
//	float yaw = quaternion_2_euler_angle(quat)/pi*180;

	//acc_deal();

	
}

int16_t dir_control(){
	static float error_p1 = 0,error_p2 = 0;
	static float output = 0;
	float kp = 800, ki = 600, kd = 0;
	//����������ת����
	float error = IMU.exp_dir - IMU.Yaw;
	if(fabs(360 - fabs(IMU.exp_dir - IMU.Yaw)) >= fabs(IMU.exp_dir - IMU.Yaw))
		error = IMU.exp_dir - IMU.Yaw;
	else
		error = -error/fabs(error) * (360 - fabs(IMU.exp_dir - IMU.Yaw));
	error = nolinar_error(1,error);
	//ֻ����ֱ�ߵ������
	if((vel_x != 0 || vel_y != 0) && vel_w == 0){
		output = kp*(error - error_p1) + ki * error + kd*(error - 2*error_p1 + error_p2);
		//�����޷�
		uint8_t k = 100;
		if((error >=0 && pid->output >= k*error) || (error < 0 && pid->output <= k*error)) 
		{		
			output = k*error;
		}
		error_p2 = error_p1;
		error_p1 = error;
		//printf("error:%f,output:%f\r\n",error,output);
	}
	else{
		output = 0;
	}
	return (int16_t)output;
}
//���㳵Ŀǰ��λ��
void pos_now()
{
	static float start_dis = 0;
	static uint8_t start_flag = 0;
	if(work_state == 3){
		IMU.now_x = pos_XY[pos_num][0] - 30*cos(IMU.Yaw);
		IMU.now_y = pos_XY[pos_num][1] - 30*sin(IMU.Yaw);
		start_flag = 0;
	}
//	else if(work_state == 1){
//		if(vel_x != 0 && vel_y == 0 && vel_w == 0){
//			//��ʼ��־
//			if(start_flag == 0){
//				start_flag = 1;
//				start_dis = enc[0].dis;
//			}
//			else if(start_flag == 1){
//				IMU.now_x = pos_XY[pos_num][0] + (enc[0].dis - start_dis)*cos(IMU.Yaw);
//				IMU.now_y = pos_XY[pos_num][1] + (enc[0].dis - start_dis)*sin(IMU.Yaw);
//			}
//			
//		}
//	}
}