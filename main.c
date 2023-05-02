#include "SEEKFREE_FONT.h"
#include "headfile.h"
#include "self_headfile.h"
#include "time.h"
#include "math.h"
//#include "path_solve.h"
#include "ga_solve.h"
/* �ж����ȼ��� */
#define NVIC_Group0   0x07
#define NVIC_Group1   0x06
#define NVIC_Group2   0x05
#define NVIC_Group3   0x04
#define NVIC_Group4   0x03
#define pi 3.142
#define normal -1    //�Ƿ���Ҫ��ͼ

int32_t vel_x = 0;               //���ٶ�
int32_t vel_y = 0;
int32_t vel_w = 0;
//�������
struct MOTOR_ motor[4],*motor_p = motor;       //���
struct PID_ pid[4],*p = pid;             //Pid
struct ENC_ enc[4],*e = enc;             //������;
KalmanStructTypedef kalmanFilter[6],*kf = kalmanFilter; //6�Ῠ�����˲�
//��������

int16_t pos_XY[max_pot][2] = {0};
//int16_t pos_XY[max_pot][2] = {{450,350},{0,0},{100,300},{250,300},{350,400},{400,250},{300,100},{0,0}};
uint8_t pos_sum = 0;       

//��������
uint8_t pos_num = 0;           //�ڼ�������

bool go_next(int16_t next_x,int16_t next_y);   //��ͼ
void move_next(int16_t next_x,int16_t next_y);
float img_error[2] = {0,0};                    //������ͷ�������ͼƬ���� 

uint8_t error_flag = 0;                             //error����flag
int state1ok_flag = 0;

void approach_img();                            //ǰ��ӽ�ͼƬ
void state_control(uint8_t state);              //״̬����
float vofa_data[7] = {0,0,0,0,0,0,0};     
void state_0();
void state_1();
void state_2();
void state_3();
void state_4();
void state_5();
//vofa���ε���
uint8_t work_state = 0;   
uint8_t work_state_p = 0;
uint8_t pre_flag = 0;
int main(void)
{
	config_all();
//	test_pid_debug();
	state_control(0);
	
    while(1){
		switch(work_state){	
			case 0:
				state_0();
				break;
			//��ʼ��ͼ
			case 1:
				state_1();
				break;
			//��һ��Ŀ���΢��
			case 2:
				state_2();
				break;
			//�ȴ�ͼƬʶ����
			case 3:
				state_3();
				break;
			//����
			case 4:
				state_4();
				break;
			case 5:
				state_5();
				break;
		}
		work_state_p = work_state;
	
	}
}

void state_0()
{
	arrivel_n_3();
	//�ȴ�����
	if(normal != -1)
	{
		for(uint8_t i=0;i<12;i++)
		{
			pos_XY[i][0] = pre_map[normal][i][0];
			pos_XY[i][1] = pre_map[normal][i][1];
//			object_type[i] = pre_type[normal][i];
			pos_sum++;
		}

	}
	else
	{
		//�ȴ���������
		while(pos_XY[0][0] == 0 || pos_XY[0][1] == 0)
		{
			systick_delay_ms(1);
		}
		
		//�������
		//�����������
		for(int i=0;pos_XY[i][0]!=0 || pos_XY[i][1]!=0;i++){
			pos_sum++;
			
		}
		int8_t map_num = -1;
//		map_num = map_right();
		lcd_showuint8(1,1,map_num);
		//��ͼ����ѡ��
		if(map_num != -1)    //ѡ�е�ͼ
		{
			for(uint8_t i=0;i<12;i++)
			{
				pos_XY[i][0] = pre_map[map_num][i][0];
				pos_XY[i][1] = pre_map[map_num][i][1];
				object_type[i] = pre_type[map_num][i];
				
			}
			pre_flag = 1;
		}
		else     //δѡ�е�ͼ�������Ŵ�����·��
		{
//			double d = mygenus(pos_sum);
		}
	}
	
//	while(gpio_get(KEY2) == 1)
//	{
//		systick_delay_ms(10);
//	}
//	systick_delay_ms(1000);
	//��ʼλ��
	IMU.now_x = 30;
	IMU.now_y = -30;
//	state_control(1);
	work_state = 1;
}

void state_1()
{
	uint32_t putdown_time = 0;
//	//������
//	static uint8_t chuku = 0;
//	if(chuku == 0)
//	{
//		chuku++;
//		IMU.rem_y = 30;
//		//IMU.now_y = 30;
//		fin_flag = 0;
//		while(fin_flag == 0)
//		{
//			//�ȴ�����
//			systick_delay_ms(1);
//		}
//	}
	//go_next(pos_XY[pos_num][0],pos_XY[pos_num][1]);
	//���������ǰ��1��bug
	find_flag = 0;
	move_next(pos_XY[pos_num][0],pos_XY[pos_num][1]);
	
	
	//��е�۲���
	if(pos_num >= 1)
	{
//		lcd_showint8(0,0,object_type[two_con]/10);
//		lcd_showint8(0,1,two_con);
		arrivel_n_2(object_type[two_con]/10);
		
	}
	
	putdown_time = get_time_ms();
	uint8_t fanum = 0;       //���Ĵ���
	uint8_t putdowm_num = 0; //��ǰ�ŵı�־
	uint8_t ahead_flag = 0; //��ǰ�����ı�־
	while(1)
	{
		
		//�ȴ�����
		systick_delay_ms(1);
		if(pos_num >= 1 && object_type[two_con]%10 != 0 && fanum == 0)
		{
			fanum++;
			//�ȴ�ʶ��
			systick_delay_ms(1);
			send_msg(IMU.now_x, IMU.now_y, object_type[two_con]);
			//lcdshow_sys(two_con);
			two_con++;
		}
		if(get_time_ms() - putdown_time >= 700 && putdowm_num == 0)
		{
			//���Դ�΢������ͷ
			state1ok_flag = 1;   //��ʼ����img_error
			putdowm_num++;
			putdown();
			arrivel_n_3();
		}
		if(fin_flag == 1 && fanum == 1 && pos_num >= 1)   break;
		if(fin_flag == 1 && pos_num < 1)   break;
		//����ͼƬ����������ͷ�������˳���ǰ״̬
		if(pos_num < pos_sum && find_flag == 1 )
		{
			state_control(2);
			ahead_flag++;
			break;
		}
		
	}
	if(pos_num < pos_sum && ahead_flag == 0)
	{
		
		state_control(2);
	}
	if(putdowm_num == 0)
	{
		putdown();
		arrivel_n_3();
	}
	
	state1ok_flag = 0;
}
void state_2()
{
	//�ȴ�����
	find_flag = 0;
	while(find_flag == 0)   //δ��������
	{
		img_error[0] = -40;
		img_error[1] = 0;
	}
	
	uint16_t st_time = get_time_ms();
	uint16_t dt_time = get_time_ms();
	uint16_t use_time = 0;
	float img_error_p[2] = {0};
	uint8_t flag = 0;
	uint8_t flag1 = 0;
	uint8_t num = 0;
	//��΢����ʶ��
	while(1)
	{
		
//		if(flag1 == 0 && fabs(img_error[0]) <= 20.0f && fabs(img_error[1]) <= 20.0f)
//		{
//			flag1++;
//			state_control(2);
//		}
		//ֻ����������ͣ
		if(img_error[0] <= 5.0f && img_error[0] >= -2.0f && fabs(img_error[1]) <= 5.f &&
			find_flag == 1 && flag == 0 && IMU.acc_x < 40)
		{
			
			
			while(object_type[two_con]/10 == 0)
			{
				//�ȴ�ʶ��
				systick_delay_ms(1);
			}
			//����ȷ��
			find_flag = 0;
			while(find_flag == 0)
			{
				systick_delay_ms(1);
			}
			if((img_error[0] > 5.0f || img_error[0] < -2.0f) || fabs(img_error[1]) > 5.f)  continue;
			work_state = 3;
			//ʶ����ɣ���е��ץȡ 
			if(gpio_get(SW1))
				arrivel_n_1();
			flag = 1;
		}
		//ͣ��
		if(fabs(img_error[0]) <= 0.9f && fabs(img_error[1]) <= 0.9f && find_flag == 1)
		{
			img_error[0] = 0;
			img_error[1] = 0;
			vel_x = 0;
			vel_y = 0;
			find_flag = 0;   //΢������ͷ�ҵ�ͼƬ��־
			break;
		}
		if(find_flag == 0 && get_time_ms() - st_time >= 500)   //δ��������
		{
			st_time = get_time_ms();
			img_error[0] = -40;
			img_error[1] = 0;
		}
		if(find_flag == 1 && IMU.acc_x <= 2)   //��ץȡ��Ŀ�꣬����1s������
		{
			use_time = get_time_ms() - dt_time;
			dt_time = get_time_ms();
			if(use_time >= 3000)
			{
				find_flag = 0;
			}
			
		}
		else if(find_flag == 1 && IMU.acc_x > 2)
		{
			use_time = 0;
			
		}
	lcd_showfloat(0,2,img_error[0],2,2);
	}
	
}
void state_3()
{

//	while(object_type[two_con]/10 == 0)
//	{
//		//�ȴ�ʶ��
//		systick_delay_ms(1);
//	}
//	//ʶ����ɣ���е��ץȡ 
//	if(gpio_get(SW1))
//		arrivel_n_1();
	//���һ��ʰȡ
	pos_num++;
	//�л�״̬
	if(pos_num < pos_sum)
		work_state = 1;
	else
		work_state = 4;
	
//	systick_delay_ms(2000);
//	state_control(1);
//	if(pos_num > 3 && pos_num <= 5) IMU.Yaw += pos_num * 0.25;
//	if(pos_num > 6 && pos_num <= 7) IMU.Yaw += pos_num * 0.20;
//	else if(pos_num > 7)		    IMU.Yaw += pos_num * 0.15;
	IMU.now_x = pos_XY[two_con][0] - 25*cos(IMU.Yaw*pi/180);
	IMU.now_y = pos_XY[two_con][1];
}
void state_4()
{
	uint32_t putdown_time = get_time_ms();
	int16_t targe_x = 0;
	int16_t targe_y = 0;
	//1(B15):����   2(B21):ˮ��  3(B23)����ͨ����
	//�Ž�ͨ����ͼƬ---------------------------
	targe_x = 630;
	targe_y = 530;
	fin_flag = 0;
	IMU.rem_x = (float)(targe_x - IMU.now_x);
	IMU.rem_y = (float)(targe_y - IMU.now_y);
	//��е�۲���
	arrivel_n_2(object_type[two_con]/10);
	uint8_t fanum = 0;
	uint8_t putdowm_num = 0;
	while(1)
	{
		
		//�ȴ�����
		systick_delay_ms(1);
		if(pos_num >= 1 && object_type[two_con]%10 != 0 && fanum == 0)
		{
			fanum++;
			//�ȴ�ʶ��
			systick_delay_ms(1);
			send_msg(IMU.now_x, IMU.now_y, object_type[two_con]);
			//lcdshow_sys(two_con);
			two_con++;
		}
		if(get_time_ms() - putdown_time >= 700 && putdowm_num == 0)
		{
			putdowm_num++;
			putdown();
			systick_delay_ms(500);
		}
		if(fin_flag == 1)   break;
	}
	//��ȫͣ���ٷ�
	if(putdowm_num == 0)
	{
		putdown();
		systick_delay_ms(500);
	}
	gpio_set(B15,0);
	IMU.now_x = targe_x;
	IMU.now_y = targe_y;
	
	//��ˮ��-----------------------------------
	targe_x = 740;
	targe_y = 380;
	IMU.rem_x = (float)(targe_x - IMU.now_x);
	IMU.rem_y = (float)(targe_y - IMU.now_y);
	fin_flag = 0;
	while(fin_flag == 0)
	{
		//�ȴ�����
		systick_delay_ms(1);
	}
	gpio_set(B14,0);
	systick_delay_ms(500);
	turn_angle(-90);
	systick_delay_ms(500);
	//x -> y   ;   y -> -x
	IMU.now_x = targe_y;
	IMU.now_y = -targe_x;
	//�Ŷ���-----------------------------------
	targe_x = 200;   //**λ������
	targe_y = 100;   //**Խ��Խ����
	fin_flag = 0;
	IMU.rem_x = (float)(targe_x - IMU.now_x);
	IMU.rem_y = (float)(targe_y - IMU.now_y);
	uint8_t num = 0;
	img_error[0] = 0;
	while(fin_flag == 0)
	{
		//�ȴ�����
		systick_delay_ms(1);
	}
	gpio_set(B21,0);
	state_control(5);
	fin_flag = 0;

	
	
}

//������
void state_5()
{
	while(1)
	{
		//�ȴ�����
		systick_delay_ms(1);
		IMU.rem_x = -70;
		IMU.rem_y = img_error[1] - 20;
		if(img_error[0] != 0)
		{
			vel_x = 0;
			vel_y = 0;
			IMU.rem_x = 0;
			IMU.rem_y = 0;
			break;
		}
	}
	while(1){
		//��ɱ���
	}
	
}
//ȫ����ͼ	
void move_next(int16_t next_x,int16_t next_y)
{
	int16_t dis_x = 0;
	int16_t dis_y = 0;
	int16_t va = 0;
	//bug:   nowλ��δд��
	
	dis_x = (next_x - 45) - IMU.now_x;
	//dis_y = (next_y - 5*sin(IMU.Yaw*pi/180)) - IMU.now_y;
	dis_y = next_y - IMU.now_y;
	//��Ӧ��
	//������ǿ
	if(pos_num == 0)
	{
		dis_x += 20;
		dis_y += 20;
	}
	else if(dis_x < 0 && dis_y > 0)   //��ֹ�����Զ������
	{
		dis_x += 30;
	}
	else if(dis_x < -70 && dis_y > 0)  //���������
	{
		dis_y += 25;
	}
	else if(dis_x < -70 && dis_y < 0)   //���Ҹ���
	{
		dis_x += 20;
	}
	else if(dis_x > 70 && dis_y < -70)  //���Ҹ���
	{
		dis_y -= 20;
	}
	else if(dis_x > 0 && dis_x < 100 && dis_y > 100)  //�������
	{
		va = dis_x*0.1 + dis_y*0.2;
		va = va>30?30:va;
		dis_x += va;
	}
	else if(dis_x < -70 && abs(dis_y) < 30)   //���˼���
	{
		dis_x += 30;
	}
	else if(dis_x > 100 && abs(dis_y) < 30)    //ǰ������
	{
		dis_x -= 20;
	}

//	if(dis_x < -30)
//	{
//		dis_x -= 10;
//	}
//	if(dis_y < -30)
//	{
//		dis_y -= 10;
//	}
//	if(dis_y > 30)
//	{
//		dis_y += 10;
//	}

	//run_free((float)dis_x,(float)dis_y);
	IMU.rem_x = (float)dis_x;
	IMU.rem_y = (float)dis_y;
	fin_flag = 0;    //�����־��0
}

//������ͼ
bool go_next(int16_t next_x,int16_t next_y)
{
	
	int16_t dis_x = next_x - IMU.now_x;
	int16_t dis_y = next_y - IMU.now_y;
	
//	static int16_t last_x = 0,last_y = 0;
//	int16_t dis_x = next_x - last_x;
//	int16_t dis_y = next_y - last_y;
//	last_x = next_x;
//	last_y = next_y;
	float dis = sqrt((double)(dis_x*dis_x + dis_y*dis_y));
	float yaw_angle = atan2(dis_y,dis_x)/pi*180;
	bool flag = 0;
	if(dis <= 40)  dis = 0;
	else if(dis > 40) dis -= 40;
	if(yaw_angle > -180 && yaw_angle < 0)   yaw_angle = 360 + yaw_angle;
	IMU.exp_dir = yaw_angle;
	systick_delay_ms(500);
	turn_fin = 0;
	while(1)
	{
		if(turn_fin)  
		{
			turn_fin = run_straight(10,dis);
			if(!turn_fin)
			{
				systick_delay_ms(100);
				return 1;
			}
		}
		
	}
	return 0;
}

/*
���ܣ���֤���ݴ���
����ʱ�䣺2022/7/4
�������ݣ��Ľ���ʱ�ش�������ͬ��
*/
void state_control(uint8_t state)
{
//	uint8_t head[2] = {0xed,0x92};
	//�ش�����
	//static uint16_t reply = 0;
	//ͨ��Э��
	while(1){
		if(state != 5)
			uart_putchar(USART_4,state);
		else
			uart_putchar(USART_3,state);
		systick_delay_ms(45);
		if(reply_state == state) 
		{
			work_state = state;
			return;
		}
		//reply++;
		//vofa_data[2] = (float)reply;
	}
}
//while(fin_flag == 0)
//	{
//		//�ȴ�����
//		systick_delay_ms(1);
//		if(fabs(IMU.rem_x) < 200 && fabs(IMU.rem_y) < 100 && num == 0)
//		{
//			//ͨ�������������ͷ��״̬
//			if(num == 0){
//				state_control(5);
//			}
//			if(img_error[0] != 0)
//			{
//				gpio_set(B21,0);
//				
//				IMU.rem_x = 0;
//				IMU.rem_y = 0;
//				vel_x = 0;
//				vel_y = 0;
//				break;
//			}
//			num = 1;
//		}
//		
//	}
//	


////������
//void state_5()
//{
//	gpio_set(B21,0);
//	//һ����
//	fin_flag = 0;
//	img_error[0] = 0;
//	img_error[1] = 0;
//	while(img_error[1] == 0)
//	{
//		//�ȴ�����
//		systick_delay_ms(1);
//		IMU.rem_x = img_error[0];
//		IMU.rem_y = -50;
//	}
//	//������
//	IMU.rem_x = img_error[0] + 30;
//	IMU.rem_y = 0;
//	fin_flag = 0;
//	while(fin_flag == 0)
//	{
//		//�ȴ�����
//		systick_delay_ms(1);
//	}
//	//������
//	IMU.rem_x = 0;
//	IMU.rem_y = -70;
//	fin_flag = 0;
//	while(fin_flag == 0)
//	{
//		//�ȴ�����
//		systick_delay_ms(1);
//	}
//	while(1){
//		//��ɱ���
//	}
//	
//}