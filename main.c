#include "SEEKFREE_FONT.h"
#include "headfile.h"
#include "self_headfile.h"
#include "time.h"
#include "math.h"
//#include "path_solve.h"
#include "ga_solve.h"
/* 中断优先级组 */
#define NVIC_Group0   0x07
#define NVIC_Group1   0x06
#define NVIC_Group2   0x05
#define NVIC_Group3   0x04
#define NVIC_Group4   0x03
#define pi 3.142
#define normal -1    //是否需要看图

int32_t vel_x = 0;               //主速度
int32_t vel_y = 0;
int32_t vel_w = 0;
//定义对象
struct MOTOR_ motor[4],*motor_p = motor;       //电机
struct PID_ pid[4],*p = pid;             //Pid
struct ENC_ enc[4],*e = enc;             //编码器;
KalmanStructTypedef kalmanFilter[6],*kf = kalmanFilter; //6轴卡尔曼滤波
//坐标数据

int16_t pos_XY[max_pot][2] = {0};
//int16_t pos_XY[max_pot][2] = {{450,350},{0,0},{100,300},{250,300},{350,400},{400,250},{300,100},{0,0}};
uint8_t pos_sum = 0;       

//坐标总数
uint8_t pos_num = 0;           //第几个坐标

bool go_next(int16_t next_x,int16_t next_y);   //跑图
void move_next(int16_t next_x,int16_t next_y);
float img_error[2] = {0,0};                    //后摄像头坐标点离图片距离 

uint8_t error_flag = 0;                             //error接收flag
int state1ok_flag = 0;

void approach_img();                            //前摄接近图片
void state_control(uint8_t state);              //状态控制
float vofa_data[7] = {0,0,0,0,0,0,0};     
void state_0();
void state_1();
void state_2();
void state_3();
void state_4();
void state_5();
//vofa波形调试
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
			//开始跑图
			case 1:
				state_1();
				break;
			//到一个目标点微调
			case 2:
				state_2();
				break;
			//等待图片识别结果
			case 3:
				state_3();
				break;
			//完赛
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
	//等待坐标
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
		//等待发送坐标
		while(pos_XY[0][0] == 0 || pos_XY[0][1] == 0)
		{
			systick_delay_ms(1);
		}
		
		//接收完成
		//计算坐标个数
		for(int i=0;pos_XY[i][0]!=0 || pos_XY[i][1]!=0;i++){
			pos_sum++;
			
		}
		int8_t map_num = -1;
//		map_num = map_right();
		lcd_showuint8(1,1,map_num);
		//地图命中选定
		if(map_num != -1)    //选中地图
		{
			for(uint8_t i=0;i<12;i++)
			{
				pos_XY[i][0] = pre_map[map_num][i][0];
				pos_XY[i][1] = pre_map[map_num][i][1];
				object_type[i] = pre_type[map_num][i];
				
			}
			pre_flag = 1;
		}
		else     //未选中地图，正常遗传计算路径
		{
//			double d = mygenus(pos_sum);
		}
	}
	
//	while(gpio_get(KEY2) == 1)
//	{
//		systick_delay_ms(10);
//	}
//	systick_delay_ms(1000);
	//初始位置
	IMU.now_x = 30;
	IMU.now_y = -30;
//	state_control(1);
	work_state = 1;
}

void state_1()
{
	uint32_t putdown_time = 0;
//	//出库规避
//	static uint8_t chuku = 0;
//	if(chuku == 0)
//	{
//		chuku++;
//		IMU.rem_y = 30;
//		//IMU.now_y = 30;
//		fin_flag = 0;
//		while(fin_flag == 0)
//		{
//			//等待跑完
//			systick_delay_ms(1);
//		}
//	}
	//go_next(pos_XY[pos_num][0],pos_XY[pos_num][1]);
	//解决出库提前置1的bug
	find_flag = 0;
	move_next(pos_XY[pos_num][0],pos_XY[pos_num][1]);
	
	
	//机械臂操作
	if(pos_num >= 1)
	{
//		lcd_showint8(0,0,object_type[two_con]/10);
//		lcd_showint8(0,1,two_con);
		arrivel_n_2(object_type[two_con]/10);
		
	}
	
	putdown_time = get_time_ms();
	uint8_t fanum = 0;       //发的次数
	uint8_t putdowm_num = 0; //提前放的标志
	uint8_t ahead_flag = 0; //提前看到的标志
	while(1)
	{
		
		//等待跑完
		systick_delay_ms(1);
		if(pos_num >= 1 && object_type[two_con]%10 != 0 && fanum == 0)
		{
			fanum++;
			//等待识别
			systick_delay_ms(1);
			send_msg(IMU.now_x, IMU.now_y, object_type[two_con]);
			//lcdshow_sys(two_con);
			two_con++;
		}
		if(get_time_ms() - putdown_time >= 700 && putdowm_num == 0)
		{
			//可以打开微调摄像头
			state1ok_flag = 1;   //开始接收img_error
			putdowm_num++;
			putdown();
			arrivel_n_3();
		}
		if(fin_flag == 1 && fanum == 1 && pos_num >= 1)   break;
		if(fin_flag == 1 && pos_num < 1)   break;
		//看到图片，交由摄像头导航，退出当前状态
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
	//等待坐标
	find_flag = 0;
	while(find_flag == 0)   //未看到后退
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
	//边微调边识别
	while(1)
	{
		
//		if(flag1 == 0 && fabs(img_error[0]) <= 20.0f && fabs(img_error[1]) <= 20.0f)
//		{
//			flag1++;
//			state_control(2);
//		}
		//只依靠编码器停
		if(img_error[0] <= 5.0f && img_error[0] >= -2.0f && fabs(img_error[1]) <= 5.f &&
			find_flag == 1 && flag == 0 && IMU.acc_x < 40)
		{
			
			
			while(object_type[two_con]/10 == 0)
			{
				//等待识别
				systick_delay_ms(1);
			}
			//二次确认
			find_flag = 0;
			while(find_flag == 0)
			{
				systick_delay_ms(1);
			}
			if((img_error[0] > 5.0f || img_error[0] < -2.0f) || fabs(img_error[1]) > 5.f)  continue;
			work_state = 3;
			//识别完成，机械臂抓取 
			if(gpio_get(SW1))
				arrivel_n_1();
			flag = 1;
		}
		//停稳
		if(fabs(img_error[0]) <= 0.9f && fabs(img_error[1]) <= 0.9f && find_flag == 1)
		{
			img_error[0] = 0;
			img_error[1] = 0;
			vel_x = 0;
			vel_y = 0;
			find_flag = 0;   //微调摄像头找到图片标志
			break;
		}
		if(find_flag == 0 && get_time_ms() - st_time >= 500)   //未看到后退
		{
			st_time = get_time_ms();
			img_error[0] = -40;
			img_error[1] = 0;
		}
		if(find_flag == 1 && IMU.acc_x <= 2)   //已抓取过目标，超过1s车身不动
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
//		//等待识别
//		systick_delay_ms(1);
//	}
//	//识别完成，机械臂抓取 
//	if(gpio_get(SW1))
//		arrivel_n_1();
	//完成一次拾取
	pos_num++;
	//切换状态
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
	//1(B15):动物   2(B21):水果  3(B23)：交通工具
	//放交通工具图片---------------------------
	targe_x = 630;
	targe_y = 530;
	fin_flag = 0;
	IMU.rem_x = (float)(targe_x - IMU.now_x);
	IMU.rem_y = (float)(targe_y - IMU.now_y);
	//机械臂操作
	arrivel_n_2(object_type[two_con]/10);
	uint8_t fanum = 0;
	uint8_t putdowm_num = 0;
	while(1)
	{
		
		//等待跑完
		systick_delay_ms(1);
		if(pos_num >= 1 && object_type[two_con]%10 != 0 && fanum == 0)
		{
			fanum++;
			//等待识别
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
	//完全停下再放
	if(putdowm_num == 0)
	{
		putdown();
		systick_delay_ms(500);
	}
	gpio_set(B15,0);
	IMU.now_x = targe_x;
	IMU.now_y = targe_y;
	
	//放水果-----------------------------------
	targe_x = 740;
	targe_y = 380;
	IMU.rem_x = (float)(targe_x - IMU.now_x);
	IMU.rem_y = (float)(targe_y - IMU.now_y);
	fin_flag = 0;
	while(fin_flag == 0)
	{
		//等待跑完
		systick_delay_ms(1);
	}
	gpio_set(B14,0);
	systick_delay_ms(500);
	turn_angle(-90);
	systick_delay_ms(500);
	//x -> y   ;   y -> -x
	IMU.now_x = targe_y;
	IMU.now_y = -targe_x;
	//放动物-----------------------------------
	targe_x = 200;   //**位置正常
	targe_y = 100;   //**越大越往外
	fin_flag = 0;
	IMU.rem_x = (float)(targe_x - IMU.now_x);
	IMU.rem_y = (float)(targe_y - IMU.now_y);
	uint8_t num = 0;
	img_error[0] = 0;
	while(fin_flag == 0)
	{
		//等待跑完
		systick_delay_ms(1);
	}
	gpio_set(B21,0);
	state_control(5);
	fin_flag = 0;

	
	
}

//入库程序
void state_5()
{
	while(1)
	{
		//等待跑完
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
		//完成比赛
	}
	
}
//全向跑图	
void move_next(int16_t next_x,int16_t next_y)
{
	int16_t dis_x = 0;
	int16_t dis_y = 0;
	int16_t va = 0;
	//bug:   now位置未写入
	
	dis_x = (next_x - 45) - IMU.now_x;
	//dis_y = (next_y - 5*sin(IMU.Yaw*pi/180)) - IMU.now_y;
	dis_y = next_y - IMU.now_y;
	//适应性
	//出库增强
	if(pos_num == 0)
	{
		dis_x += 20;
		dis_y += 20;
	}
	else if(dis_x < 0 && dis_y > 0)   //防止侧移自动向后退
	{
		dis_x += 30;
	}
	else if(dis_x < -70 && dis_y > 0)  //下左更左下
	{
		dis_y += 25;
	}
	else if(dis_x < -70 && dis_y < 0)   //下右更上
	{
		dis_x += 20;
	}
	else if(dis_x > 70 && dis_y < -70)  //上右更右
	{
		dis_y -= 20;
	}
	else if(dis_x > 0 && dis_x < 100 && dis_y > 100)  //上左更上
	{
		va = dis_x*0.1 + dis_y*0.2;
		va = va>30?30:va;
		dis_x += va;
	}
	else if(dis_x < -70 && abs(dis_y) < 30)   //后退减速
	{
		dis_x += 30;
	}
	else if(dis_x > 100 && abs(dis_y) < 30)    //前进减速
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
	fin_flag = 0;    //跑完标志置0
}

//单向跑图
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
功能：保证数据传输
更新时间：2022/7/4
更新内容：改进超时重传，数据同步
*/
void state_control(uint8_t state)
{
//	uint8_t head[2] = {0xed,0x92};
	//重传次数
	//static uint16_t reply = 0;
	//通信协议
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
//		//等待跑完
//		systick_delay_ms(1);
//		if(fabs(IMU.rem_x) < 200 && fabs(IMU.rem_y) < 100 && num == 0)
//		{
//			//通信让上面的摄像头发状态
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


////入库程序
//void state_5()
//{
//	gpio_set(B21,0);
//	//一步调
//	fin_flag = 0;
//	img_error[0] = 0;
//	img_error[1] = 0;
//	while(img_error[1] == 0)
//	{
//		//等待跑完
//		systick_delay_ms(1);
//		IMU.rem_x = img_error[0];
//		IMU.rem_y = -50;
//	}
//	//二步调
//	IMU.rem_x = img_error[0] + 30;
//	IMU.rem_y = 0;
//	fin_flag = 0;
//	while(fin_flag == 0)
//	{
//		//等待跑完
//		systick_delay_ms(1);
//	}
//	//三步调
//	IMU.rem_x = 0;
//	IMU.rem_y = -70;
//	fin_flag = 0;
//	while(fin_flag == 0)
//	{
//		//等待跑完
//		systick_delay_ms(1);
//	}
//	while(1){
//		//完成比赛
//	}
//	
//}