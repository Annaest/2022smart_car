#include "self_headfile.h"
#include "headfile.h"


void test_rect()
{
	IMU.exp_dir = 0;
	while(1)
	{
		turn_fin = run_straight(10,100);
		if(!turn_fin)  
		{
			IMU.exp_dir += 90;
			while(1){
				if(turn_fin == 1)  break;
				//等待turn_fin = 1
			}
		}
		
	}
}
void test_polygen()
{
	bool flag = 0;
	int yaw_angle = 30;
	while(1)
	{
		if(flag == 0 && !run_straight(20,80))  
		{
			systick_delay_ms(100);
			flag = 1;
		}
//		if(flag == 1 && !turn_Yaw(yaw_angle))
//		{
//			systick_delay_ms(100);
//			flag = 0;
//			yaw_angle += 30;
//			if(yaw_angle == 360) yaw_angle = 0;	
//		}
	}
}
void test_angle()
{
	IMU.exp_dir = 90.0f;
	int time = get_time_ms();
	while(1)
	{
		//转向部分需要重构
		if(get_time_ms() - time >= 3000){
			time = get_time_ms();
			IMU.exp_dir+=90.0f;
			if(IMU.exp_dir >= 360.0f){
				IMU.exp_dir = 0.0f;
			}
		}
		
	}
}



void test_run()
{
	uint16_t pos_XY[][2] = {{100,100},{100,200},{200,200},{200,300},{300,200},{150,150},
					{100,100},{150,200},{300,100},{400,400},{350,200},{0,0}};
	pos_sum = 11;
	while(1){	
	if(pos_num <= pos_sum){
			go_next(pos_XY[pos_num][0],pos_XY[pos_num][1]);
			pos_num++;
		}	
	}
}

void test_close()
{
		while(1){
	}
	while(1){
		if(!run_straight(20,100)){
			break;
		}
	}

}

void test_serve()
{
	//开关状态变量
	uint8 key1_status = 1;
	uint8 key2_status = 1;
	uint8 key3_status = 1;
	uint8 key4_status = 1;

	int id=0;
	int pos[3]={1500,1500,1500};
	while(1){
       
        //读取当前按键状态
        key1_status = gpio_get(KEY1);
        key2_status = gpio_get(KEY2);
        key3_status = gpio_get(KEY3);
        key4_status = gpio_get(KEY4);
        
       
        
        //标志位置位之后，可以使用标志位执行自己想要做的事件
        if( key1_status)   
        {
           id = 1;
           pos[id]+=3;
			
        }
        
        if(key2_status)   
        {
           id = 1;
           pos[id]-=3;
			
        }
		if(!key3_status)   
        {
           id = 2;
           pos[id]+=3;
			
        }
        
        if(key4_status)   
        {
           id = 2;
          pos[id]-=3;
			
        }
        
      
        
        writepos(id,pos[id], vV11, aA11);
		
		vofa_data[4] = (float)pos[1];
		vofa_data[5] = (float)pos[2];
        systick_delay_ms(10);//延时，按键程序应该保证调用时间不小于10ms
	}
	
	
}
/*
数据传输测试
*/
void test_transfer()
{
	uint16_t num = 0;
	systick_delay_ms(1000);
	while(1)
	{
//		if(gpio_get(KEY1))
//		{
//			state_control(num );
//			num++;
//		}
		state_control(num);
	    num++;
		//systick_delay_ms(5);
		if(num == 256)  break;
	}
	while(1)
	{}
}

void test_pid_debug()
{
	work_state = 1;	
	//指向对应的pid参数
	float *pid_debug = IMU.kpid_in;
	float *pid_debug1 = IMU.kpid_out;
	uint16_t up = 0,down = 8,x_center = 15,left = 20,right = left + 70; //布局
	lcd_clear(GRAY);

	lcd_showstr(x_center,up,"pid debug system");
	lcd_showstr(1,3,"1:");
	lcd_showstr(1,6,"2:");
	uint16_t sw_pid = 1,sw_pid_p = 1,sw_up = 6,sw_down = 1;
	uint8_t key1,key1_p;
	uint8_t key2,key2_p;
	uint8_t key3,key3_p;
	uint8_t key4,key4_p;
	uint8_t temp1;
	uint8_t temp2;
	uint8_t temp3;
	uint8_t temp4;
	uint16_t axis_x = 0,axis_y = 0; //波形坐标轴
	uint16_t bo_left = right +20,bo_down = 6; //波形左，下边界
	while(1)
	{
		//按键检测
		temp1 = gpio_get(KEY2);
		temp2 = gpio_get(KEY1);
		temp3 = gpio_get(KEY3);
		temp4 = gpio_get(SW1);
		//按键消抖
		systick_delay_ms(10);
		if(temp1 == gpio_get(KEY2))  key1 = gpio_get(KEY2);
		if(temp2 == gpio_get(KEY1))  key2 = gpio_get(KEY1);
		if(temp3 == gpio_get(KEY3))  key3 = gpio_get(KEY3);
		if(temp4 == gpio_get(SW1))  key4 = gpio_get(SW1);
		if(key4 == 1)
		{
			lcd_showstr(right,sw_pid+1,"++");
			if(key2 == 0 && key2_p != key2)
			{
				*(pid_debug+sw_pid-1) = *(pid_debug+sw_pid-1) + 0.1;
			}
			if(key1 == 0 && key1_p != key1)
			{
				*(pid_debug+sw_pid-1) = *(pid_debug+sw_pid-1) - 0.1;
			}
		}
		else
		{
			
			if(key2 == 0 && key2 != key2_p)
			{
				sw_pid++;
			}
			if(key1 == 0 && key1 != key1_p)
			{
				sw_pid--;
			}
			if(sw_pid == sw_up + 1)
			{
				sw_pid = sw_down;
			}
			if(sw_pid == sw_down - 1)
			{
				sw_pid = sw_up;
			}
			lcd_showstr(right,sw_pid_p+1,"  ");
			lcd_showstr(right,sw_pid+1,"--");
			sw_pid_p = sw_pid;
			
		}
		
		//显示
		lcd_showstr(left,up+2,"P:");
		lcd_showfloat(left+15,up+2,*pid_debug,2,2);
		lcd_showstr(left,up+3,"I:");
		lcd_showfloat(left+15,up+3,*(pid_debug+1),2,2);
		lcd_showstr(left,up+4,"D:");
		lcd_showfloat(left+15,up+4,*(pid_debug+2),2,2);
		
		lcd_showstr(left,up+5,"P:");
		lcd_showfloat(left+15,up+5,*pid_debug1,2,2);
		lcd_showstr(left,up+6,"I:");
		lcd_showfloat(left+15,up+6,*(pid_debug1+1),2,2);
		lcd_showstr(left,up+7,"D:");
		lcd_showfloat(left+15,up+7,*(pid_debug1+2),2,2);
		//试运行
		if(key3 == 0 && key3_p != key3)
		{
			lcd_showstr(right+20,up+4,"run ");
			systick_delay_ms(2000);
//			IMU.rem_x = 400;
//			IMU.rem_y = 400;
//			fin_flag = 0;
//			while(fin_flag == 0)
//			{
//				//等待跑完
//				systick_delay_ms(1);
//			}
			turn_angle(90);
			
		}
		else
		{
			lcd_showstr(right+20,up+4,"wait");
		}
		key1_p = key1;
		key2_p = key2;
		key3_p = key3;
		key4_p = key4;
		lcd_showfloat(left,up+1,IMU.Yaw,3,2);
		
	}
}


//void test_pid_debug()
//{
//	//指向对应的pid参数
//	float *pid1_debug = pid[0].kpid0;
//	float *pid2_debug = pid[1].kpid0;
//	float *pid3_debug = pid[2].kpid0;
//	float *pid4_debug = pid[3].kpid0;
//	float *pid1_debug1 = pid[0].kpid1;
//	float *pid2_debug1 = pid[1].kpid1;
//	float *pid3_debug1 = pid[2].kpid1;
//	float *pid4_debug1 = pid[3].kpid1;
//	uint16_t up = 0,down = 8,x_center = 15,left = 20,right = left + 70; //布局
//	lcd_clear(GRAY);

//	lcd_showstr(x_center,up,"pid debug system");
//	lcd_showstr(1,3,"1:");
//	lcd_showstr(1,6,"2:");
//	uint16_t sw_pid = 1,sw_pid_p = 1,sw_up = 6,sw_down = 1;
//	uint8_t key1,key1_p;
//	uint8_t key2,key2_p;
//	uint8_t key3,key3_p;
//	uint8_t key4,key4_p;
//	uint8_t temp1;
//	uint8_t temp2;
//	uint8_t temp3;
//	uint8_t temp4;
//	uint16_t axis_x = 0,axis_y = 0; //波形坐标轴
//	uint16_t bo_left = right +20,bo_down = 6; //波形左，下边界
//	while(1)
//	{
//		//按键检测
//		temp1 = gpio_get(KEY2);
//		temp2 = gpio_get(KEY1);
//		temp3 = gpio_get(KEY3);
//		temp4 = gpio_get(SW1);
//		//按键消抖
//		systick_delay_ms(10);
//		if(temp1 == gpio_get(KEY2))  key1 = gpio_get(KEY2);
//		if(temp2 == gpio_get(KEY1))  key2 = gpio_get(KEY1);
//		if(temp3 == gpio_get(KEY3))  key3 = gpio_get(KEY3);
//		if(temp4 == gpio_get(SW1))  key4 = gpio_get(SW1);
//		if(key4 == 1)
//		{
//			lcd_showstr(right,sw_pid+1,"++");
//			if(key2 == 0 && key2_p != key2)
//			{
//				*(pid1_debug+sw_pid-1) = *(pid1_debug+sw_pid-1) + 0.1;
//				*(pid2_debug+sw_pid-1) = *(pid2_debug+sw_pid-1) + 0.1;
//				*(pid3_debug+sw_pid-1) = *(pid3_debug+sw_pid-1) + 0.1;
//				*(pid4_debug+sw_pid-1) = *(pid4_debug+sw_pid-1) + 0.1;
//			}
//			if(key1 == 0 && key1_p != key1)
//			{
//				*(pid1_debug+sw_pid-1) = *(pid1_debug+sw_pid-1) - 0.1;
//				*(pid2_debug+sw_pid-1) = *(pid2_debug+sw_pid-1) - 0.1;
//				*(pid3_debug+sw_pid-1) = *(pid3_debug+sw_pid-1) - 0.1;
//				*(pid4_debug+sw_pid-1) = *(pid4_debug+sw_pid-1) - 0.1;
//			}
//		}
//		else
//		{
//			
//			if(key2 == 0 && key2 != key2_p)
//			{
//				sw_pid++;
//			}
//			if(key1 == 0 && key1 != key1_p)
//			{
//				sw_pid--;
//			}
//			if(sw_pid == sw_up + 1)
//			{
//				sw_pid = sw_down;
//			}
//			if(sw_pid == sw_down - 1)
//			{
//				sw_pid = sw_up;
//			}
//			lcd_showstr(right,sw_pid_p+1,"  ");
//			lcd_showstr(right,sw_pid+1,"--");
//			sw_pid_p = sw_pid;
//			
//		}
//		
//		//显示
//		lcd_showstr(left,up+2,"P:");
//		lcd_showfloat(left+15,up+2,*pid1_debug,2,2);
//		lcd_showstr(left,up+3,"I:");
//		lcd_showfloat(left+15,up+3,*(pid1_debug+1),2,2);
//		lcd_showstr(left,up+4,"D:");
//		lcd_showfloat(left+15,up+4,*(pid1_debug+2),2,2);
//		
//		lcd_showstr(left,up+5,"P:");
//		lcd_showfloat(left+15,up+5,*pid1_debug1,2,2);
//		lcd_showstr(left,up+6,"I:");
//		lcd_showfloat(left+15,up+6,*(pid1_debug1+1),2,2);
//		lcd_showstr(left,up+7,"D:");
//		lcd_showfloat(left+15,up+7,*(pid1_debug1+2),2,2);
//		//试运行
//		if(key3 == 0 && key3_p != key3)
//		{
//			lcd_showstr(right+20,up+4,"run ");
//			systick_delay_ms(2000);
//			IMU.rem_x = 0;
//			IMU.rem_y = 200;
//			fin_flag = 0;
//			while(fin_flag == 0)
//			{
//				//等待跑完
//				systick_delay_ms(1);
//			}
//		}
//		else
//		{
//			lcd_showstr(right+20,up+4,"wait");
//		}
//		key1_p = key1;
//		key2_p = key2;
//		key3_p = key3;
//		key4_p = key4;
//		lcd_showfloat(left,up+1,IMU.Yaw,3,2);
//		
//	}
//}