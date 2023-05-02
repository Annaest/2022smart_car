#include "self_headfile.h"
robomaster rm;
void rm_revolve()
{
	IMU.exp_dir += rm.rev;
}

void rem_main()
{
	rm.rev = 0.1;
	rm.vel = 5;
	rm.dir = 0;
	while(1)
	{
//		vel_x = rm.vel*cos(IMU.Yaw/180);
//		vel_y = rm.vel*sin(IMU.Yaw/180);
	
	}
}