#ifndef __ROBOMASTER_H__
#define __ROBOMASTER_H__
#include "stdint.h"
typedef struct {
	int8_t vel;
	float dir;
	float rev;
	
}robomaster;
extern robomaster rm;

void rm_revolve();
void  rem_main();
#endif