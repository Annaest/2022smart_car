#ifndef __ENC_H__
#define __ENC_H__
#include "stdint.h"

//编码器类
struct ENC_{
	int enc;     //脉冲
	int32_t vel;     //速度
	int32_t last_enc;
	float dis;     //里程
	
};
extern struct ENC_ enc[4];
extern struct ENC_ *e;
void get_enc_vel(struct ENC_ *enc);
void get_enc_dis(struct ENC_ *enc);
void enc_deal();
#endif