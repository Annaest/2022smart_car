#ifndef __ENC_H__
#define __ENC_H__
#include "stdint.h"

//��������
struct ENC_{
	int enc;     //����
	int32_t vel;     //�ٶ�
	int32_t last_enc;
	float dis;     //���
	
};
extern struct ENC_ enc[4];
extern struct ENC_ *e;
void get_enc_vel(struct ENC_ *enc);
void get_enc_dis(struct ENC_ *enc);
void enc_deal();
#endif