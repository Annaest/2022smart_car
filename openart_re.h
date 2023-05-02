#ifndef __OPENART_RE_H__
#define __OPENART_RE_H__
#include "stdint.h"
#include "stdio.h"
#define max_pot 25   //������

void openart_xy(uint8_t data);
void openart_error(uint8_t data);
void openart1_error(uint8_t data);
void openart1_receive(uint8_t data);
void openart2_receive(uint8_t data);
void openart_type(uint8_t data);
void approach_img();   //�ӽ�ͼƬ

void state_send();
//��������
extern int16_t pos_XY[][2];
extern float img_error[2];
extern float img_1_error[2];
extern uint8_t error_flag;
extern uint8_t error1_flag;
extern uint8_t object_type[max_pot];
extern uint8_t reply_state;
extern uint8_t find_flag;   //�Ƿ��ܿ����ı�־��ÿ��ʶ����һ����0
extern float xkpid[3];
extern float ykpid[3];
extern uint8_t two_con;

#endif