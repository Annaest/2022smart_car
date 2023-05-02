#ifndef __PREDICT_H__
#define __PREDICT_H__
#include "stdint.h"

extern int16_t pre_map[6][12][2];
extern uint8_t pre_type[6][12];
extern uint8_t pre_num;
int8_t map_right();
#endif