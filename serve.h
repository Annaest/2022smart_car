#ifndef __SERVE_H__
#define __SERVE_H__
#define serve2_pos1 2000    //刚好不被遮挡的位置
#define serve2_pos2 1250    //识别的位置
#define serve1_pos1 2100    //便于抓取
//#define serve2_pos1         //不被看见
extern int vV11;
extern int aA11;
//机械臂控制
void writepos(int i,int p,int v,int a);
void arrivel_n_1(void);
void putdown(void);
void arrivel_n_2(int n);
void arrivel_n_3();
#endif