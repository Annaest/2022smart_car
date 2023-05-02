#include "headfile.h"
#include "self_headfile.h"
void set_enc()
{
	
}
void get_enc_vel(struct ENC_ *enc)
{
	float k = 440.00; 
	enc->vel = (enc->enc - enc->last_enc)/(pit0_time*1.0) * k;
	
	//enc->last_enc = enc->enc;
}

void get_enc_dis(struct ENC_ *enc)
{
	enc->dis += enc->enc*0.00828;            //M车轮直径6.3cm，齿轮数70，编码器齿轮数30；19.782/(1024*2.333)
	//printf("enc->dis:%f\r\n",enc->dis);
}

void enc_deal()
{
	int16_t enc_value[4] = {0,0,0,0};
	static int16_t last_enc_value[4] = {0,0,0,0};
	float k = 0.7;             //滤波系数
	//读取编码器计数值
	enc_value[0] = -qtimer_quad_get(QTIMER_1,QTIMER1_TIMER0_C0 );
	enc_value[1] = +qtimer_quad_get(QTIMER_1,QTIMER1_TIMER2_C2 );
	enc_value[2] = -qtimer_quad_get(QTIMER_2,QTIMER2_TIMER0_C3 );
	enc_value[3] = +qtimer_quad_get(QTIMER_3,QTIMER3_TIMER2_B18);

	qtimer_quad_clear(QTIMER_1, QTIMER1_TIMER0_C0);
	qtimer_quad_clear(QTIMER_1,QTIMER1_TIMER2_C2);
	qtimer_quad_clear(QTIMER_2,QTIMER2_TIMER0_C3);
	qtimer_quad_clear(QTIMER_3,QTIMER3_TIMER2_B18);
 
	//编码器数值存入结构体
	enc[0].enc = (int16_t)(enc_value[0]*k + (1-k)*last_enc_value[0]);
	enc[1].enc = (int16_t)(enc_value[1]*k + (1-k)*last_enc_value[1]);
	enc[2].enc = (int16_t)(enc_value[2]*k + (1-k)*last_enc_value[2]);
	enc[3].enc = (int16_t)(enc_value[3]*k + (1-k)*last_enc_value[3]);

	

	
	//printf("%d %d %d %d\r\n",enc[0].enc,enc[1].enc,enc[2].enc,enc[3].enc);
	last_enc_value[0] = enc_value[0];
	last_enc_value[1] = enc_value[1];
	last_enc_value[2] = enc_value[2];
	last_enc_value[3] = enc_value[3];
	
	//获得每个轮子的采用速度
	get_enc_vel(e+0);
	get_enc_vel(e+1);
	get_enc_vel(e+2);
	get_enc_vel(e+3);
	//printf("%d %d %d %d\r\n",enc[0].vel,enc[1].vel,enc[2].vel,enc[3].vel);
	//里程计
	get_enc_dis(e+0);
	get_enc_dis(e+1);
	get_enc_dis(e+2);
	get_enc_dis(e+3);
	
}