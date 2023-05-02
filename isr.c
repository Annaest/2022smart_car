#include "headfile.h"
#include "self_headfile.h"
void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler();     //����SDK�Դ����жϺ��� ���������������������õĻص�����
    __DSB();                    //����ͬ������
}

void PIT_IRQHandler(void)
{
//	uint32_t time = 0;
	//�����Զ�ȡ��������ֵʹ��
	//pid����
    if(PIT_FLAG_GET(PIT_CH0))
    {
//		time = get_time_us();
		enc_deal();
		xy_vel();
		//ȫ���ƶ�
		if(work_state == 1 || work_state == 4 || work_state == 5)
		{
			run_free();
		}
		//΢���ӽ�ͼƬ
		if(work_state == 2 || work_state == 3)
		{
			approach_img();
		}
		move_control();
//		vofa_data[6] = (float)(get_time_us() - time);
		PIT_FLAG_CLEAR(PIT_CH0);
    }
    //�����Ƕ�ȡ
    if(PIT_FLAG_GET(PIT_CH1))
    {
		
		cal_rollAngle();
        PIT_FLAG_CLEAR(PIT_CH1);
    }
    //�������
    if(PIT_FLAG_GET(PIT_CH2))
    {
		//rm_revolve();
        PIT_FLAG_CLEAR(PIT_CH2);
    }
    
    if(PIT_FLAG_GET(PIT_CH3))
    {
		//pos_now();
		//state_send(); //״̬����
		//����pidʱ���test_close()��
		
		vofa_data[0] = (float)IMU.Yaw;
		vofa_data[1] = (float)work_state;
		vofa_data[2] = IMU.acc_x;
		vofa_data[3] = IMU.acc_y;
		vofa_data[4] = pos_num;
		vofa_print(vofa_data,7);
//		lcd_showstr(0,0,"state:");
//		lcd_showint8(50,0,work_state);
//		lcd_showfloat(100,6,img_error[0],2,2);
//		lcd_showint8(100,5,find_flag);
        PIT_FLAG_CLEAR(PIT_CH3);
    } 

    __DSB();
}


void GPIO2_Combined_16_31_IRQHandler(void)
{

    CLEAR_GPIO_FLAG(C16);//����жϱ�־λ
    
}



void GPIO2_Combined_0_15_IRQHandler(void)
{
    if(GET_GPIO_FLAG(MT9V03X_VSYNC_PIN))
    {
        //���������־λ����־λ��mt9v03x_vsync�����ڲ������
        if(CAMERA_GRAYSCALE == flexio_camera_type)mt9v03x_vsync();
    }
    if(GET_GPIO_FLAG(SCC8660_VSYNC_PIN))
    {
        //���������־λ����־λ��scc8660_vsync�����ڲ������
        if(CAMERA_COLOR == flexio_camera_type)scc8660_vsync();
    }
}

uint8_t ch = '0';
//�жϷ�����
void LPUART8_IRQHandler(void) 
	{ 
		
		if((LPUART8->STAT)&kLPUART_RxDataRegFullFlag) //�����ж� 
			{ 
				uint8_t receive_data; 
				receive_data = LPUART8->DATA;
				//����ֹͣ
				if(receive_data == 's'){
					event_stop();
				}
				
				//Զ�̵���pid
				static uint8_t flag = 0;
				if(receive_data == 'p' || receive_data == 'i' || receive_data == 'd' || flag == 1) 
				{
					adj_pid(p+0,receive_data);
					flag = 1;
					if(receive_data == 'e')  flag = 0;
				}
				if(flag == 0)  uart_car(receive_data);
				
			
			}
			
			__DSB(); //����ͬ������ָ�� 
	}

void LPUART1_IRQHandler(void) 
{
	
	if((LPUART1->STAT)&kLPUART_RxDataRegFullFlag) //�����ж� 
	{ 
		uint8_t receive_data; 
		receive_data = LPUART1->DATA;
		
		
		//adj_pid(p,receive_data);
		
	}
	__DSB(); //����ͬ������ָ�� 
	
}
void LPUART2_IRQHandler(void) 
{
	if((LPUART2->STAT)&kLPUART_RxDataRegFullFlag) //�����ж� 
		{ 
			uint8_t receive_data; 
			receive_data = LPUART2->DATA;
			
			
		}
		__DSB(); //����ͬ������ָ�� 
	
}
void LPUART3_IRQHandler(void) 
{
	if((LPUART3->STAT)&kLPUART_RxDataRegFullFlag) //�����ж� 
	{ 
		uint8_t receive_data = 0; 
		receive_data = LPUART3->DATA;
		//״̬ͨ��
		if(work_state == 4)
			openart2_receive(receive_data);
		if(work_state == 2 || work_state == 5 || (state1ok_flag == 1 && fabs(IMU.rem_x) <= 30 && fabs(IMU.rem_y) <= 20))
		{
			openart_error(receive_data);
		
			//adj_pid(p,receive_data);
		}
		
	}
	
	__DSB(); //����ͬ������ָ�� 
}
//�жϷ�����
void LPUART4_IRQHandler(void) 
{ 
	if((LPUART4->STAT)&kLPUART_RxDataRegFullFlag) //�����ж� 
		{ 
			uint8_t receive_data = 0; 
			receive_data = LPUART4->DATA;
			
			//״̬ͨ��
			openart2_receive(receive_data);
			
			//��������
			if(work_state == 0)
				openart_xy(receive_data);
			//����Ŀ����Ϣ
			else if(work_state == 3 || work_state == 1 || work_state == 4 || work_state == 2)
				openart_type(receive_data);
		}
		__DSB(); //����ͬ������ָ�� 
}
/*
�жϺ������ƣ��������ö�Ӧ���ܵ��жϺ���
Sample usage:��ǰ���������ڶ�ʱ���ж�
void PIT_IRQHandler(void)
{
    //��������־λ
    __DSB();
}
�ǵý����жϺ������־λ
CTI0_ERROR_IRQHandler
CTI1_ERROR_IRQHandler
CORE_IRQHandler
FLEXRAM_IRQHandler
KPP_IRQHandler
TSC_DIG_IRQHandler
GPR_IRQ_IRQHandler
LCDIF_IRQHandler
CSI_IRQHandler
PXP_IRQHandler
WDOG2_IRQHandler
SNVS_HP_WRAPPER_IRQHandler
SNVS_HP_WRAPPER_TZ_IRQHandler
SNVS_LP_WRAPPER_IRQHandler
CSU_IRQHandler
DCP_IRQHandler
DCP_VMI_IRQHandler
Reserved68_IRQHandler
TRNG_IRQHandler
SJC_IRQHandler
BEE_IRQHandler
PMU_EVENT_IRQHandler
Reserved78_IRQHandler
TEMP_LOW_HIGH_IRQHandler
TEMP_PANIC_IRQHandler
USB_PHY1_IRQHandler
USB_PHY2_IRQHandler
ADC1_IRQHandler
ADC2_IRQHandler
DCDC_IRQHandler
Reserved86_IRQHandler
Reserved87_IRQHandler
GPIO1_INT0_IRQHandler
GPIO1_INT1_IRQHandler
GPIO1_INT2_IRQHandler
GPIO1_INT3_IRQHandler
GPIO1_INT4_IRQHandler
GPIO1_INT5_IRQHandler
GPIO1_INT6_IRQHandler
GPIO1_INT7_IRQHandler
GPIO1_Combined_0_15_IRQHandler
GPIO1_Combined_16_31_IRQHandler
GPIO2_Combined_0_15_IRQHandler
GPIO2_Combined_16_31_IRQHandler
GPIO3_Combined_0_15_IRQHandler
GPIO3_Combined_16_31_IRQHandler
GPIO4_Combined_0_15_IRQHandler
GPIO4_Combined_16_31_IRQHandler
GPIO5_Combined_0_15_IRQHandler
GPIO5_Combined_16_31_IRQHandler
WDOG1_IRQHandler
RTWDOG_IRQHandler
EWM_IRQHandler
CCM_1_IRQHandler
CCM_2_IRQHandler
GPC_IRQHandler
SRC_IRQHandler
Reserved115_IRQHandler
GPT1_IRQHandler
GPT2_IRQHandler
PWM1_0_IRQHandler
PWM1_1_IRQHandler
PWM1_2_IRQHandler
PWM1_3_IRQHandler
PWM1_FAULT_IRQHandler
SEMC_IRQHandler
USB_OTG2_IRQHandler
USB_OTG1_IRQHandler
XBAR1_IRQ_0_1_IRQHandler
XBAR1_IRQ_2_3_IRQHandler
ADC_ETC_IRQ0_IRQHandler
ADC_ETC_IRQ1_IRQHandler
ADC_ETC_IRQ2_IRQHandler
ADC_ETC_ERROR_IRQ_IRQHandler
PIT_IRQHandler
ACMP1_IRQHandler
ACMP2_IRQHandler
ACMP3_IRQHandler
ACMP4_IRQHandler
Reserved143_IRQHandler
Reserved144_IRQHandler
ENC1_IRQHandler
ENC2_IRQHandler
ENC3_IRQHandler
ENC4_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
TMR4_IRQHandler
PWM2_0_IRQHandler
PWM2_1_IRQHandler
PWM2_2_IRQHandler
PWM2_3_IRQHandler
PWM2_FAULT_IRQHandler
PWM3_0_IRQHandler
PWM3_1_IRQHandler
PWM3_2_IRQHandler
PWM3_3_IRQHandler
PWM3_FAULT_IRQHandler
PWM4_0_IRQHandler
PWM4_1_IRQHandler
PWM4_2_IRQHandler
PWM4_3_IRQHandler
PWM4_FAULT_IRQHandler
Reserved171_IRQHandler
GPIO6_7_8_9_IRQHandler*/



