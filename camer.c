#include "camer.h"
#include "self_headfile.h"
#include "headfile.h"

void get_image()
{
	if(mt9v03x_csi_finish_flag)
	{      
		mt9v03x_csi_finish_flag = 0;
		//ʹ��������ʾ����������ԭʼͼ���С �Լ�������Ҫ��ʾ�Ĵ�С�Զ��������Ż��߷Ŵ���ʾ
		lcd_displayimage032_zoom(mt9v03x_csi_image[0], MT9V03X_CSI_W, MT9V03X_CSI_H, 160, 128);
		uart_putbuff(USART_8,mt9v03x_csi_image[0],160*128);
	}
}