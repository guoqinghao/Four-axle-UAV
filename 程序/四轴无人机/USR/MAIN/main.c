#include "pbdata.h"
int main(void)
{	
 uboot();
 OS_Init(); 
	Moto_Ctrl(0,1); //���õ�1�ŵ��
	Moto_Ctrl(0,2); //���õ�2�ŵ��
	Moto_Ctrl(0,3); //���õ�3�ŵ��
	Moto_Ctrl(0,4); //���õ�4�ŵ��
	Plane_status=0;
  Oil_n=0;	
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);	
 while(1)
 {
	 BAT_ADC();  //����ʵʱ���
	 MSG_Recieve();    
//	 UI(); 		
	}
}


