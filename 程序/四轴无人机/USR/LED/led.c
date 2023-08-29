#include "pbdata.h"
//*****************LEDʱ�ӳ�ʼ��********************
void LED_RCC_Init(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //ʹ��PA�˿�ʱ��
}
//*****************LED���ų�ʼ��********************
//ע�⣺��ɫ����PA15��JTAG_IO_Init�����ڳ�ʼ������
void LED_GPIO_Init(void)
{
 GPIO_InitTypeDef  GPIO_InitStructure;
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_8;//LED�˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);					       //�����趨������ʼ
 GPIO_SetBits(GPIOA,GPIO_Pin_5);						           //PA.5 �����
 GPIO_SetBits(GPIOA,GPIO_Pin_8);						           //PA.8 �����
}
//*****************LED��ʼ��************************
void LED_Init(void)
{
 LED_RCC_Init();
 LED_GPIO_Init();
}
 
