#include "pbdata.h"
//************LED��������*************************
void LED_RCC_Configuration(void)
{
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
}
//*************LED��������**************************
void LED_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}
//****************LED��ʼ��**************************
void LED_Init(void)
{
	LED_RCC_Configuration();
	LED_GPIO_Configuration();
}
//***********LED���ƺ���***************************
//type:led������
//n: 0 �صƣ�1 ����
void LED(u8 type,u8 n)
{
	//ʧ��ָʾ��
  if(type==1)
	{
	 if(n==0)             
	 {
		 GPIO_SetBits(GPIOB,GPIO_Pin_12);    	  
	 }
   else             
	 {
	   GPIO_ResetBits(GPIOB,GPIO_Pin_12);    
	 } 
	}
	//�ɻ��͵���ָʾ��
  if(type==2)
	{
	 if(n==0)             
	 {
		 GPIO_SetBits(GPIOB,GPIO_Pin_13);    	  
	 }
   else             
	 {
	   GPIO_ResetBits(GPIOB,GPIO_Pin_13);    
	 }	  
	}
	//�ֱ��͵���ָʾ��
  if(type==3)
	{
	 if(n==0)             
	 {
		 GPIO_SetBits(GPIOB,GPIO_Pin_14);    	  
	 }
   else            
	 {
	   GPIO_ResetBits(GPIOB,GPIO_Pin_14);    
	 }
	}
}

