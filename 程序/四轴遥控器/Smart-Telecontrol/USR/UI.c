#include "pbdata.h"
u8 lock_flag=0;          //����״̬��1��ʾ������0��ʾ����
u8 Plane_status=0;       //�ɻ�״̬��1��ʾ������2��ʾ����
u8 Plane_Bat=0;          //�ɻ��������ɴ��ڻ�ȡ
u8 out_control_flag=0;   //ʧ�ر�־λ
u8 Plane_Bat_low_flag=0; //�ɻ��͵�����־λ
u8 Remote_Bat_low_flag=0;//�ֱ��͵�����־λ
//***************������ʾ����**************************
void photo(u8 n)
{
	//logoҳ��
  if(n==0)
	{
		OLED_Clear();  
	  OLED_ShowPicture(31,0,64,64,logo,1); //��ʾlogo
    OLED_Refresh();	
	}
	//��ҳ
	if(n==1)
	{
		OLED_Clear();
	  OLED_ShowString(0,2,"Plane-Bat:",8,1);
		OLED_ShowString(0,12,"Remote-Bat:",8,1);
		OLED_ShowString(0,22,"Oil:",8,1);
		OLED_ShowString(0,32,"Pitch:",8,1);
		OLED_ShowString(0,42,"Roll:",8,1);
		OLED_ShowString(0,52,"Tips:",8,1);
		OLED_ShowString(30,52,"Please unlock       ",8,1);			
    OLED_Refresh();	
	}
}
//***********������ʾ����*****************************
// n:������־λ
void num_show(u8 n)
{
	BAT_ADC();          //����ֱ�����
	//����״̬��������ʾ
	//�ɻ�������ʾ
	if(Plane_Bat<10)
	{
	 OLED_ShowNum(80,2,Plane_Bat,1,8,1); 
	 OLED_ShowString(86,2,"    ",8,1);
	}
	if((Plane_Bat>=10)&&(Plane_Bat<100))
	{
	 OLED_ShowNum(80,2,Plane_Bat,2,8,1); 
	 OLED_ShowString(93,2,"   ",8,1);
	}
	if(Plane_Bat>=100)
	{
	 OLED_ShowNum(80,2,Plane_Bat,3,8,1); 
	 OLED_ShowString(102,2,"  ",8,1);
	}
	//�ֱ�������ʾ
  if(Bat_n<10)
	{
	 OLED_ShowNum(80,12,Bat_n,1,8,1);
   OLED_ShowString(86,12,"    ",8,1);		
	}
	if((Bat_n>=10)&&(Bat_n<100))
	{
	 OLED_ShowNum(80,12,Bat_n,2,8,1); 
	 OLED_ShowString(93,12,"   ",8,1);
	}
	if(Bat_n>=100)
	{
	 OLED_ShowNum(80,12,Bat_n,3,8,1); 
	 OLED_ShowString(102,12,"  ",8,1);
	}	
	//����״̬
   if(n==0)
	 {
	  OLED_ShowString(80,22,"lock    ",8,1);
		OLED_ShowString(80,32,"lock    ",8,1);
		OLED_ShowString(80,42,"lock    ",8,1); 
	 }	
	//����״̬
   if(n==1)
	 {
		//������ֵ��ʾ
	  if(Oil_n<10)
		{
		 OLED_ShowNum(80,22,Oil_n,1,8,1); 
		 OLED_ShowString(86,22,"      ",8,1);
		}
		if((Oil_n>=10)&&(Oil_n<100))
		{
		 OLED_ShowNum(80,22,Oil_n,2,8,1); 
		 OLED_ShowString(93,22,"     ",8,1);
		}
		if(Oil_n>=100)
		{
		 OLED_ShowNum(80,22,Oil_n,3,8,1); 
		 OLED_ShowString(102,22,"    ",8,1);
		}
		//������ֵ��ʾ
	  if(Pitch_n<10)
		{
		 OLED_ShowNum(80,32,Pitch_n,1,8,1); 
		 OLED_ShowString(86,32,"      ",8,1);
		}
		if((Pitch_n>=10)&&(Pitch_n<100))
		{
		 OLED_ShowNum(80,32,Pitch_n,2,8,1); 
		 OLED_ShowString(93,32,"     ",8,1);
		}
		if(Pitch_n>=100)
		{
		 OLED_ShowNum(80,32,Pitch_n,3,8,1);
     OLED_ShowString(102,32,"    ",8,1);			
		}
		//������ֵ��ʾ
	  if(Roll_n<10)
		{
		 OLED_ShowNum(80,42,Roll_n,1,8,1); 
		 OLED_ShowString(86,42,"      ",8,1);
		}
		if((Roll_n>=10)&&(Roll_n<100))
		{
		 OLED_ShowNum(80,42,Roll_n,2,8,1); 
		 OLED_ShowString(93,42,"     ",8,1);
		}
		if(Roll_n>=100)
		{
		 OLED_ShowNum(80,42,Roll_n,3,8,1); 
		 OLED_ShowString(102,42,"    ",8,1);
		}		
	 }
   OLED_Refresh();	   //ˢ����ֵ	 
}
//*****************�ƹ���ʾ************************
void LED_Show(void)
{
  BAT_ADC();          //����ֱ�����
	if(Plane_Bat<=20)   //�ɻ��͵���
	{
	  Plane_Bat_low_flag=1;
	}
	if(Plane_Bat>20)   //�ɻ���������
	{
	  Plane_Bat_low_flag=0;
		LED(2,0);         //�ص�
	}
	if(Bat_n<=20)       //�ֱ��͵���
	{
	  Remote_Bat_low_flag=1;
	}
	if(Bat_n>20)        //�ֱ���������
	{
	  Remote_Bat_low_flag=0;
		LED(3,0);        //�ص�
	}
}
//**************�ɻ�����*************************
void Plane_Control(void)
{
 if(lock_flag==1)   //����״̬
 {
		if(auto_land_flag==0)
		{
		 Oil_ADC();   //����״̬���
		}
    if(fine_tuning_flag==0)	 
		{		 
		 Pitch_ADC();  //����״̬���    
		 Roll_ADC();   //����״̬���
		}		
		fine_tuning(); //΢����̬���
		yaw_status(); 		
		msg_send[0]=0x02;
		msg_send[1]=Oil_n;
		msg_send[2]=Pitch_n;
		msg_send[3]=Roll_n;
		msg_send[4]=yaw_flag;		
		MSG_Send(msg_send);
 }	
}
//*******��������************************
void UI(void)
{ 	
  num_show(lock_flag);
	LED_Show();

}



