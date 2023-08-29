#include "pbdata.h"
u8 mpu6050_data_flag;    								  //�Ƿ�������ȡ����
float pitch,roll,yaw;     								//ŷ����
short aacx,aacy,aacz;	   								  //���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz; 								  //������ԭʼ����
short mpu6050_temp;		   								  //�¶�	
u8 Oil_n=0;   														//������ֵ
u8 Pitch_n=50; 														//������ֵ
u8 Roll_n=50; 														 //������ֵ
u8 Yaw_n=0;   														//������ֵ
int PWM_Out1=0; 													//�������õ������PWM
int PWM_Out2=0;													  //�������õ������PWM
int PWM_Out3=0;													  //�������õ������PWM
int PWM_Out4=0; 												  //�������õ������PWM
int single_pitch_balance_out=0;  					//����ƽ�⻷���������
int single_roll_balance_out=0;   					//����ƽ�⻷���������
int single_yaw_balance_out=0;             //ƫ��ƽ�⻵���������
int double_pitch_balance_angle_out=0;     //����ƽ���⻷�����
int double_roll_balance_angle_out=0;      //����ƽ���⻷�����
int double_yaw_balance_angle_out=0;       //ƫ��ƽ���⻷�����  
int double_pitch_balance_out=0;           //����ƽ�⻷���������
int double_roll_balance_out=0;            //����ƽ�⻷���������
int double_yaw_balance_out=0;             //ƫ��ƽ�⻷���������
int yaw_out=0;           									//���򻷵��������
float pwm_adc=0.0f;                       //���ڲ�����ѹ������PWM���������Ϊ����ĵ�ѹֱ����أ���һֱ����
float double_PID_Pitch_err;               //�����ڻ����ֵ  
float double_PID_Pitch_last_err;          //�����ڻ��ϴ����ֵ
float double_PID_Pitch_Angle_Integral;    //�����⻷����
float double_PID_Pitch_Gyro_Integral;     //�����ڻ�����
float double_PID_Roll_err;                //�����ڻ����ֵ
float double_PID_Roll_last_err;           //�����ڻ��ϴ����ֵ
float double_PID_Roll_Angle_Integral;     //�����⻷����
float double_PID_Roll_Gyro_Integral;      //�����ڻ�����
float double_PID_Yaw_err;                 //�����ڻ����ֵ
float double_PID_Yaw_last_err;            //�����ڻ��ϴ����ֵ
float double_PID_Yaw_Angle_Integral;      //�����⻷����
float double_PID_Yaw_Gyro_Integral;       //�����ڻ�����
//****************�ɻ��Ļ�е���***************
//��е�����˿��Խ�����װ��������У׼���ڵ������Ҷ����װ�Ƕȵȸ���ԭ������ķɻ��޷�ƽ�������
float single_Fly_Pitch_Zero=10.0f;   
float single_Fly_Roll_Zero=3.0f;  
float single_Fly_Yaw_Zero=0.0f;
//****************�ɻ��Ļ�е���***************
//��е�����˿��Խ�����װ��������У׼���ڵ������Ҷ����װ�Ƕȵȸ���ԭ������ķɻ��޷�ƽ�������
//float double_Fly_Pitch_Zero=5.5f;   //ƽ���ʱ�� �ɻ����˼Ӵ󣬷ɻ�ǰ����С
//float double_Fly_Roll_Zero=3.2f;    //ƽ���ʱ��  �ɻ����ҼӴ󣬷ɻ������С
//float double_Fly_Yaw_Zero=0.0f; 

float double_Fly_Pitch_Zero=5.0f;   //ƽ���ʱ�� �ɻ����˼Ӵ󣬷ɻ�ǰ����С
float double_Fly_Roll_Zero=3.0f;    //ƽ���ʱ��  �ɻ����ҼӴ󣬷ɻ������С
float double_Fly_Yaw_Zero=0.0f; 

//*************����PID��������*****************
//����ƽ�⻷
float single_pitch_balance_Kp=-40.0f; 
float single_pitch_balance_Kd=0.8f; 
//����ƽ�⻷
float single_roll_balance_Kp=-40.0f; 
float single_roll_balance_Kd=0.8f; 
//����ƽ�⻷
float single_yaw_balance_Kp=10.0f; 
float single_yaw_balance_Kd=-0.2f; 
//*************����PID��������*****************
//����ƽ�⻷�⻷
float double_pitch_balance_Angle_Kp=105.0f; 
float double_pitch_balance_Angle_Ki=0.0;
float double_pitch_balance_Angle_Kd=-0.8f;  
//����ƽ�⻷�⻷
float double_roll_balance_Angle_Kp=50.0f;
float double_roll_balance_Angle_Ki=0.0f;
float double_roll_balance_Angle_Kd=-0.8f; 
//����ƽ�⻷�⻷
float double_yaw_balance_Angle_Kp=-15.0f;
float double_yaw_balance_Angle_Ki=-0.0f;
float double_yaw_balance_Angle_Kd=0.12f; 
//***********************************************
//����ƽ�⻷�ڻ�
float double_pitch_balance_Gyro_Kp=-0.45f;  
float double_pitch_balance_Gyro_Ki=-0.0f; 
float double_pitch_balance_Gyro_Kd=-0.008f; 
//����ƽ�⻷�ڻ�
float double_roll_balance_Gyro_Kp=-0.45f; 
float double_roll_balance_Gyro_Ki=-0.0f; 
float double_roll_balance_Gyro_Kd=-0.008f; 
//����ƽ�⻷�ڻ�
float double_yaw_balance_Gyro_Kp=0.45f; 
float double_yaw_balance_Gyro_Ki=0.0f;
float double_yaw_balance_Gyro_Kd=0.008f; 
//***********************************************
//�������
float yaw_Kp=1.0f; 
//************MPU6050������ݶ�ȡ********************
void MPU6050_Data_read(void)
{
	//�յ���װλ��Ӱ�죬������pitch��roll
	//ǰ��pitch�������󣬺���pitch��������    ��Ӧgyroy������ͬ
	//�󷭹�roll���������ҷ���roll��������  ��Ӧgyrox������ͬ
	//����yaw��ת������ת��С               ��Ӧgyroz������ͬ
   mpu6050_data_flag=mpu_dmp_get_data(&roll,&pitch,&yaw); //�õ��Ƕ�����
	//MPU_Get_Accelerometer(&aacy,&aacx,&aacz); //�õ����ٶȴ���������
	 MPU_Get_Gyroscope(&gyroy,&gyrox,&gyroz);	  //�õ�����������	  
}
//***********����ֵ����*****************************
int my_abs(int n)
{
  if(n<0)
	{
	  n=(-n);
	}
	if(n>=0)
	{
	 n=n;
	}
	return n;
}
//***********�������*******************************
//�ɻ������Ӧͼ
//        1��    2��
//  ��ͷ��     \/
//            /\
//        3��    4��
//pwm: ���pwm
//n:   ������
void Moto_Ctrl(int pwm,u8 n)
{
	if(pwm<=0)  //�����������
	{
	 pwm=0;
	}
	if(pwm>=7200)  //�����������
	{
	 pwm=7200;
	}
	if(n==1)
	{
	  TIM_SetCompare3(TIM3,pwm);  //1�ŵ��
	}
	if(n==2)
	{
	  TIM_SetCompare2(TIM3,pwm);  //2�ŵ��
	}
	if(n==3)
	{
	  TIM_SetCompare1(TIM3,pwm);  //3�ŵ��
	}
	if(n==4)
	{
	  TIM_SetCompare4(TIM3,pwm);  //4�ŵ�� 
	}			   
}
//********ң������ת������************************************
//n:ң�������յ�����
float pitch_remote_num(u8 n)
{
	float angle=0.0f;
	//�ɻ�����������˶�
  if(n<50)
	{
	  angle=-MAX_PITCH_ANGLE*(50-n);
	}
	//�ɻ�����ƽ��
  if(n==50)
	{
	  angle=0.0f;
	}	
  //�ɻ�ǰ�㣬��ǰ�˶�
  if(n>50)
	{
	  angle=MAX_PITCH_ANGLE*(n-50);
	}		
	return angle;
}
//********ң������ת������************************************
//n:ң�������յ�����
float roll_remote_num(u8 n)
{
	float angle=0.0f;
	//�ɻ����㣬�����˶�
  if(n<50)
	{
	  angle=MAX_ROLL_ANGLE*(50-n);
	}
	//�ɻ�����ƽ��
  if(n==50)
	{
	  angle=0.0f;
	}	
  //�ɻ����㣬�����˶�
  if(n>50)
	{
	  angle=-MAX_ROLL_ANGLE*(n-50);
	}		
	return angle;
}
//*************************************************************
//���ﺽ��Ϊ�Ǿ�ȷ���ƣ������ֵ�����˫��,
//��ͷ��ת�����µĻ�ͷ�Ƕȸ�ֵ������Ļ�е���
//���Խ��ߵ��������ת�ټӴ������ķ����ػ�ʹ����������ת�ķ�����ƫ��
//*************************************************************
//�������ܣ����Ʒɻ�����
//Set_turn��Ŀ����ת���ٶ�
int yaw_control(float Set_turn)
{
  int pwm_yaw=0; 
	pwm_yaw=yaw_Kp*YAW_SPEED*Set_turn; //��ת������KpΪ�����ɻ�ת�� 
	return pwm_yaw;
}


//**************************************************************
//              ���˻���̬�����㷨����PID����
//˵��������PID���� ���ýǶȻ�PD������
//      ����PID���ƣ�С�Ƕ���̬�������ȶ������ǿ�����������
//      ��Ƕȿ�����Ӧ֮���ŵ��Ǵ���򵥣�
//***********���˻�����*****************************************
//�������ܣ����Ʒɻ��������򱣳�ˮƽ
//Angle���ɼ�����ʵ�ʽǶ�ֵ
//Gyro�� �ɼ�����ʵ�ʽ��ٶ�ֵ
int single_pitch_balance(float Angle,float Gyro)
{  
   float err;
	 int pwm_balance;
	 float angle_num=0.0f;
	 angle_num=pitch_remote_num(Pitch_n); //ң������
	 err=(single_Fly_Pitch_Zero+angle_num)-Angle;     //����ֵ-ʵ��ֵ�����������ɻ�ƽ�⣬�������ֵ���ǻ�е����������ֵΪң�����ݣ�ң������������ڻ�е����    
	 pwm_balance=single_pitch_balance_Kp*err+Gyro*single_pitch_balance_Kd;//����ƽ����Ƶĵ��PWM
	 return pwm_balance;
}
//***************************************************
//�������ܣ����Ʒɻ��������򱣳�ˮƽ
//Angle���ɼ�����ʵ�ʽǶ�ֵ
//Gyro�� �ɼ�����ʵ�ʽ��ٶ�ֵ
int single_roll_balance(float Angle,float Gyro)
{  
   float err;
	 int pwm_balance;
	 float angle_num=0.0f;
	 angle_num=roll_remote_num(Roll_n); //ң������
	 err=(single_Fly_Roll_Zero+angle_num)-Angle;      //����ֵ-ʵ��ֵ�����������ɻ�ƽ�⣬�������ֵ���ǻ�е����������ֵΪң�����ݣ�ң������������ڻ�е����     
	 pwm_balance=single_roll_balance_Kp*err+Gyro*single_roll_balance_Kd;//����ƽ����Ƶĵ��PWM
	 return pwm_balance;
}
//***************************************************
//�������ܣ����Ʒɻ�ƫ�����򱣳�ˮƽ
//Angle���ɼ�����ʵ�ʽǶ�ֵ
//Gyro�� �ɼ�����ʵ�ʽ��ٶ�ֵ
int single_yaw_balance(float Angle,float Gyro)
{  
   float err;
	 int pwm_balance;
	 err=single_Fly_Yaw_Zero-Angle;    //����ֵ-ʵ��ֵ�����������ɻ�ƽ�⣬�������ֵ���ǻ�е����������ֵΪң�����ݣ�ң������������ڻ�е����     
	 pwm_balance=single_yaw_balance_Kp*err+Gyro*single_yaw_balance_Kd;//����ƽ����Ƶĵ��PWM
	 return pwm_balance;
}
//*********���˻��˶�����**********************************
void single_Fly_Task(void)
{
	single_pitch_balance_out=single_pitch_balance(pitch,gyroy);	//����ƽ�⻷�����
	single_roll_balance_out=single_roll_balance(roll,gyrox);    //����ƽ�⻷�����	
	if(Yaw_n==0)      
	{
	  yaw_out=0;
		single_yaw_balance_out=single_yaw_balance(yaw,gyroz);       //����ƽ�⻷�����
	}	
	if(Yaw_n==0x01)
	{
		single_yaw_balance_out=0;
	  yaw_out=yaw_control(350);                                   //����ת�򻷵����
	  single_Fly_Yaw_Zero=yaw;                                    //���º����0�㣬���δ˴���ɻ�ת���������Զ���λ��ͷ
	}
	if(Yaw_n==0x02)
	{
		single_yaw_balance_out=0;
	  yaw_out=yaw_control(-200);                                   //����ת�򻷵����
	  single_Fly_Yaw_Zero=yaw;                                     //���º����0�㣬���δ˴���ɻ�ת���������Զ���λ��ͷ
	}

	//���Ʋ�������㷨
	PWM_Out1=Oil_n*PWM_OIL+single_pitch_balance_out+single_roll_balance_out+single_yaw_balance_out+yaw_out;                 
	PWM_Out2=Oil_n*PWM_OIL+single_pitch_balance_out-single_roll_balance_out-single_yaw_balance_out-yaw_out;               
  PWM_Out3=Oil_n*PWM_OIL-single_pitch_balance_out+single_roll_balance_out-single_yaw_balance_out-yaw_out;                
	PWM_Out4=Oil_n*PWM_OIL-single_pitch_balance_out-single_roll_balance_out+single_yaw_balance_out+yaw_out; 
  //�������ѹ��
	PWM_Out1*=pwm_adc;
	PWM_Out2*=pwm_adc;
	PWM_Out3*=pwm_adc;
	PWM_Out4*=pwm_adc;
	//���õ����
	Moto_Ctrl(PWM_Out1,1); //���õ�1�ŵ��
	Moto_Ctrl(PWM_Out2,2); //���õ�2�ŵ��
	Moto_Ctrl(PWM_Out3,3); //���õ�3�ŵ��
	Moto_Ctrl(PWM_Out4,4); //���õ�4�ŵ��
}
//**************************************************************
//              ���˻���̬�����㷨����PID����
//˵��������PID���ƣ��⻷���ýǶȻ�PD���ƣ��ڻ����ý��ٶȻ�PD����
//      ����PID���ƣ�����������ǿ�����ֽǶȵ���̬������Ӧ�ٶȿ졣
//      ����ȵ������ӣ�
//***********���˻�����*****************************************
//�������ܣ��������ƣ��⻷�ǶȻ�
//Angle���ɼ�����ʵ�ʽǶ�ֵ
//Gyro�� �ɼ�����ʵ�ʽ��ٶ�ֵ
int double_pitch_balance_Angle(float Angle,float Gyro)
{
   float err;
	 int PID_Angle_Out;
	 float angle_num=0.0f;
	 angle_num=pitch_remote_num(Pitch_n); //ң������
	 err=(double_Fly_Pitch_Zero+angle_num)-Angle;    //����ֵ-ʵ��ֵ�����������ɻ�ƽ�⣬�������ֵ���ǻ�е����������ֵΪң�����ݣ�ң������������ڻ�е����    
   if(Oil_n<30)
	 {
	   double_PID_Pitch_Angle_Integral=0; //��������
	 }		 
	 if(Oil_n>=30)
	 {
		double_PID_Pitch_Angle_Integral+=err; //���ַ���
	 }
	 //�����ַ�
	 if(double_PID_Pitch_Angle_Integral>=Angle_IMAX)
	 {
	   double_PID_Pitch_Angle_Integral=Angle_IMAX;
	 }
	 if(double_PID_Pitch_Angle_Integral<=-Angle_IMAX)
	 {
	   double_PID_Pitch_Angle_Integral=-Angle_IMAX;
	 }
	 PID_Angle_Out=double_pitch_balance_Angle_Kp*err+double_pitch_balance_Angle_Ki*double_PID_Pitch_Angle_Integral+Gyro*double_pitch_balance_Angle_Kd;//�����⻷�����ֵ�������ٶ�����ֵ
	 return PID_Angle_Out;
}
//***************************************************************
//�������ܣ��������ƣ��ڻ����ٶȻ�
//PID_Angle_Out�� �⻷�����
//Gyro��          �ɼ����Ľ��ٶ�
int double_pitch_balance_Gyro(float PID_Angle_Out,float Gyro)
{
	 int pwm_balance;
	 double_PID_Pitch_err=PID_Angle_Out-Gyro;    //����ֵ-ʵ��ֵ  
   if(Oil_n<30)
	 {
	   double_PID_Pitch_Gyro_Integral=0; //��������
	 }		 
	 if(Oil_n>=30)
	 {
		double_PID_Pitch_Gyro_Integral+=double_PID_Pitch_err; //���ַ���
	 }
	 //�����ַ�
	 if(double_PID_Pitch_Gyro_Integral>=Gyro_IMIN)
	 {
	   double_PID_Pitch_Gyro_Integral=Gyro_IMIN;
	 }
	 if(double_PID_Pitch_Gyro_Integral<=-Gyro_IMIN)
	 {
	   double_PID_Pitch_Gyro_Integral=-Gyro_IMIN;
	 }  
	 pwm_balance=double_pitch_balance_Gyro_Kp*double_PID_Pitch_err+double_pitch_balance_Gyro_Ki*double_PID_Pitch_Gyro_Integral+double_pitch_balance_Gyro_Kd*(double_PID_Pitch_err-double_PID_Pitch_last_err);//����ƽ����Ƶĵ��PWM
	 double_PID_Pitch_last_err=double_PID_Pitch_err;    //���汾�����ֵ����Ϊ�´μ���ʱ���ϴ����ֵʹ��
	 return pwm_balance;
}
//****************************************************************
//�������ܣ��������ƣ��⻷�ǶȻ�
//Angle���ɼ�����ʵ�ʽǶ�ֵ
//Gyro�� �ɼ�����ʵ�ʽ��ٶ�ֵ
int double_roll_balance_Angle(float Angle,float Gyro)
{
   float err;
	 int PID_Angle_Out;
	 float angle_num=0.0f;
	 angle_num=roll_remote_num(Roll_n); //ң������
	 err=(double_Fly_Roll_Zero+angle_num)-Angle;    //����ֵ-ʵ��ֵ�����������ɻ�ƽ�⣬�������ֵ���ǻ�е����������ֵΪң�����ݣ�ң������������ڻ�е����    
	 if(Oil_n<30)
	 {
	   double_PID_Roll_Angle_Integral=0; //��������
	 }		 
	 if(Oil_n>=30)
	 {
		double_PID_Roll_Angle_Integral+=err; //���ַ���
	 }
	 //�����ַ�
	 if(double_PID_Roll_Angle_Integral>=Angle_IMAX)
	 {
	   double_PID_Roll_Angle_Integral=Angle_IMAX;
	 }
	 if(double_PID_Roll_Angle_Integral<=-Angle_IMAX)
	 {
	   double_PID_Roll_Angle_Integral=-Angle_IMAX;
	 }
	 PID_Angle_Out=double_roll_balance_Angle_Kp*err+double_roll_balance_Angle_Ki*double_PID_Roll_Angle_Integral+Gyro*double_roll_balance_Angle_Kd;//�����⻷�����ֵ�������ٶ�����ֵ
	 return PID_Angle_Out;
}
//***************************************************************
//�������ܣ��������ƣ��ڻ����ٶȻ�
//PID_Angle_Out�� �⻷�����
//Gyro��          �ɼ����Ľ��ٶ�
int double_roll_balance_Gyro(float PID_Angle_Out,float Gyro)
{
	 int pwm_balance;
	 double_PID_Roll_err=PID_Angle_Out-Gyro;    //����ֵ-ʵ��ֵ    
   if(Oil_n<30)
	 {
	   double_PID_Roll_Gyro_Integral=0; //��������
	 }		 
	 if(Oil_n>=30)
	 {
		double_PID_Roll_Gyro_Integral+=double_PID_Roll_err; //���ַ���
	 }
	 //�����ַ�
	 if(double_PID_Roll_Gyro_Integral>=Gyro_IMIN)
	 {
	   double_PID_Roll_Gyro_Integral=Gyro_IMIN;
	 }
	 if(double_PID_Roll_Gyro_Integral<=-Gyro_IMIN)
	 {
	   double_PID_Roll_Gyro_Integral=-Gyro_IMIN;
	 }
	 pwm_balance=double_roll_balance_Gyro_Kp*double_PID_Roll_err+double_roll_balance_Gyro_Ki*double_PID_Roll_Gyro_Integral+double_roll_balance_Gyro_Kd*(double_PID_Roll_err-double_PID_Roll_last_err);//����ƽ����Ƶĵ��PWM
	 double_PID_Roll_last_err=double_PID_Roll_err;    //���汾�����ֵ����Ϊ�´μ���ʱ���ϴ����ֵʹ��
	 return pwm_balance;
}
//*****************************************************************
//�������ܣ�������ƣ��⻷�ǶȻ�
//Angle���ɼ�����ʵ�ʽǶ�ֵ
//Gyro�� �ɼ�����ʵ�ʽ��ٶ�ֵ
int double_yaw_balance_Angle(float Angle,float Gyro)
{
   float err;
	 int PID_Angle_Out;
	 err=double_Fly_Yaw_Zero-Angle;    //����ֵ-ʵ��ֵ�����������ɻ�ƽ�⣬�������ֵ���ǻ�е����������ֵΪң�����ݣ�ң������������ڻ�е����    
	 if(Oil_n<30)
	 {
	   double_PID_Yaw_Angle_Integral=0; //��������
	 }		 
	 if(Oil_n>=30)
	 {
		double_PID_Yaw_Angle_Integral+=err; //���ַ���
	 }
	 //�����ַ�
	 if(double_PID_Yaw_Angle_Integral>=Angle_IMAX)
	 {
	   double_PID_Yaw_Angle_Integral=Angle_IMAX;
	 }
	 if(double_PID_Yaw_Angle_Integral<=-Angle_IMAX)
	 {
	   double_PID_Yaw_Angle_Integral=-Angle_IMAX;
	 }	 
	 PID_Angle_Out=double_yaw_balance_Angle_Kp*err+double_yaw_balance_Angle_Ki*double_PID_Yaw_Angle_Integral+Gyro*double_yaw_balance_Angle_Kd;//�����⻷�����ֵ�������ٶ�����ֵ
	 return PID_Angle_Out;
}
//***************************************************************
//�������ܣ�������ƣ��ڻ����ٶȻ�
//PID_Angle_Out�� �⻷�����
//Gyro��          �ɼ����Ľ��ٶ�
int double_yaw_balance_Gyro(float PID_Angle_Out,float Gyro)
{
	 int pwm_balance;
	 double_PID_Yaw_err=PID_Angle_Out-Gyro;    //����ֵ-ʵ��ֵ    
   if(Oil_n<30)
	 {
	   double_PID_Yaw_Gyro_Integral=0; //��������
	 }		 
	 if(Oil_n>=30)
	 {
		double_PID_Yaw_Gyro_Integral+=double_PID_Roll_err; //���ַ���
	 }
	 //�����ַ�
	 if(double_PID_Yaw_Gyro_Integral>=Gyro_IMIN)
	 {
	   double_PID_Yaw_Gyro_Integral=Gyro_IMIN;
	 }
	 if(double_PID_Yaw_Gyro_Integral<=-Gyro_IMIN)
	 {
	   double_PID_Yaw_Gyro_Integral=-Gyro_IMIN;
	 }
 	 pwm_balance=double_yaw_balance_Gyro_Kp*double_PID_Yaw_err+double_yaw_balance_Gyro_Ki*double_PID_Yaw_Gyro_Integral+double_yaw_balance_Gyro_Kd*(double_PID_Yaw_err-double_PID_Yaw_last_err);//����ƽ����Ƶĵ��PWM
	 double_PID_Yaw_last_err=double_PID_Yaw_err;    //���汾�����ֵ����Ϊ�´μ���ʱ���ϴ����ֵʹ��
	 return pwm_balance;
}
//*********���˻��˶�����**********************************
void double_Fly_Task(void)
{
	double_pitch_balance_angle_out=double_pitch_balance_Angle(pitch,gyroy);									  //����ƽ�⻷�⻷�����
	double_roll_balance_angle_out=double_roll_balance_Angle(roll,gyrox);   										//����ƽ�⻷�⻷�����
	double_pitch_balance_out=double_pitch_balance_Gyro(double_pitch_balance_angle_out,gyroy); //����ƽ�⻷�ڻ������
	double_roll_balance_out=double_roll_balance_Gyro(double_roll_balance_angle_out,gyrox);    //����ƽ�⻷�ڻ������
	if(Yaw_n==0)      
	{
	  yaw_out=0;
		double_yaw_balance_angle_out=double_yaw_balance_Angle(yaw,gyroz);									  //����ƽ�⻷�⻷�����
	  double_yaw_balance_out=double_yaw_balance_Gyro(double_yaw_balance_angle_out,gyroz); //����ƽ�⻷�ڻ������
	}	
	if(Yaw_n==0x01)
	{
		double_yaw_balance_out=0;
	  yaw_out=yaw_control(350);                                   //����ת�򻷵����
	  double_Fly_Yaw_Zero=yaw;                                    //���º����0�㣬���δ˴���ɻ�ת���������Զ���λ��ͷ
	}
	if(Yaw_n==0x02)
	{
		double_yaw_balance_out=0;
	  yaw_out=yaw_control(-200);                                   //����ת�򻷵����
	  double_Fly_Yaw_Zero=yaw;                                     //���º����0�㣬���δ˴���ɻ�ת���������Զ���λ��ͷ
	}
	//���Ʋ�������㷨
	PWM_Out1=Oil_n*PWM_OIL+double_pitch_balance_out+double_roll_balance_out+double_yaw_balance_out+yaw_out;                 
	PWM_Out2=Oil_n*PWM_OIL+double_pitch_balance_out-double_roll_balance_out-double_yaw_balance_out-yaw_out;               
  PWM_Out3=Oil_n*PWM_OIL-double_pitch_balance_out+double_roll_balance_out-double_yaw_balance_out-yaw_out;                
	PWM_Out4=Oil_n*PWM_OIL-double_pitch_balance_out-double_roll_balance_out+double_yaw_balance_out+yaw_out; 
  //�������ѹ��
	PWM_Out1*=pwm_adc;
	PWM_Out2*=pwm_adc;
	PWM_Out3*=pwm_adc;
	PWM_Out4*=pwm_adc;
	//���õ����
	Moto_Ctrl(PWM_Out1,1); //���õ�1�ŵ��
	Moto_Ctrl(PWM_Out2,2); //���õ�2�ŵ��
	Moto_Ctrl(PWM_Out3,3); //���õ�3�ŵ��
	Moto_Ctrl(PWM_Out4,4); //���õ�4�ŵ��
}
















