#ifndef _moto_H
#define _moto_H
#include "pbdata.h"
//*************��ر���***********************
extern u8 mpu6050_data_flag;   								//�Ƿ�������ȡ����
extern float pitch,roll,yaw;  								//ŷ����
extern short aacx,aacy,aacz;									//���ٶȴ�����ԭʼ����
extern short gyrox,gyroy,gyroz;								//������ԭʼ����
extern short mpu6050_temp;		 								//�¶�	
extern u8 Oil_n;   														//������ֵ
extern u8 Pitch_n; 														//������ֵ
extern u8 Roll_n;  														//������ֵ
extern u8 Yaw_n;   														//������ֵ
extern int PWM_Out1; 													//�������õ������PWM
extern int PWM_Out2; 													//�������õ������PWM
extern int PWM_Out3; 													//�������õ������PWM
extern int PWM_Out4; 													//�������õ������PWM
extern int single_pitch_balance_out;  				//����ƽ�⻷���������
extern int single_roll_balance_out;   				//����ƽ�⻷���������
extern int single_yaw_balance_out;            //ƫ��ƽ�⻷���������
extern int double_pitch_balance_angle_out;    //����ƽ���⻷�����
extern int double_roll_balance_angle_out;     //����ƽ���⻷�����
extern int double_yaw_balance_angle_out;      //ƫ��ƽ���⻷����� 
extern int double_pitch_balance_out;          //����ƽ�⻷���������
extern int double_roll_balance_out;           //����ƽ�⻷���������
extern int double_yaw_balance_out;            //ƫ��ƽ�⻷���������
extern int yaw_out;                           //���򻷵��������
extern float pwm_adc;                         //���ڲ�����ѹ������PWM�����
extern float double_PID_Pitch_err;            //�����ڻ����ֵ  
extern float double_PID_Pitch_last_err;       //�����ڻ��ϴ����ֵ
extern float double_PID_Pitch_Angle_Integral; //�����⻷����
extern float double_PID_Pitch_Gyro_Integral;  //�����ڻ�����
extern float double_PID_Roll_err;             //�����ڻ����ֵ
extern float double_PID_Roll_last_err;        //�����ڻ��ϴ����ֵ
extern float double_PID_Roll_Angle_Integral;  //�����⻷����
extern float double_PID_Roll_Gyro_Integral;   //�����ڻ�����
extern float double_PID_Yaw_err;              //�����ڻ����ֵ
extern float double_PID_Yaw_last_err;         //�����ڻ��ϴ����ֵ
extern float double_PID_Yaw_Angle_Integral;   //�����⻷����
extern float double_PID_Yaw_Gyro_Integral;    //�����ڻ�����
//****************�ɻ��Ļ�е���***************
extern float single_Fly_Pitch_Zero;   
extern float single_Fly_Roll_Zero; 
extern float single_Fly_Yaw_Zero; 
//****************�ɻ��Ļ�е���***************
extern float double_Fly_Pitch_Zero;   
extern float double_Fly_Roll_Zero; 
extern float double_Fly_Yaw_Zero; 
//*************����PID��������*****************
//����ƽ�⻷
extern float single_pitch_balance_Kp; 
extern float single_pitch_balance_Kd; 
//����ƽ�⻷
extern float single_roll_balance_Kp; 
extern float single_roll_balance_Kd; 
//����ƽ�⻷
extern float single_yaw_balance_Kp; 
extern float single_yaw_balance_Kd;
//*************����PID��������*****************
//����ƽ�⻷�⻷
extern float double_pitch_balance_Angle_Kp; 
extern float double_pitch_balance_Angle_Ki; 
extern float double_pitch_balance_Angle_Kd; 
//����ƽ�⻷�⻷
extern float double_roll_balance_Angle_Kp; 
extern float double_roll_balance_Angle_Ki; 
extern float double_roll_balance_Angle_Kd; 
//����ƽ�⻷�⻷
extern float double_yaw_balance_Angle_Kp;
extern float double_yaw_balance_Angle_Ki;
extern float double_yaw_balance_Angle_Kd; 
//***********************************************
//����ƽ�⻷�ڻ�
extern float double_pitch_balance_Gyro_Kp; 
extern float double_pitch_balance_Gyro_Ki; 
extern float double_pitch_balance_Gyro_Kd; 
//����ƽ�⻷�ڻ�
extern float double_roll_balance_Gyro_Kp;
extern float double_roll_balance_Gyro_Ki;
extern float double_roll_balance_Gyro_Kd; 
//����ƽ�⻷�ڻ�
extern float double_yaw_balance_Gyro_Kp; 
extern float double_yaw_balance_Gyro_Ki; 
extern float double_yaw_balance_Gyro_Kd;
//�������
extern float yaw_Kp; 
//*****************��������************************
void MPU6050_Data_read(void);
int my_abs(int n);
void Moto_Ctrl(int pwm,u8 n);
float pitch_remote_num(u8 n);
float roll_remote_num(u8 n);
int yaw_control(float Set_turn);
int single_pitch_balance(float Angle,float Gyro);    
int single_roll_balance(float Angle,float Gyro);
int single_yaw_balance(float Angle,float Gyro);
void single_Fly_Task(void);
int double_pitch_balance_Angle(float Angle,float Gyro);
int double_pitch_balance_Gyro(float PID_Angle_Out,float Gyro);
int double_roll_balance_Angle(float Angle,float Gyro);
int double_roll_balance_Gyro(float PID_Angle_Out,float Gyro);
int double_yaw_balance_Angle(float Angle,float Gyro);
int double_yaw_balance_Gyro(float PID_Angle_Out,float Gyro);
void double_Fly_Task(void);
#endif

