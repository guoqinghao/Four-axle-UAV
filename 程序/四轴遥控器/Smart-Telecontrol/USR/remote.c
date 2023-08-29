#include "pbdata.h"
u8 USART1_MSG_flag=0;
u8 USART1_MSG_Value=0;
u8 USART1_Count=0;
u8 USART1_Cmd_Flag=0;       //接收包完毕标志位
u8 USART1_Receieve[8];  
u8 Data1=0;
u8 Data2=0;
u8 msg_send[5];       //发送数据包
//****************串口1时钟**************************
void USART1_MSG_RCC_Configuration(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}
//***************串口1引脚配置***********************************
void USART1_MSG_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}	
//*****************串口1功能配置**********************************
void USART1_MSG_Configration(void)
{
  USART_InitTypeDef USART_InitStructure;                      
  USART_InitStructure.USART_BaudRate = 9600;                  
  USART_InitStructure.USART_WordLength = USART_WordLength_8b; 
  USART_InitStructure.USART_StopBits = USART_StopBits_1;      
  USART_InitStructure.USART_Parity = USART_Parity_No;        
  USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; 
  USART_Init(USART1, &USART_InitStructure);                    
  USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);                
  USART_Cmd(USART1,ENABLE);                                    
  USART_ClearFlag(USART1,USART_FLAG_TC);                       
}
//****************串口1优先级********************************
void USART1_MSG_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;                        
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);             
  NVIC_InitStructure.NVIC_IRQChannel =USART1_IRQn;            
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;              
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             
  NVIC_Init(&NVIC_InitStructure);             
}
//*******************串口1初始化**********************************
void USART1_MSG_Init(void)
{
  USART1_MSG_RCC_Configuration();
  USART1_MSG_GPIO_Configuration();
  USART1_MSG_Configration();
  USART1_MSG_NVIC_Configuration();
}
//********************串口1中断********************************
void USART1_IRQHandler(void)
{                
   if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET) 
	 {
		 USART1_MSG_flag=1;
	   USART1_MSG_Value=USART_ReceiveData(USART1);           		 
     USART1_Receieve[USART1_Count]=USART1_MSG_Value;         
		 USART1_Count++;			 
		 if(USART1_Count>7)        //一个包7个字节， 2*4-1=7                           
			{
				 USART1_Count=0;	
         USART1_Cmd_Flag=1;			
			}	
		 //USART_SendData(USART1,USART_MSG_Value);
		 while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	 }
}
//***********************串口1数据解码**********************
void Receive_Data(void)
{ 
	u8 j;		
	 for(j=0;j<8;j++)
	{
	   if(USART1_Receieve[j]==0xFE)                     
		 {                                               
			 if(USART1_Receieve[j+3]==((USART1_Receieve[j+2]+USART1_Receieve[j+1]+USART1_Receieve[j])&0x00FF))  
			 {		 					 					
					Data1=USART1_Receieve[j+1];  
					Data2=USART1_Receieve[j+2];			 
			 }
	   }
   }
	USART1_Count=0;	
}
//********************发送函数********************
void MSG_Send(u8 msg[5])
{
	u8 i;		
  u8 msgbox[7];
	
	msgbox[0]=0xFF;
	msgbox[1]=msg[0];
	msgbox[2]=msg[1];
	msgbox[3]=msg[2];
	msgbox[4]=msg[3];
	msgbox[5]=msg[4];
	msgbox[6]=(0xFF+msg[0]+msg[1]+msg[2]+msg[3]+msg[4])&0x00FF;
	
  for(i=0;i<7;i++)
	{
	   USART_SendData(USART1,msgbox[i]); 
		 while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	}
	
	for(i=0;i<7;i++)
	{
	   USART_SendData(USART1,msgbox[i]); 
		 while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	}	
}			
//*******************接收函数******************************
void MSG_Recieve(void)
{
	//解码获取的数据
	if(USART1_Cmd_Flag==1)
	{
		USART1_Cmd_Flag=0;   
		Receive_Data();      
	}
//**************飞机电量解码****************************	
	if(Data1==0x01)
	{
	  Plane_Bat=Data2;
		Data1=0;
		Data2=0;
		USART1_Count=0;
	}
//**************飞机锁机状态解码****************************	
	if(Data1==0x02)
	{
	  Plane_status=Data2;
		Data1=0;
		Data2=0;
		USART1_Count=0;
	}
}















