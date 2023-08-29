#ifndef _OLED_H
#define _OLED_H
#include "pbdata.h"
//-----------------OLED�˿ڶ���---------------- 
#define OLED_SCL_Clr() GPIO_ResetBits(GPIOA,GPIO_Pin_8)// CLK
#define OLED_SCL_Set() GPIO_SetBits(GPIOA,GPIO_Pin_8)

#define OLED_SDA_Clr() GPIO_ResetBits(GPIOA,GPIO_Pin_7)//MOS
#define OLED_SDA_Set() GPIO_SetBits(GPIOA,GPIO_Pin_7)

#define OLED_RES_Clr() GPIO_ResetBits(GPIOA,GPIO_Pin_15)//FS0
#define OLED_RES_Set() GPIO_SetBits(GPIOA,GPIO_Pin_15)

#define OLED_DC_Clr()  GPIO_ResetBits(GPIOA,GPIO_Pin_6)//DC
#define OLED_DC_Set()  GPIO_SetBits(GPIOA,GPIO_Pin_6)
 		     
#define OLED_CS_Clr()  GPIO_ResetBits(GPIOA,GPIO_Pin_5)//CS1
#define OLED_CS_Set()  GPIO_SetBits(GPIOA,GPIO_Pin_5)

#define OLED_CMD  0	//д����
#define OLED_DATA 1	//д����

extern const u8 logo[512];

void OLED_ClearPoint(u8 x,u8 y);
void OLED_ColorTurn(u8 i);
void OLED_DisplayTurn(u8 i);
void OLED_WR_Byte(u8 dat,u8 mode);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2,u8 mode);
void OLED_DrawCircle(u8 x,u8 y,u8 r);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size1,u8 mode);
void OLED_ShowChar6x8(u8 x,u8 y,u8 chr,u8 mode);
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 size1,u8 mode);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size1,u8 mode);
void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1,u8 mode);
void OLED_ScrollDisplay(u8 num,u8 space,u8 mode);
void OLED_ShowPicture(u8 x,u8 y,u8 sizex,u8 sizey,const u8 BMP[],u8 mode);
void OLED_Init(void);

#endif






