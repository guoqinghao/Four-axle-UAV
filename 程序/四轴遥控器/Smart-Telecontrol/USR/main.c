#include "pbdata.h"
int main(void)
{ 
	Hard_Init();     //ϵͳ��ʼ��
	photo(0);        //��ʾLOGOҳ��
  delay_ms(1000); 
  photo(1);        //��ʾ��ҳ
  while(1)
  {
    UI(); 
  }  
}

