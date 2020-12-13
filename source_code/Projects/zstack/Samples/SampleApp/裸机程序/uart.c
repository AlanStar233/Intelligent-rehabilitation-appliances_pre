#include <ioCC2530.h>
#include "uart.h"

/****************************************************************
   串口初始化函数				
****************************************************************/
void uartInit(void)
{ 
  PERCFG = 0x00;		      //位置1 P0口
  P0SEL = 0x0c;		      //P0_2,P0_3用作串口（外部设备功能）
  P2DIR &= ~0XC0;                   //P0优先作为UART0
  
  U0CSR |= 0x80;		      //设置为UART方式
  U0GCR |= 11;				       
  U0BAUD |= 216;		      //波特率设为115200
  UTX0IF = 0;                       //UART0 TX中断标志初始置位0
}

void uartPutChar(char s)
{
  U0DBUF = s;
  while(UTX0IF == 0);
  UTX0IF = 0;
}


/****************************************************************
串口发送字符串函数			
****************************************************************/
void uartSendString(char *Data,int len)
{
  int j;
  for(j=0;j<len;j++)
  {
    uartPutChar(*Data++);
  }
}

void uartPutInt16(int num)
{
  char temp[5];
  if(num<0)
  {
    uartPutChar('-');
    num = -num;
  }
  else
  {
    uartPutChar('+');
  }
  temp[0] = num/10000+0x30;
  temp[1] = num%10000/1000+0x30;
  temp[2] = num%1000/100+0x30;
  temp[3] = num%100/10+0x30;
  temp[4] = num%10+0x30;
  uartSendString(temp,5);
}