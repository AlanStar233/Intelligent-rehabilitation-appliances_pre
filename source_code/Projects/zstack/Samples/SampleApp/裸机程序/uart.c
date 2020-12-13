#include <ioCC2530.h>
#include "uart.h"

/****************************************************************
   ���ڳ�ʼ������				
****************************************************************/
void uartInit(void)
{ 
  PERCFG = 0x00;		      //λ��1 P0��
  P0SEL = 0x0c;		      //P0_2,P0_3�������ڣ��ⲿ�豸���ܣ�
  P2DIR &= ~0XC0;                   //P0������ΪUART0
  
  U0CSR |= 0x80;		      //����ΪUART��ʽ
  U0GCR |= 11;				       
  U0BAUD |= 216;		      //��������Ϊ115200
  UTX0IF = 0;                       //UART0 TX�жϱ�־��ʼ��λ0
}

void uartPutChar(char s)
{
  U0DBUF = s;
  while(UTX0IF == 0);
  UTX0IF = 0;
}


/****************************************************************
���ڷ����ַ�������			
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