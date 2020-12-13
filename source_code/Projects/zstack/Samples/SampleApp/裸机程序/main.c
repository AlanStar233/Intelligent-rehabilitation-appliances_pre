/**************************************/
/*           WeBee团队                */
/*           Zigbee                   */
/*例程名称：                          */
/*建立时间：2018/5/18                 */
/*描述：main
**************************************/
#include <ioCC2530.h>
#include "type.h"
#include "clock.h"
#include "led.h"
#include "uart.h"
#include"MPU6050.h"
void main(void)
{
  clockInit(); 
  uartInit(); 
  ledInit();    
  LED1 = 1; 
  LED2 = 0; 
  Init_MPU6050();                //初始化MPU6050
  while(1)
  {      
    Multiple_read_MPU6050();
    uartPutInt16(accX);
    uartSendString("   ",3);
    uartPutInt16(accY);
    uartSendString("   ",3);
    uartPutInt16(accZ);
    uartSendString("   ",3);

    uartSendString("\n",1);
    uartPutInt16(graX);
    uartSendString("   ",3);
    uartPutInt16(graY);
    uartSendString("   ",3);
    uartPutInt16(graZ);
    uartSendString("\n",1);
    LED1 = ~LED1; 
    LED2 = ~LED2; 
    delayMs(1000);
  }
}
