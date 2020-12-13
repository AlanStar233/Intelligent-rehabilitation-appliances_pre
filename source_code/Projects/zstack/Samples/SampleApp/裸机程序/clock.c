/**************************************/
/*           WeBee团队                */
/*           Zigbee                   */
/*例程名称：                          */
/*建立时间：2018/5/18                 */
/*描述：clock（时钟）
**************************************/

#include <ioCC2530.h>
#include "type.h"
#include "clock.h"
void clockInit(void)
{
  CLKCONCMD &= ~0x40;               //设置系统时钟源为32MHZ晶振
  while(CLKCONSTA & 0x40);          //等待晶振稳定为32M
  CLKCONCMD &= ~0x47;               //设置系统主时钟频率为32MHZ   
}

/**************************************
          32M晶振毫秒延时函数
**************************************/
void delayMs(uint16 t)
{
  uint16 k,j;
  for(k=0;k<t;k++)
  {
    for(j=0;j<1774;j++);
  }
}
