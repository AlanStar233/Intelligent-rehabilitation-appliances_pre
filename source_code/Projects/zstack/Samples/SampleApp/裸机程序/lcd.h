#ifndef _LCD_H_
#define _LCD_H_

//串行发送IO口定义
#define L_CS P1_2	//_CS
//#define L_RS P1_0	//_RES  hardware reset
#define L_LD P0_0       //A0=H data A0=L commend 
#define L_CK P1_5	//SCLK
#define L_DA P1_6	//SI
#define L_BK P0_7	//backlight

extern void lcdInit( void );
extern void lcdPrintString(uint8 x, uint8 y, uchar *pstr);
extern void lcdPrintUint8(uint8 x, uint8 y, uint8 data);

#endif 