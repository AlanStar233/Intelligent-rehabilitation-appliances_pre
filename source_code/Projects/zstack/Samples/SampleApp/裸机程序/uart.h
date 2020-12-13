#ifndef _UART_H_
#define _UART_H_

extern void uartInit(void);
extern void uartPutChar(char s);
extern void uartSendString(char *Data,int len);
extern void uartPutInt16(int s);


#endif 