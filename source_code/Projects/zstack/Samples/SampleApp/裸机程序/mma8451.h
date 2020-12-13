#ifndef _MMA8451_H_
#define _MMA8451_H_


#define	SlaveAddress   0x38	//����������IIC�����еĴӵ�ַ,����SA0��ַ���Ų�ͬ�޸�
#define SCL P1_6      //IICʱ�����Ŷ���
#define SDA P1_7      //IIC�������Ŷ���

#define SDA_OUT()   P1DIR |= 0x80
#define SDA_IN()    P1DIR &= ~0x80
#define SCL_OUT()   P1DIR |= 0x40

extern int mmaX,mmaY,mmaZ;
extern void Init_MMA8452(void);
extern void Multiple_read_MMA8452(void);
extern unsigned char Single_Read_MMA8452(unsigned char REG_Address);

#endif