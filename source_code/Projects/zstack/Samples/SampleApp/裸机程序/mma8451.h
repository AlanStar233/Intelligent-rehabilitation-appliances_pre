#ifndef _MMA8451_H_
#define _MMA8451_H_


#define	SlaveAddress   0x38	//定义器件在IIC总线中的从地址,根据SA0地址引脚不同修改
#define SCL P1_6      //IIC时钟引脚定义
#define SDA P1_7      //IIC数据引脚定义

#define SDA_OUT()   P1DIR |= 0x80
#define SDA_IN()    P1DIR &= ~0x80
#define SCL_OUT()   P1DIR |= 0x40

extern int mmaX,mmaY,mmaZ;
extern void Init_MMA8452(void);
extern void Multiple_read_MMA8452(void);
extern unsigned char Single_Read_MMA8452(unsigned char REG_Address);

#endif