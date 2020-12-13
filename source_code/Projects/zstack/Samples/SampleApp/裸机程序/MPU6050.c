/**************************************/
/*           WeBee团队                */
/*           Zigbee                   */
/*例程名称：                          */
/*建立时间：2018/5/18                 */
/*描述：    MPU6050驱动函数
**************************************/
#include <ioCC2530.h>
#include "type.h"
#include"MPU6050.h"
int accX,accY,accZ,graX,graY,graZ;
static void mma_Delay5us(void);
static void MPU6050_Start(void);
static void MPU6050_Stop(void);
static void MPU6050_SendACK(char ack);
static char MPU6050_RecvACK(void);
static void MPU6050_SendByte(unsigned char dat);
static unsigned char MPU6050_RecvByte(void);
static void Single_Write_MPU6050(unsigned char REG_Address,unsigned char REG_data);
unsigned char Single_Read_MPU6050(unsigned char REG_Address);


/**************************************
延时5微秒
**************************************/
static void mma_Delay5us(void)//uint16 microSecs)
{
  uint8 microSecs = 5;
  while(microSecs--)            
  {
    /* 32 NOPs == 1 usecs */
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop");
  
  }
}

/**************************************
起始信号
**************************************/
static void MPU6050_Start()
{
  SDA = 1;                    //拉高数据线
  SCL = 1;                    //拉高时钟线
  mma_Delay5us();                 
  SDA = 0;                    //产生下降沿
  mma_Delay5us();             //延时
  SCL = 0;                    //拉低时钟线
  mma_Delay5us();         
}

/**************************************
停止信号
**************************************/
static void MPU6050_Stop()
{
  SDA = 0;                    //拉低数据线
  SCL = 1;                    //拉高时钟线
  mma_Delay5us();                 //延时
  SDA = 1;                    //产生上升沿
  mma_Delay5us();                 //延时
}

/**************************************
发送应答信号
入口参数:ack (0:ACK 1:NAK)
**************************************/
static void MPU6050_SendACK(char ack)
{
  SDA = ack;                  //写应答信号
  SCL = 1;                    //拉高时钟线
  mma_Delay5us();                 //延时
  SCL = 0;                    //拉低时钟线
  mma_Delay5us();                 //延时
}

/**************************************
接收应答信号
**************************************/
static char MPU6050_RecvACK()
{
  SCL = 1;                    //拉高时钟线
  mma_Delay5us();            
  SDA_IN();
  CY = SDA;                   //读应答信号
  SDA_OUT();
  SCL = 0;                    //拉低时钟线
  mma_Delay5us();                 //延时

  return CY;
}

/**************************************
向IIC总线发送一个字节数据
**************************************/
static void MPU6050_SendByte(unsigned char dat)
{
  unsigned char i;
  
  for (i=0; i<8; i++)         //8位计数器
  {
    dat <<= 1;              //移出数据的最高位
    SDA = CY;               //送数据口
    SCL = 1;                //拉高时钟线
    mma_Delay5us();             //延时
    SCL = 0;                //拉低时钟线
    mma_Delay5us();             //延时
  }
  MPU6050_RecvACK();
}

/**************************************
从IIC总线接收一个字节数据
**************************************/
static unsigned char MPU6050_RecvByte()
{
  unsigned char i;
  unsigned char dat = 0;
  
  SDA = 1;                    //使能内部上拉,准备读取数据,
  for (i=0; i<8; i++)         //8位计数器
  {
    dat <<= 1;
    SCL = 1;                //拉高时钟线
    mma_Delay5us();             //延时
    SDA_IN();
    dat |= SDA;             //读数据  
    SDA_OUT();
    SCL = 0;                //拉低时钟线
    mma_Delay5us();             //延时
  }
  return dat;
}

//******单字节写入*******************************************
static void Single_Write_MPU6050(unsigned char REG_Address,unsigned char REG_data)
{
  MPU6050_Start();                  //起始信号
  MPU6050_SendByte(SlaveAddress);   //发送设备地址+写信号
  MPU6050_SendByte(REG_Address);    //内部寄存器地址
  MPU6050_SendByte(REG_data);       //内部寄存器数据
  MPU6050_Stop();                   //发送停止信号
}

//********单字节读取*****************************************

unsigned char Single_Read_MPU6050(unsigned char REG_Address)
{  
  unsigned char REG_data;
  MPU6050_Start();                          //起始信号
  MPU6050_SendByte(SlaveAddress);           //发送设备地址+写信号
  MPU6050_SendByte(REG_Address);            //发送存储单元地址，从0开始	
  MPU6050_Start();                          //起始信号
  MPU6050_SendByte(SlaveAddress+1);         //发送设备地址+读信号
  REG_data=MPU6050_RecvByte();              //读出寄存器数据
  MPU6050_SendACK(1);   
  MPU6050_Stop();                           //停止信号
  return REG_data; 
}


//*****************************************************************
//                        初始化MPU6050
//*****************************************************************
void Init_MPU6050(void)
{
  P1SEL &= ~0xc0; //作为普通 IO 口
  SDA_OUT();
  SCL_OUT(); 
  Single_Write_MPU6050(PWR_MGMT_1, 0x00);//电源管理，典型值：0x00(正常启用)
  Single_Write_MPU6050(SMPLRT_DIV, 0x07);
  Single_Write_MPU6050(CONFIG, 0x06);
  Single_Write_MPU6050(GYRO_CONFIG, 0x18);
  Single_Write_MPU6050(ACCEL_CONFIG, 0x01); 
}
//*********************************************************
//
//连续读出MPU6050内部数据
//
//*********************************************************
void Multiple_read_MPU6050(void)
{   
  char BUF[12]; //接收数据缓存区  
  BUF[0]=Single_Read_MPU6050(ACCEL_XOUT_H);
  BUF[1]=Single_Read_MPU6050(ACCEL_XOUT_L);
  BUF[2]=Single_Read_MPU6050(ACCEL_YOUT_H);
  BUF[3]=Single_Read_MPU6050(ACCEL_YOUT_L);
  BUF[4]=Single_Read_MPU6050(ACCEL_ZOUT_H);
  BUF[5]=Single_Read_MPU6050(ACCEL_ZOUT_L);
  
  BUF[6]=Single_Read_MPU6050(GYRO_XOUT_H);
  BUF[7]=Single_Read_MPU6050(GYRO_XOUT_L);
  BUF[8]=Single_Read_MPU6050(GYRO_YOUT_H);
  BUF[9]=Single_Read_MPU6050(GYRO_YOUT_L);
  BUF[10]=Single_Read_MPU6050(GYRO_ZOUT_H);
  BUF[11]=Single_Read_MPU6050(GYRO_ZOUT_L);
  
  accX=(BUF[0]<<8)|BUF[1]; 
  accY=(BUF[2]<<8)|BUF[3]; 
  accZ=(BUF[4]<<8)|BUF[5]; 
  graX=(BUF[6]<<8)|BUF[7];
  graY=(BUF[8]<<8)|BUF[9];
  graZ=(BUF[10]<<8)|BUF[11];
}
