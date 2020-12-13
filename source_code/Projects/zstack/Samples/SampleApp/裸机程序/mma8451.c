#include <ioCC2530.h>
#include "type.h"
#include"mma8451.h"

int mmaX,mmaY,mmaZ;

static void mma_Delay5us(void);
static void MMA8452_Start(void);
static void MMA8452_Stop(void);
static void MMA8452_SendACK(char ack);
static char MMA8452_RecvACK(void);
static void MMA8452_SendByte(unsigned char dat);
static unsigned char MMA8452_RecvByte(void);
static void Single_Write_MMA8452(unsigned char REG_Address,unsigned char REG_data);
unsigned char Single_Read_MMA8452(unsigned char REG_Address);


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
static void MMA8452_Start()
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
static void MMA8452_Stop()
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
static void MMA8452_SendACK(char ack)
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
static char MMA8452_RecvACK()
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
static void MMA8452_SendByte(unsigned char dat)
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
  MMA8452_RecvACK();
}

/**************************************
从IIC总线接收一个字节数据
**************************************/
static unsigned char MMA8452_RecvByte()
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
static void Single_Write_MMA8452(unsigned char REG_Address,unsigned char REG_data)
{
  MMA8452_Start();                  //起始信号
  MMA8452_SendByte(SlaveAddress);   //发送设备地址+写信号
  MMA8452_SendByte(REG_Address);    //内部寄存器地址
  MMA8452_SendByte(REG_data);       //内部寄存器数据
  MMA8452_Stop();                   //发送停止信号
}

//********单字节读取*****************************************

unsigned char Single_Read_MMA8452(unsigned char REG_Address)
{  
  unsigned char REG_data;
  MMA8452_Start();                          //起始信号
  MMA8452_SendByte(SlaveAddress);           //发送设备地址+写信号
  MMA8452_SendByte(REG_Address);                   //发送存储单元地址，从0开始	
  MMA8452_Start();                          //起始信号
  MMA8452_SendByte(SlaveAddress+1);         //发送设备地址+读信号
  REG_data=MMA8452_RecvByte();              //读出寄存器数据
  MMA8452_SendACK(1);   
  MMA8452_Stop();                           //停止信号
  return REG_data; 
}


//*****************************************************************

//初始化MMA8452，根据需要请参考pdf进行修改************************
void Init_MMA8452(void)
{
  P1SEL &= ~0xc0; //作为普通 IO 口
  //P1INP &= ~0Xc0; //打开上拉
  //P1INP |= 0Xc0; //打开下拉
  SDA_OUT();
  SCL_OUT();
  
  Single_Write_MMA8452(0x2A,0x01);  //唤醒装置
  Single_Write_MMA8452(0x2B,0x02);  //设置量程
}



//*********************************************************
//
//连续读出MMA8452内部加速度数据，地址范围0x01~0x06
//
//*********************************************************
void Multiple_read_MMA8452(void)
{   
  char BUF[6];                         //接收数据缓存区  
  unsigned char i;
  MMA8452_Start();                          //起始信号
  MMA8452_SendByte(SlaveAddress);           //发送设备地址+写信号
  MMA8452_SendByte(0x01);                   //发送存储单元地址，从0x01开始	
  MMA8452_Start();                          //起始信号
  MMA8452_SendByte(SlaveAddress+1);         //发送设备地址+读信号
  for (i=0; i<6; i++)                      //连续读取6个地址数据，存储中BUF
  {
    BUF[i] = MMA8452_RecvByte();          //BUF[0]存储0x32地址中的数据
    if (i == 5)
    {
      MMA8452_SendACK(1);                //最后一个数据需要回NOACK
    }
    else
    {
      MMA8452_SendACK(0);                //回应ACK
    }
  }
  MMA8452_Stop();                          //停止信号
    
  mmaX=(BUF[0]<<8)|BUF[1]; 
  mmaX>>=4;
  mmaY=(BUF[2]<<8)|BUF[3]; 
  mmaY>>=4;
  mmaZ=(BUF[4]<<8)|BUF[5]; 
  mmaZ>>=4;
}
