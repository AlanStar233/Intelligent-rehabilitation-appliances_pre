/**************************************/
/*           WeBee�Ŷ�                */
/*           Zigbee                   */
/*�������ƣ�                          */
/*����ʱ�䣺2018/5/18                 */
/*������    MPU6050��������
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
��ʱ5΢��
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
��ʼ�ź�
**************************************/
static void MPU6050_Start()
{
  SDA = 1;                    //����������
  SCL = 1;                    //����ʱ����
  mma_Delay5us();                 
  SDA = 0;                    //�����½���
  mma_Delay5us();             //��ʱ
  SCL = 0;                    //����ʱ����
  mma_Delay5us();         
}

/**************************************
ֹͣ�ź�
**************************************/
static void MPU6050_Stop()
{
  SDA = 0;                    //����������
  SCL = 1;                    //����ʱ����
  mma_Delay5us();                 //��ʱ
  SDA = 1;                    //����������
  mma_Delay5us();                 //��ʱ
}

/**************************************
����Ӧ���ź�
��ڲ���:ack (0:ACK 1:NAK)
**************************************/
static void MPU6050_SendACK(char ack)
{
  SDA = ack;                  //дӦ���ź�
  SCL = 1;                    //����ʱ����
  mma_Delay5us();                 //��ʱ
  SCL = 0;                    //����ʱ����
  mma_Delay5us();                 //��ʱ
}

/**************************************
����Ӧ���ź�
**************************************/
static char MPU6050_RecvACK()
{
  SCL = 1;                    //����ʱ����
  mma_Delay5us();            
  SDA_IN();
  CY = SDA;                   //��Ӧ���ź�
  SDA_OUT();
  SCL = 0;                    //����ʱ����
  mma_Delay5us();                 //��ʱ

  return CY;
}

/**************************************
��IIC���߷���һ���ֽ�����
**************************************/
static void MPU6050_SendByte(unsigned char dat)
{
  unsigned char i;
  
  for (i=0; i<8; i++)         //8λ������
  {
    dat <<= 1;              //�Ƴ����ݵ����λ
    SDA = CY;               //�����ݿ�
    SCL = 1;                //����ʱ����
    mma_Delay5us();             //��ʱ
    SCL = 0;                //����ʱ����
    mma_Delay5us();             //��ʱ
  }
  MPU6050_RecvACK();
}

/**************************************
��IIC���߽���һ���ֽ�����
**************************************/
static unsigned char MPU6050_RecvByte()
{
  unsigned char i;
  unsigned char dat = 0;
  
  SDA = 1;                    //ʹ���ڲ�����,׼����ȡ����,
  for (i=0; i<8; i++)         //8λ������
  {
    dat <<= 1;
    SCL = 1;                //����ʱ����
    mma_Delay5us();             //��ʱ
    SDA_IN();
    dat |= SDA;             //������  
    SDA_OUT();
    SCL = 0;                //����ʱ����
    mma_Delay5us();             //��ʱ
  }
  return dat;
}

//******���ֽ�д��*******************************************
static void Single_Write_MPU6050(unsigned char REG_Address,unsigned char REG_data)
{
  MPU6050_Start();                  //��ʼ�ź�
  MPU6050_SendByte(SlaveAddress);   //�����豸��ַ+д�ź�
  MPU6050_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ
  MPU6050_SendByte(REG_data);       //�ڲ��Ĵ�������
  MPU6050_Stop();                   //����ֹͣ�ź�
}

//********���ֽڶ�ȡ*****************************************

unsigned char Single_Read_MPU6050(unsigned char REG_Address)
{  
  unsigned char REG_data;
  MPU6050_Start();                          //��ʼ�ź�
  MPU6050_SendByte(SlaveAddress);           //�����豸��ַ+д�ź�
  MPU6050_SendByte(REG_Address);            //���ʹ洢��Ԫ��ַ����0��ʼ	
  MPU6050_Start();                          //��ʼ�ź�
  MPU6050_SendByte(SlaveAddress+1);         //�����豸��ַ+���ź�
  REG_data=MPU6050_RecvByte();              //�����Ĵ�������
  MPU6050_SendACK(1);   
  MPU6050_Stop();                           //ֹͣ�ź�
  return REG_data; 
}


//*****************************************************************
//                        ��ʼ��MPU6050
//*****************************************************************
void Init_MPU6050(void)
{
  P1SEL &= ~0xc0; //��Ϊ��ͨ IO ��
  SDA_OUT();
  SCL_OUT(); 
  Single_Write_MPU6050(PWR_MGMT_1, 0x00);//��Դ��������ֵ��0x00(��������)
  Single_Write_MPU6050(SMPLRT_DIV, 0x07);
  Single_Write_MPU6050(CONFIG, 0x06);
  Single_Write_MPU6050(GYRO_CONFIG, 0x18);
  Single_Write_MPU6050(ACCEL_CONFIG, 0x01); 
}
//*********************************************************
//
//��������MPU6050�ڲ�����
//
//*********************************************************
void Multiple_read_MPU6050(void)
{   
  char BUF[12]; //�������ݻ�����  
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
