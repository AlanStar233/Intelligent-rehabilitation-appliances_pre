/**************************************/
/*           WeBee�Ŷ�                */
/*           Zigbee                   */
/*�������ƣ�                          */
/*����ʱ�䣺2018/5/18                 */
/*������    MPU6050��������
**************************************/
#include <ioCC2530.h>
#include"MPU6050.h"
#include "OnBoard.h"
#include "hal_types.h"

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
void Delay_us(uint8 Time); //us��ʱ
void type_change(int num);
void WriteSDA_0(); 
void WriteSDA_1(); 
void WriteSCL_0();    
void WriteSCL_1();    
void ReadSDA(void);//sda����,=0
void Init_IO(void);
void delay_usec(unsigned char u);



/****************us��ʱ���� 32M���� ������MCU********************/
void delay_usec(uchar u)
{
  while(u --)
  {
    /* 32 NOPs == 1 usecs */
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); 
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); 
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); 
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); 
    asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); 
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); 
    asm("nop"); asm("nop"); asm("nop"); asm("nop");
  }
}

/*****************************MPU6050*******************************/
void WriteSDA_0(void) 
{
    P0DIR|=0x40;
    SDA=0;
}

void WriteSDA_1() 
{
    P0DIR|=0x40;
    SDA=1;
}
    
void WriteSCL_0()    
{
    P0DIR|=0x80;
    SCL=0;
}

void WriteSCL_1()    
{
    P0DIR|=0x80;
    SCL=1;
}

void ReadSDA(void)//sda����,=0
{
    P0DIR&=0xBF;
}

void Init_IO(void)
{
  P0DIR|=0xc0;
  P0SEL&=0x3f;
}



/**************************************
��ʱ5΢��
**************************************/
static void mma_Delay5us(void)//uint16 microSecs)
{
  MicroWait(20);
   
}
 void Delay_us(uint8 Time) //us��ʱ
{
  unsigned char i;
  for(i=0;i<Time;i++)
  {
     MicroWait(1);
  }
}/**************************************
��ʼ�ź�
**************************************/
static void MPU6050_Start()
{
    WriteSDA_1();
    WriteSCL_1();
    delay_usec(50);
    WriteSDA_0();
    delay_usec(50);
    WriteSCL_0();
    delay_usec(50);}

/**************************************
ֹͣ�ź�
**************************************/
static void MPU6050_Stop()
{
   WriteSDA_0();
   WriteSCL_1();
   delay_usec(50);
   WriteSDA_1();
}

/**************************************
����Ӧ���ź�
��ڲ���:ack (0:ACK 1:NAK)
**************************************/
static void MPU6050_SendACK(char ack)
{  
    SDA=ack;                     //дӦ���ź�
    WriteSCL_1();                   //����ʱ����
    delay_usec(50);//Delay5us();    //��ʱ
    WriteSCL_0();                   //����ʱ����
    delay_usec(50);//Delay5us();    //��ʱ
}

/**************************************
����Ӧ���ź�
**************************************/
static char MPU6050_RecvACK()
{   
    ReadSDA();
    WriteSCL_1();                       //����ʱ����
    delay_usec(50);//Delay5us();    //��ʱ
    CY=SDA;                      //��Ӧ���ź�
    WriteSCL_0();                       //����ʱ����
    delay_usec(50);//Delay5us();    //��ʱ
    P0DIR|=0x08;
    return CY;

}

/**************************************
��IIC���߷���һ���ֽ�����
**************************************/
static void MPU6050_SendByte(unsigned char dat)
{
  unsigned char i;
    P0DIR|=0x08;
    
    for(i=0;i<8;i++)         //8λ������
    {       
        if(data&0x80)
        {WriteSDA_1();}
        else 
        {WriteSDA_0();}
        
        data<<=1;
        WriteSCL_1();            //����ʱ����
        delay_usec(50);          //��ʱ
        WriteSCL_0();            //����ʱ����
        delay_usec(50);          //��ʱ
    }
//
//  for (i=0; i<8; i++)         //8λ������
//  {
//    dat <<= 1;              //�Ƴ����ݵ����λ
//    SDA = CY;               //�����ݿ�
//    SCL = 1;                //����ʱ����
//    Delay_us(5)  ;
//    SCL = 0;                //����ʱ����
//    Delay_us(5)  ;
//  }
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
    Delay_us(5)  ;
    SDA_IN();
    dat |= SDA;             //������  
    SDA_OUT();
    SCL = 0;                //����ʱ����
    Delay_us(5)  ;
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
//  P0SEL &= ~0xc0; //��Ϊ��ͨ IO ��
//  SDA_OUT();
//  SCL_OUT(); 
  Single_Write_MPU6050(PWR_MGMT_1, 0x00);//��Դ��������ֵ��0x00(��������)
  Single_Write_MPU6050(SMPLRT_DIV, 0x07);
  Single_Write_MPU6050(CONFIG, 0x06);
  Single_Write_MPU6050(GYRO_CONFIG, 0x18);
  Single_Write_MPU6050(ACCEL_CONFIG, 0x01); 
}
//*********************************************************
//
//��������MPU6050�ڲ����ٶ�����
//
//*********************************************************
void Multiple_read_MPU6050(void)
{   
  char BUF[12]; //�������ݻ�����  
  BUF[0]=Single_Read_MPU6050(0x3B);
  BUF[1]=Single_Read_MPU6050(0x3C);
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
   
 // HalUARTWrite(0,"the data is ",12);
  type_change(accX);
  HalUARTWrite(0,"  ",2);
  type_change(accY);
  HalUARTWrite(0,"  ",2);
  type_change(accZ);
  HalUARTWrite(0,"  ",2);

  
  
  type_change(graX);
  HalUARTWrite(0,"  ",2);
  type_change(graY);
  HalUARTWrite(0,"  ",2);
  type_change(graZ);
  HalUARTWrite(0,"\n",1);
}

void type_change(int num)
{
  uint8 temp[5];
  if(num<0)
  {
    HalUARTWrite(0,"-",1);
    num = -num;
  }
  else
  {
    HalUARTWrite(0,"+",1);
  }
  temp[0] = num/10000+0x30;
  temp[1] = num%10000/1000+0x30;
  temp[2] = num%1000/100+0x30;
  temp[3] = num%100/10+0x30;
  temp[4] = num%10+0x30;
  HalUARTWrite(0,&temp[0],5);
}
