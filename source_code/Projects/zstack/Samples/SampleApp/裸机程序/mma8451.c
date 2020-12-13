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
static void MMA8452_Start()
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
static void MMA8452_Stop()
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
static void MMA8452_SendACK(char ack)
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
static char MMA8452_RecvACK()
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
static void MMA8452_SendByte(unsigned char dat)
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
  MMA8452_RecvACK();
}

/**************************************
��IIC���߽���һ���ֽ�����
**************************************/
static unsigned char MMA8452_RecvByte()
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
static void Single_Write_MMA8452(unsigned char REG_Address,unsigned char REG_data)
{
  MMA8452_Start();                  //��ʼ�ź�
  MMA8452_SendByte(SlaveAddress);   //�����豸��ַ+д�ź�
  MMA8452_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ
  MMA8452_SendByte(REG_data);       //�ڲ��Ĵ�������
  MMA8452_Stop();                   //����ֹͣ�ź�
}

//********���ֽڶ�ȡ*****************************************

unsigned char Single_Read_MMA8452(unsigned char REG_Address)
{  
  unsigned char REG_data;
  MMA8452_Start();                          //��ʼ�ź�
  MMA8452_SendByte(SlaveAddress);           //�����豸��ַ+д�ź�
  MMA8452_SendByte(REG_Address);                   //���ʹ洢��Ԫ��ַ����0��ʼ	
  MMA8452_Start();                          //��ʼ�ź�
  MMA8452_SendByte(SlaveAddress+1);         //�����豸��ַ+���ź�
  REG_data=MMA8452_RecvByte();              //�����Ĵ�������
  MMA8452_SendACK(1);   
  MMA8452_Stop();                           //ֹͣ�ź�
  return REG_data; 
}


//*****************************************************************

//��ʼ��MMA8452��������Ҫ��ο�pdf�����޸�************************
void Init_MMA8452(void)
{
  P1SEL &= ~0xc0; //��Ϊ��ͨ IO ��
  //P1INP &= ~0Xc0; //������
  //P1INP |= 0Xc0; //������
  SDA_OUT();
  SCL_OUT();
  
  Single_Write_MMA8452(0x2A,0x01);  //����װ��
  Single_Write_MMA8452(0x2B,0x02);  //��������
}



//*********************************************************
//
//��������MMA8452�ڲ����ٶ����ݣ���ַ��Χ0x01~0x06
//
//*********************************************************
void Multiple_read_MMA8452(void)
{   
  char BUF[6];                         //�������ݻ�����  
  unsigned char i;
  MMA8452_Start();                          //��ʼ�ź�
  MMA8452_SendByte(SlaveAddress);           //�����豸��ַ+д�ź�
  MMA8452_SendByte(0x01);                   //���ʹ洢��Ԫ��ַ����0x01��ʼ	
  MMA8452_Start();                          //��ʼ�ź�
  MMA8452_SendByte(SlaveAddress+1);         //�����豸��ַ+���ź�
  for (i=0; i<6; i++)                      //������ȡ6����ַ���ݣ��洢��BUF
  {
    BUF[i] = MMA8452_RecvByte();          //BUF[0]�洢0x32��ַ�е�����
    if (i == 5)
    {
      MMA8452_SendACK(1);                //���һ��������Ҫ��NOACK
    }
    else
    {
      MMA8452_SendACK(0);                //��ӦACK
    }
  }
  MMA8452_Stop();                          //ֹͣ�ź�
    
  mmaX=(BUF[0]<<8)|BUF[1]; 
  mmaX>>=4;
  mmaY=(BUF[2]<<8)|BUF[3]; 
  mmaY>>=4;
  mmaZ=(BUF[4]<<8)|BUF[5]; 
  mmaZ>>=4;
}
