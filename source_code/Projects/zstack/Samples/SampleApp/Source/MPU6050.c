/***************************************/
/*           WeBee�Ŷ�                 */
/*��Ŀ���ͣ�  �����Ŀ                 */
/*�������ƣ�  MPU6050                  */
/*����ʱ�䣺  2018/05                  */
/*��������ʾ���ٶȼƺ������ǵ�6λԭʼ����
****************************************/
#include "ioCC2530.h"
#include "MPU6050.h"
#include "OnBoard.h"
#include "hal_types.h"

#define uint  unsigned int
#define uchar unsigned char

int accX,accY,accZ,graX,graY,graZ;

//****************************************
//��������
//****************************************
void WriteSDA_0(void);
void WriteSDA_1(void);
void WriteSCL_0(void);
void WriteSCL_1(void);
void ReadSDA(void);
void Init_IO(void);
void MPU6050_Start(void);
void MPU6050_Stop(void);
void MPU6050_SendACK(uchar ack);
uchar MPU6050_RecvACK(void);
void MPU6050_SendByte(uchar data);
uchar MPU6050_RecvByte(void);
void Single_Write_MPU6050(uchar REG_Address,uchar REG_data);
uchar Single_Read_MPU6050(uchar REG_Address);
void InitMPU6050(void);
void ALLInit(void);
void Multiple_read_MPU6050(void);
void delay_us(unsigned int u);


void ALLInit(void)
{
   Init_IO();
   InitMPU6050();
}

/****************us��ʱ���� 32M���� ������MCU********************/
void delay_us(unsigned int u)
{
   u=4*u;
   MicroWait(u);
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
��ʼ�ź�
**************************************/
void MPU6050_Start()  
{
    WriteSDA_1();
    WriteSCL_1();
    delay_us(5);
    WriteSDA_0();
    delay_us(5);
    WriteSCL_0();
    delay_us(5);
}

/**************************************
ֹͣ�ź�
**************************************/
void MPU6050_Stop()  
{    
   WriteSDA_0();
   WriteSCL_1();
   delay_us(5);
   WriteSDA_1();
}

//I2C����Ӧ���ź�
//��ڲ���:ack (0:ACK 1:NAK)
void MPU6050_SendACK(uchar ack)
{
    
    SDA=ack;                     //дӦ���ź�
    WriteSCL_1();                   //����ʱ����
    delay_us(5);//Delay5us();    //��ʱ
    WriteSCL_0();                   //����ʱ����
    delay_us(5);//Delay5us();    //��ʱ
}

/**************************************
����Ӧ���ź�
**************************************/
uchar MPU6050_RecvACK()
{
    ReadSDA();
    WriteSCL_1();                       //����ʱ����
    delay_us(5);//Delay5us();    //��ʱ
    CY=SDA;                      //��Ӧ���ź�
    WriteSCL_0();                       //����ʱ����
    delay_us(5);//Delay5us();    //��ʱ
    P0DIR|=0x00;
    return CY;
}

/**************************************
��IIC���߷���һ���ֽ�����
**************************************/
void MPU6050_SendByte(uchar data)
{
    uchar i;
    SDA_OUT();
    
    for(i=0;i<8;i++)         //8λ������
    {       
        if(data&0x80)
        {WriteSDA_1();}
        else 
        {WriteSDA_0();}
        
        data<<=1;
        WriteSCL_1();            //����ʱ����
        delay_us(5);          //��ʱ
        WriteSCL_0();            //����ʱ����
        delay_us(5);          //��ʱ
    }
    MPU6050_RecvACK();
}

/**************************************
��IIC���߽���һ���ֽ�����
**************************************/
uchar MPU6050_RecvByte()
{
  
    uchar i;
    uchar dat = 0;
    WriteSDA_1();                  //ʹ���ڲ�����,׼����ȡ����,
    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;
        SCL = 1;                //����ʱ����
        delay_us(5);                 //��ʱ
        dat |= SDA;             //������               
        SCL = 0;                //����ʱ����
         delay_us(5);                 //��ʱ
    }
    return dat;

}

//******���ֽ�д��*******************************************
void Single_Write_MPU6050(uchar REG_Address,uchar REG_data)
{
    MPU6050_Start();                  //��ʼ�ź�
    MPU6050_SendByte(SlaveAddress);   //�����豸��ַ+д�ź�
    MPU6050_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ��
    MPU6050_SendByte(REG_data);       //�ڲ��Ĵ������ݣ�
    MPU6050_Stop();                   //����ֹͣ�ź�
}

//********���ֽڶ�ȡ*****************************************
uchar Single_Read_MPU6050(uchar REG_Address)
{
	uchar REG_data;
	MPU6050_Start();                   //��ʼ�ź�
	MPU6050_SendByte(SlaveAddress);    //�����豸��ַ+д�ź�
	MPU6050_SendByte(REG_Address);     //���ʹ洢��Ԫ��ַ����0��ʼ	
	MPU6050_Start();                   //��ʼ�ź�
	MPU6050_SendByte(SlaveAddress+1);  //�����豸��ַ+���ź�
	REG_data=MPU6050_RecvByte();       //�����Ĵ�������
	MPU6050_SendACK(1);                //����Ӧ���ź�
	MPU6050_Stop();                    //ֹͣ�ź�
	return REG_data;
}

//*****************************************************************

//��ʼ��MPU6050��������Ҫ��ο�pdf�����޸�************************
void InitMPU6050()
{
	Single_Write_MPU6050(PWR_MGMT_1,0x00);	//�������״̬
	Single_Write_MPU6050(SMPLRT_DIV,0x07);
	Single_Write_MPU6050(CONFIG,0x06);
	Single_Write_MPU6050(GYRO_CONFIG,0x18);
	Single_Write_MPU6050(ACCEL_CONFIG,0x01);
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

