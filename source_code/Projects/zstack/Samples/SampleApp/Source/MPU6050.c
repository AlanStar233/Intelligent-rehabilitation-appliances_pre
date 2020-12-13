/***************************************/
/*           WeBee团队                 */
/*项目类型：  外包项目                 */
/*例程名称：  MPU6050                  */
/*建立时间：  2018/05                  */
/*描述：显示加速度计和陀螺仪的6位原始数据
****************************************/
#include "ioCC2530.h"
#include "MPU6050.h"
#include "OnBoard.h"
#include "hal_types.h"

#define uint  unsigned int
#define uchar unsigned char

int accX,accY,accZ,graX,graY,graZ;

//****************************************
//函数声明
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

/****************us延时函数 32M晶振 单周期MCU********************/
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

void ReadSDA(void)//sda输入,=0
{
    P0DIR&=0xBF;
}

void Init_IO(void)
{
  P0DIR|=0xc0;
  P0SEL&=0x3f;
}


/**************************************
起始信号
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
停止信号
**************************************/
void MPU6050_Stop()  
{    
   WriteSDA_0();
   WriteSCL_1();
   delay_us(5);
   WriteSDA_1();
}

//I2C发送应答信号
//入口参数:ack (0:ACK 1:NAK)
void MPU6050_SendACK(uchar ack)
{
    
    SDA=ack;                     //写应答信号
    WriteSCL_1();                   //拉高时钟线
    delay_us(5);//Delay5us();    //延时
    WriteSCL_0();                   //拉低时钟线
    delay_us(5);//Delay5us();    //延时
}

/**************************************
发送应答信号
**************************************/
uchar MPU6050_RecvACK()
{
    ReadSDA();
    WriteSCL_1();                       //拉高时钟线
    delay_us(5);//Delay5us();    //延时
    CY=SDA;                      //读应答信号
    WriteSCL_0();                       //拉低时钟线
    delay_us(5);//Delay5us();    //延时
    P0DIR|=0x00;
    return CY;
}

/**************************************
向IIC总线发送一个字节数据
**************************************/
void MPU6050_SendByte(uchar data)
{
    uchar i;
    SDA_OUT();
    
    for(i=0;i<8;i++)         //8位计数器
    {       
        if(data&0x80)
        {WriteSDA_1();}
        else 
        {WriteSDA_0();}
        
        data<<=1;
        WriteSCL_1();            //拉高时钟线
        delay_us(5);          //延时
        WriteSCL_0();            //拉低时钟线
        delay_us(5);          //延时
    }
    MPU6050_RecvACK();
}

/**************************************
从IIC总线接收一个字节数据
**************************************/
uchar MPU6050_RecvByte()
{
  
    uchar i;
    uchar dat = 0;
    WriteSDA_1();                  //使能内部上拉,准备读取数据,
    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;
        SCL = 1;                //拉高时钟线
        delay_us(5);                 //延时
        dat |= SDA;             //读数据               
        SCL = 0;                //拉低时钟线
         delay_us(5);                 //延时
    }
    return dat;

}

//******单字节写入*******************************************
void Single_Write_MPU6050(uchar REG_Address,uchar REG_data)
{
    MPU6050_Start();                  //起始信号
    MPU6050_SendByte(SlaveAddress);   //发送设备地址+写信号
    MPU6050_SendByte(REG_Address);    //内部寄存器地址，
    MPU6050_SendByte(REG_data);       //内部寄存器数据，
    MPU6050_Stop();                   //发送停止信号
}

//********单字节读取*****************************************
uchar Single_Read_MPU6050(uchar REG_Address)
{
	uchar REG_data;
	MPU6050_Start();                   //起始信号
	MPU6050_SendByte(SlaveAddress);    //发送设备地址+写信号
	MPU6050_SendByte(REG_Address);     //发送存储单元地址，从0开始	
	MPU6050_Start();                   //起始信号
	MPU6050_SendByte(SlaveAddress+1);  //发送设备地址+读信号
	REG_data=MPU6050_RecvByte();       //读出寄存器数据
	MPU6050_SendACK(1);                //接收应答信号
	MPU6050_Stop();                    //停止信号
	return REG_data;
}

//*****************************************************************

//初始化MPU6050，根据需要请参考pdf进行修改************************
void InitMPU6050()
{
	Single_Write_MPU6050(PWR_MGMT_1,0x00);	//解除休眠状态
	Single_Write_MPU6050(SMPLRT_DIV,0x07);
	Single_Write_MPU6050(CONFIG,0x06);
	Single_Write_MPU6050(GYRO_CONFIG,0x18);
	Single_Write_MPU6050(ACCEL_CONFIG,0x01);
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

