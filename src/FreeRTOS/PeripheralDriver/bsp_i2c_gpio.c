/*-----------------------------------------------------------------------------------------------
 *-----------------------------------------------------------------------------------------------
 *
 *									华大半导体有限公司HDSC
 *							          
 *
 *-----------------------------------------------------------------------------------------------

 *-----------------------------------------------------------------------------------------------
 * 修 改 人:欧阳健
 * 版	 本:
 * 描	 述:增加AT24C02EEPROM读写
 * 芯片平台： HC32F460
 *-----------------------------------------------------------------------------------------------
 *-----------------------------------------------------------------------------------------------
 */ 

#include "bsp_i2c_gpio.h"

volatile unsigned char eedata[5];
volatile unsigned char eedata_write[5];
volatile unsigned char I2C_Read_Data[2];
/******************************************************************************
函数：Set_SCL_DIR()
功能：设置SCL为输出
说明：
******************************************************************************/
void Set_SCL_DIR(void)
{
    PORT_Unlock();
	SCL_DIR = 1;//时钟线设置为输出
    PORT_Lock();
}

/******************************************************************************
函数：Set_SDA_DIR()
功能：设置SDA端口输方向
说明：1为输出，0为输入
******************************************************************************/
void Set_SDA_DIR(unsigned char dir)
{
    PORT_Unlock();
	if(dir)
		{
			SDA_DIR = 1;//DDRX |= 0xXX;//数据线设置为输出
		}
	else
		{
			SDA_DIR = 0;//DDRX &= ~0xXX;//数据线设置为输入
		}
	PORT_Lock();
}
/******************************************************************************
函数：Set_SCL_pin()
功能：设置SDA端口输出数据
说明：
******************************************************************************/
void Set_SCL_pin(bool value)
{
    SCL_Pin_out = value;
//	if(value)
//		{
//			PORTA |= (0x01<<4);//PORTX |= 0xXX;//数据线设置为输出
//		}
//	else
//		{
//			PORTA &= ~(0x01<<4);//PORTX &= ~0xXX;//数据线设置为输入
//		}
	
}
/******************************************************************************
函数：Set_SDA_pin()
功能：设置SDA端口输出数据
说明：
******************************************************************************/
void Set_SDA_pin(bool value)
{
    SDA_Pin_out = value;
//	if(value)
//		{
//			PORTA |= (0x01<<6);//PORTX |= 0xXX;//数据线设置为输出
//		}
//	else
//		{
//			PORTA &= ~(0x01<<6);//PORTX &= ~0xXX;//数据线设置为输入
//		}
	
}
/******************************************************************************
函数：Get_SDA_pin()
功能：设置SDA端口输出数据
说明：
******************************************************************************/
unsigned char Get_SDA_pin(void)
{
	if(SDA_Pin_in)
	{
		return 1;
	}
	else
	{
		return 0;
	}		
}
/******************************************************************************
函数：NOP10_Delay()
功能：延时
说明：
******************************************************************************/
void NOP10_Delay(void)
{
    Ddl_Delay1us(2);
//	asm("Nop");asm("Nop");asm("Nop");asm("Nop");asm("Nop");asm("Nop");
//	asm("Nop");asm("Nop");asm("Nop");asm("Nop");asm("Nop");asm("Nop");
	//asm("Nop");asm("Nop");asm("Nop");asm("Nop");asm("Nop");asm("Nop");
	//asm("Nop");asm("Nop");asm("Nop");asm("Nop");asm("Nop");asm("Nop");
}

/******************************************************************************
函数：Gpio_I2C_Init()
功能：I2C总线初始化，使总线处于空闲状态
说明：使用模拟I2C指令前，应当执行一次本函数
******************************************************************************/
void Gpio_I2C_Init(void)
{
     Set_SCL_DIR();
     Set_SDA_DIR(1);

     Set_SCL_pin(1);
     NOP10_Delay ();//6us;
     Set_SDA_pin(1);
     NOP10_Delay ();//6us;
}
/******************************************************************************
函数：I2C_Start()
功能：产生I2C总线的起始条件
说明：SCL处于高电平期间，当SDA出现下降沿时启动I2C总线
       本函数也用来产生重复起始条件
******************************************************************************/
void I2C_Start(void)
{

     Set_SCL_DIR(); //输出
     Set_SDA_DIR(1); //输出

     Set_SDA_pin(1);     NOP10_Delay ();//6us;
     Set_SCL_pin(1);     NOP10_Delay ();//6us;
     Set_SDA_pin(0);     NOP10_Delay ();//6us;
     Set_SCL_pin(0);     NOP10_Delay ();//6us;
}
///******************************************************************************
//函数：void I2C_Write(unsigned char dat)
//功能：向I2C总线写1个字节的数据
//参数：dat是要写到总线上的数据 
//******************************************************************************/
unsigned char I2C_Write(unsigned char dat)
{
	unsigned char  Ack;
    unsigned char b = 8;
    volatile unsigned char i2c_fault;
      Set_SDA_DIR(1);//输出
     do
     {
         if(dat & 0x80) Set_SDA_pin(1);
         else   Set_SDA_pin(0);     
         dat = dat<<1;
         NOP10_Delay ();//6us;
         Set_SCL_pin(1);     NOP10_Delay ();//6us;
         Set_SCL_pin(0);     NOP10_Delay ();//6us;
     } while( --b != 0 );
     Ack = I2C_GetAck();
	 if(Ack==1)
	 {
		 i2c_fault =1;//NAK
	 }
	 else i2c_fault =0;
     return i2c_fault;
}
///******************************************************************************
//函数：unsigned char I2C_Read()
//功能：从机读取1个字节的数据
//返回：读取的1个字节数据
//******************************************************************************/
unsigned char I2C_Read(unsigned char  Ack)
{
     unsigned char dat = 0;
     unsigned char t = 8;
	 unsigned char Bits;
     Set_SDA_DIR(1);
	 Set_SDA_pin(1);
     Set_SDA_DIR(0); //在读取数据之前，要把SDA拉高，使之处于输入状态

     do
     {
//	     SdaPin = 1;
         Set_SCL_pin(1);    NOP10_Delay ();//6us;
         dat <<= 1;
		 Bits = Get_SDA_pin();NOP10_Delay ();//6us;
         if (Bits) dat++;
         Set_SCL_pin(0);     NOP10_Delay ();//6us;
     } while ( --t != 0 );
  
 //#########主机应答从机##################

     Set_SDA_DIR(1); //输出

     if(Ack)
		{ Set_SDA_pin(0);}
     else   
        {Set_SDA_pin(1);}
     NOP10_Delay ();//6us;
     Set_SCL_pin(1);     NOP10_Delay ();//6us;
     Set_SCL_pin(0);     NOP10_Delay ();//6us;
 //#######################################/
     return dat;
}
///******************************************************************************
//函数：unsigned char  I2C_GetAck()
//功能：读取从机应答位（应答或非应答），用于判断：从机是否成功接收主机数据
//返回：0－从机应答
//       1－从机非应答
//说明：从机在收到每一个字节后都要产生应答位，主机如果收到非应答则应当终止传输
//******************************************************************************/
unsigned char  I2C_GetAck(void)
{
     unsigned char Ack;
    
     Set_SDA_DIR(0); //输入
	 Set_SDA_pin(1);//上拉
     NOP10_Delay ();//6us;
     Set_SCL_pin(1);     NOP10_Delay ();//6us;
     Ack = Get_SDA_pin();NOP10_Delay ();
     Set_SCL_pin(0);     NOP10_Delay ();//6us;

     Set_SDA_DIR(1); //输出
     return Ack;
}
/******************************************************************************
函数：void I2C_PutAck(unsigned char  Ack)
功能：主机产生应答位（应答或非应答），用于通知从机：主机是否成功接收从机数据
参数：Ack = 0：主机应答
       Ack = 1：主机非应答
说明：主机在收到每一个字节后都要产生应答，在收到最后一个字节时，应当产生非应答
******************************************************************************/
void I2C_PutAck(unsigned char  Ack)
{
     
     Set_SDA_DIR(1); //输出
 
     if(Ack) Set_SDA_pin(1);
     else   Set_SDA_pin(0);
                      NOP10_Delay ();//6us;
     Set_SCL_pin(1);  NOP10_Delay ();//6us;
     Set_SCL_pin(0);  NOP10_Delay ();//6us;
}
/******************************************************************************
函数：I2C_Stop()
功能：产生I2C总线的停止条件
说明：SCL处于高电平期间，当SDA出现上升沿时停止I2C总线
******************************************************************************/
void I2C_Stop(void)
{
     Set_SDA_pin(0);     NOP10_Delay ();//6us;
     Set_SCL_pin(1);     NOP10_Delay ();//6us;
     Set_SDA_pin(1);     NOP10_Delay ();//6us;
}

/******************************************************************************
函数：I2C_Send_Command(uint8_t DeviceAddr, uint8_t address, uint8_t *data, uint8_t len)
功能：I2C发送
说明：
******************************************************************************/
unsigned char I2C_Send_Command(uint8_t DeviceAddr, uint8_t address, uint8_t *data, uint8_t len)
{
	unsigned char count;
	volatile unsigned char i2c_fault;
	I2C_Start();
	i2c_fault = I2C_Write(DeviceAddr<<1);
	if(i2c_fault)
    {
        I2C_Stop();
        return 1;
    }
//	i2c_fault = I2C_Write(address>>8);
//    if(i2c_fault)
//    {
//        I2C_Stop();
//        return 1;
//    }
	i2c_fault = I2C_Write(address);
    if(i2c_fault)
    {
        I2C_Stop();
        return 1;
    }
	
	//	I2C_master_write(0x03);
	for(count=0;count<len;count++)
	{
		i2c_fault = I2C_Write(data[count]);
        if(i2c_fault)
        {
            I2C_Stop();
            return 1;
        }
	}
	I2C_Stop();
	return 0;

}
/******************************************************************************
函数：I2C_Read_Command(uint8_t DeviceAddr, uint8_t address, uint8_t *rxbuf, uint8_t len)
功能：I2C发送
说明：
******************************************************************************/
unsigned char I2C_Read_Command(uint8_t DeviceAddr, uint8_t address, uint8_t *rxbuf, uint8_t len)
{
    unsigned char count;
    volatile unsigned char i2c_fault;
	I2C_Start();
	
	i2c_fault = I2C_Write(DeviceAddr<<1);
	if(i2c_fault)
    {
        I2C_Stop();
        return 1;
    }
//	i2c_fault = I2C_Write(address>>8);
//    if(i2c_fault)
//    {
//        I2C_Stop();
//        return 1;
//    }
	i2c_fault = I2C_Write(address);
	if(i2c_fault)
    {
        I2C_Stop();
        return 1;
    }
	I2C_Start();
	
	i2c_fault = I2C_Write((DeviceAddr<<1)|0x01);
	if(i2c_fault)
    {
        I2C_Stop();
        return 1;
    }	
	for(count=0;count<(len-1);count++)
	{
		rxbuf[count]=I2C_Read(1);
	}
	rxbuf[count]=I2C_Read(0);	// "0" is no ack. "1" is ack
	
	I2C_Stop();
    return 0;
}
