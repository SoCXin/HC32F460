/*
 * OLED_96.c
 *
 * Created: 2015/1/7 16:15:04
 *  Author: dell1
 */ 
#include "OLED_96.h"
#include "OLEDfont.h"
#include "User_SPI.h"
#define delay_ms Ddl_Delay1ms
void OLEDIO_init(void)
{
	stc_port_init_t Port_CFG;
	MEM_ZERO_STRUCT(Port_CFG);
	Port_CFG.enPinMode = Pin_Mode_Out;
	PORT_Init(OLED_RST_PORT, OLED_RST_PIN, &Port_CFG);
	PORT_Init(OLED_DC_PORT, OLED_DC_PIN, &Port_CFG);
	User_SPI_Init();
}
void OLED_RST_SET(bool value)
{
	if(value)
	{
		PORT_SetBits(OLED_RST_PORT,OLED_RST_PIN);
	}
	else
	{
		PORT_ResetBits(OLED_RST_PORT,OLED_RST_PIN);
	}
}
void OLED_DC_SET(bool value)
{
	{
	if(value)
	{
		PORT_SetBits(OLED_DC_PORT,OLED_DC_PIN);
	}
	else
	{
		PORT_ResetBits(OLED_DC_PORT,OLED_DC_PIN);
	}
}
}
void OLED_WR_Byte(uint8_t dat,uint8_t cmd)
{
//	uint8_t i;
	if (cmd)
	{
		OLED_DC_SET(1);
	 } 
	else
	{
		OLED_DC_SET(0);
	 } 
//	
//	OLED_CS_SET(0);
//	for (i = 0;i < SPI_DataLength;i++)
//	{
//		OLED_SCL_SET(0);
//		if (dat&0x80)
//		{
//			OLED_SDA_SET(1);
//		 } 
//		else
//		{
//			OLED_SDA_SET(0);
//		 }
//	   OLED_SCL_SET(1);
//	   dat<<=1;
//	}
//	OLED_CS_SET(1);
	SPI_Writedata(dat);
	OLED_DC_SET(1);
}


void OLED_Display_On(void)
{
	OLED_WR_Byte(0x8D,OLED_CMD);  //SET DC命令
	OLED_WR_Byte(0x14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0xAF,OLED_CMD);  //DISPLAY ON
}

void OLED_Display_Off(void)
{
	OLED_WR_Byte(0x8D,OLED_CMD);  //SET DC命令
	OLED_WR_Byte(0x10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0xAE,OLED_CMD);  //DISPLAY OFF
}

void OLED_Init(void)
{
	OLEDIO_init();
	OLED_RST_SET(1);
	delay_ms(100);
	OLED_RST_SET(0);
	delay_ms(100);
	OLED_RST_SET(1);
	OLED_WR_Byte(0xAE,OLED_CMD);//--turn off oled panel
	OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
	OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
	OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	OLED_WR_Byte(0x81,OLED_CMD);//--set contrast control register
	OLED_WR_Byte(0xCF,OLED_CMD); // Set SEG Output Current Brightness
	OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0×ó??·??? 0xa1????
	OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0????·??? 0xc8????
	OLED_WR_Byte(0xA6,OLED_CMD);//--set normal display
	OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
	OLED_WR_Byte(0x3f,OLED_CMD);//--1/64 duty
	OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	OLED_WR_Byte(0x00,OLED_CMD);//-not offset
	OLED_WR_Byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
	OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
	OLED_WR_Byte(0xD9,OLED_CMD);//--set pre-charge period
	OLED_WR_Byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	OLED_WR_Byte(0xDA,OLED_CMD);//--set com pins hardware configuration
	OLED_WR_Byte(0x12,OLED_CMD);
	OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh
	OLED_WR_Byte(0x40,OLED_CMD);//Set VCOM Deselect Level
	OLED_WR_Byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
	OLED_WR_Byte(0x02,OLED_CMD);//
	OLED_WR_Byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
	OLED_WR_Byte(0x14,OLED_CMD);//--set(0x10) disable
	OLED_WR_Byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
	OLED_WR_Byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7)
	OLED_WR_Byte(0xAF,OLED_CMD);//--turn on oled panel
	
	OLED_WR_Byte(0xAF,OLED_CMD); /*display ON*/
	OLED_Clear();
	OLED_Set_Pos(0,0);
}

void OLED_Clear(void)
{
	uint8_t i,n;
	for(i=0;i<8;i++)
	{
		OLED_WR_Byte (0xb0+i,OLED_CMD);//设置页地址(0~7)
		OLED_WR_Byte (0x00,OLED_CMD);  //设置显示位置，列地址
		OLED_WR_Byte (0x10,OLED_CMD);  //设置显示位置，行地址
		for(n=0;n<128;n++)
		 OLED_WR_Byte(0,OLED_DATA);//更新显示
	}
}

void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t)
{
	
}
/************************************************************************/
//在指定位置显示一个字符
//x:0~127
//y:0~63
//char:待显示字符
/************************************************************************/
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr)
{
	unsigned char c = 0,i = 0;
	c = chr - ' ';
	if(x>Max_Column-1)
	{
		x = 0;
		y = y + 2;
	}
	if(SIZE ==16)
	{
		OLED_Set_Pos(x,y);
		for(i=0;i<8;i++)
		OLED_WR_Byte(F8X16[c*16+i],OLED_DATA);
		OLED_Set_Pos(x,y+1);
		for(i=0;i<8;i++)
		OLED_WR_Byte(F8X16[c*16+i+8],OLED_DATA);
	}
	else 
	{
		OLED_Set_Pos(x,y+1);
		for(i=0;i<6;i++)
		OLED_WR_Byte(F6x8[c][i],OLED_DATA);
	}
}

/************************************************************************/
/*                        指数函数(m^n)                                 */
/************************************************************************/
uint32_t oled_pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;
	while(n--)
	result*=m;
	return result;
}
/************************************************************************/
/*                        显示数字                                     */
//x,y：起点坐标
//len:数字的位数
//size：字体大小
//num:数值(0~4294967295)
/************************************************************************/
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size)
{
	uint8_t t,temp;
	uint8_t enshow=0;
	for(t=0;t< len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0 && t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ');
				continue;
			}
			else 
			enshow=1;
		}
		OLED_ShowChar(x+(size/2)*t,y,temp+'0');
	}
}

/************************************************************************/
/*                         显示字符串                                   */
/*arg:x,汉字显示行位置；y,汉字显示列位置；*p,为字符串*/
/************************************************************************/
void OLED_ShowString(uint8_t x,uint8_t y, uint8_t *p)
{
	unsigned char i = 0;
	while(p[i] != '\0')
	{
		OLED_ShowChar(x,y,p[i]);
		x+= 8;
		if (x >120)
		{
			x = 0;
			y+= 2;
		}
		i++;
	}
}
/************************************************************************/
/*                                                                      */
/************************************************************************/
void OLED_Set_Pos(unsigned char x, unsigned char y)
{
	OLED_WR_Byte(0xB0+y,OLED_CMD);
	OLED_WR_Byte(((x&0xF0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte((x&0x0F)|0x01,OLED_CMD);
}
/************************************************************************/
/*                            显示汉字                                  */
/*arg:x,汉字显示行位置；y,汉字显示列位置*/
/*no:汉字在Hzk数组中的行*/
/************************************************************************/

void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no)
{
	uint8_t t,adder=0;
	OLED_Set_Pos(x,y);
	for(t=0;t<16;t++)
	{
		OLED_WR_Byte(Hzk[2*no][t],OLED_DATA);
		adder+=1;
	}
	OLED_Set_Pos(x,y+1);
	for(t=0;t<16;t++)
	{
		OLED_WR_Byte(Hzk[2*no+1][t],OLED_DATA);
		adder+=1;
	}
}
/************************************************************************/
/* 功能描述：显示BMP图片                                                */
//x0,y0:左上角起始坐标
//x1,y1:右下角终点坐标
//BPM[]:是图片存储数组
/************************************************************************/
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
{
	unsigned int j=0;
	unsigned char x,y;
	
	if(y1%8==0) 
	  y=y1/8;
	else 
	  y=y1/8+1;
	for(y=y0;y<y1;y++)
	{
		OLED_Set_Pos(x0,y);
		for(x = x0;x < x1;x++)
		{
			OLED_WR_Byte(BMP[j++],OLED_DATA);
		}
	}
}

//void OLEDIO_init(void)
//{
//	struct port_config config_port_pin;
//	port_get_config_defaults(&config_port_pin);
//	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
//	port_pin_set_config(OLED_CS,&config_port_pin);
//	port_pin_set_config(OLED_SDA,&config_port_pin);
//	port_pin_set_config(OLED_SCL,&config_port_pin);
//	port_pin_set_config(OLED_RST,&config_port_pin);
//	port_pin_set_config(OLED_DC,&config_port_pin);
//}
void OLED_showFloat(uint8_t x,uint8_t y,float fnum)
{
	uint32_t temp;
	temp = (uint32_t)(fnum*10);
	if ((temp/1000)%10)
	{
		OLED_ShowNum(x,y,(temp/1000)%10,1,16);
		OLED_ShowNum(x+8,y,(temp/100)%10,1,16);
	}
	else
	{
		OLED_ShowChar(x,y,' ');
		if ((temp/100)%10)
		{
			OLED_ShowNum(x+8,y,(temp/100)%10,1,16);
		}
		else
		{
			OLED_ShowChar(x+8,y,' ');
		}
	}
	OLED_ShowNum(x+16,y,(temp/10)%10,1,16);//个位
	OLED_ShowChar(x+24,y,'.');
	OLED_ShowNum(x+32,y,temp%10,1,16);
}

void OLED_ShowInt(uint8_t x,uint8_t y,uint8_t num)
{
// 	uint8_t temp;
// 	temp = num;
	if ((num/100)%10)
	{
		OLED_ShowNum(x,y,(num/100)%10,1,16);
		OLED_ShowNum(x+8,y,(num/10)%10,1,16);
	}
	else
	{
		OLED_ShowChar(x,y,' ');
		if ((num/10)%10)
		{
			OLED_ShowNum(x+8,y,(num/10)%10,1,16);
		}
		else
		{
			OLED_ShowChar(x+8,y,' ');
		}
	}
	OLED_ShowNum(x+16,y,num%10,1,16);
}
void OLED_ShowInitial(void)
{
	OLED_Init();
//	OLED_ShowString(0,0,"TA:");
//	OLED_ShowString(32,0,"Set");
//	OLED_ShowString(64,0,"Current");
//	OLED_ShowString(0,4,"TB:");
//	OLED_ShowString(32,4,"Set");
//	OLED_ShowString(64,4,"Current");
}