#ifndef OLED_H
#define OLED_H

//#define OLED_SPI
#define VERSION_OLED    1//0旧版，1新版DEMO
#include "hc32_ddl.h"


#ifdef OLED_SPI
#include "User_SPI.h"
#else
#include "Hw_I2C.h"
#if VERSION_OLED
#define OLED_I2C    M4_I2C2
#define OLED_SCL_PORT   PortD
#define OLED_SCL_PIN    Pin00
#define OLED_SDA_PORT   PortD
#define OLED_SDA_PIN    Pin01
#else
#define OLED_I2C    M4_I2C3
#define OLED_SCL_PORT   PortB
#define OLED_SCL_PIN    Pin06
#define OLED_SDA_PORT   PortB
#define OLED_SDA_PIN    Pin07
#endif 
#endif


#define I2C_Baudrate 400000
#define OLED_ADDRESS  0x3C//0x78
#define OLED_ADDRESS_W                    0x00
#define OLED_ADDRESS_R                    0x01

/*definition--------------------------------------------*/
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据
#define OLED_MODE 0


#ifdef OLED_SPI //SPI屏，分辩率128X64
#define OLED_DC_PORT	PortE
#define OLED_DC_Pin		Pin07
#define OLED_RST_PORT	PortE
#define OLED_RST_Pin	Pin08

#define Max_Column	128
#define Max_Row		64
#define Max_Data_Row 8
#define X_WIDTH 	Max_Column
#define Y_WIDTH 	Max_Row
#else //I2C 屏 分辩率128X32
#define Max_Column	128
#define Max_Row		32
#define Max_Data_Row 4
#define X_WIDTH 	Max_Column
#define Y_WIDTH 	Max_Row
#define AddrCMD		0x00
#define AddrData	0x40
#endif
#define SIZE        16
#define XLevelL		0x02
#define XLevelH		0x10
#define	Brightness	0xFF
#define SPI_DataLength 8


#ifdef OLED_SPI
void OLED_DC_SET(uint8_t value);
void OLED_RST_SET(uint8_t value);
void SPIWriteOneByte(uint8_t *data);
#endif
void OLED_Port_init(void);
void OLED_Init(void);
void OLED_WR_Byte(unsigned char dat,unsigned char cmd);
void OLED_ShowString(unsigned char x,unsigned char y,unsigned char *chr);
void OLED_ShowNum(unsigned char x,unsigned char y,unsigned int num,unsigned char len,unsigned char size2);
void OLED_ShowCHinese(unsigned char x,unsigned char y,unsigned char no);
void OLED_ShowChar(unsigned char x,unsigned char y,unsigned char chr);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_Clear(void);
void OLED_Display_Off(void);
void OLED_Display_On(void);
void OLED_Refresh(void);
void OLED_Set_Point(unsigned char x,unsigned char y);
void OLED_Draw_line(unsigned char x1,unsigned char y1,unsigned char x2,unsigned char y2);
void showsin(void);
void move_left(void);
void insertdisplaydata(unsigned char data);
void OLED_ShowChar2(unsigned char x,unsigned char y,unsigned char chr);
void OLED_unShowChar2(unsigned char x,unsigned char y,unsigned char chr);
void OLED_ShowString2(unsigned char x,unsigned char y,unsigned char *chr);
void TestDrawline(void);
#endif

