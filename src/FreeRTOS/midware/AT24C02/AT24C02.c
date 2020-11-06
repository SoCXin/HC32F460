#include"HW_I2C.h"
#include "SEGGER_RTT.h"
uint8_t txdata[10]={1,2,3,4,5,6,7,8,9,0},rxdata[10];
void TestEEPROM(void)
{
	uint8_t trytime = 0;
	uint8_t status;
//	do
//	{
//		trytime++;
//		if(trytime>3)
//		{
//			trytime = 0;
//			break;
//		}
//		status = I2C_Write_data(I2C1_UNIT,0x54,0x00,txdata,8);	
//		if(status == I2C_BADADDR)
//		{
//			break;
//		}
//		SEGGER_RTT_printf(0,"0x54 Write I2C_Status1 = %d,trytime = %d\r\n",status,trytime);	
//		Ddl_Delay1ms(10);
//	}while(status !=I2C_RET_OK);
	trytime = 0;
	do
	{
		trytime++;
		if(trytime>3)
		{
			trytime = 0;
			break;
		}
		status = I2C_Write_data(I2C1_UNIT,0x50,0x00,txdata,8);
		SEGGER_RTT_printf(0,"0x50 Write I2C_Status2 = %d,trytime = %d\r\n",status,trytime);
		Ddl_Delay1ms(10);			
	}while(status !=I2C_RET_OK);
	trytime = 0;
	do
	{
		trytime++;
		if(trytime>3)
		{
			trytime = 0;
			break;
		}
		status = I2C_Read_data(I2C1_UNIT,0x50,0x00,rxdata,8);
		SEGGER_RTT_printf(0,"0x50 Read I2C_Status3 = %d,trytime = %d\r\n",status,trytime);
		Ddl_Delay1ms(10);
	}while(status !=I2C_RET_OK);
    trytime = 0;
	do
	{
		trytime++;
		if(trytime>3)
		{
			trytime = 0;
			break;
		}
		status = I2C_Read_Buffer(I2C1_UNIT,0x50,rxdata,8);
		SEGGER_RTT_printf(0,"0x50 Read I2C_Status3 = %d,trytime = %d\r\n",status,trytime);
		Ddl_Delay1ms(10);
	}while(status !=I2C_RET_OK);
}
