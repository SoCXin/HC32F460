#include "HW_I2C.h"
//#define AUTOACK
void HW_I2C_Port_Init(void)
{
	PORT_SetFunc(I2C1_SCL_PORT, I2C1_SCL_Pin, Func_I2c1_Scl, Disable);
    PORT_SetFunc(I2C1_SDA_PORT, I2C1_SDA_Pin, Func_I2c1_Sda, Disable);
}
uint8_t  HW_I2C_Init(M4_I2C_TypeDef* pstcI2Cx,uint32_t baudrate)
{
	stc_clk_freq_t freq_clk;
	stc_i2c_init_t stcI2cInit;	
	CLK_GetClockFreq(&freq_clk);	
	if(pstcI2Cx == I2C1_UNIT)
	{
		PWC_Fcg1PeriphClockCmd(I2C1_CLK,Enable);
	}
	else if(pstcI2Cx == I2C2_UNIT)
	{
		PWC_Fcg1PeriphClockCmd(I2C2_CLK,Enable);
	}
	else if(pstcI2Cx == I2C3_UNIT)
	{
		PWC_Fcg1PeriphClockCmd(I2C3_CLK,Enable);
	}
	else
	{
		return 1;
	}
    I2C_DeInit(pstcI2Cx);
    
    MEM_ZERO_STRUCT(stcI2cInit);
    stcI2cInit.enI2cMode = I2cMaster;
    stcI2cInit.u32Baudrate = baudrate;
	stcI2cInit.u32Pclk3 = freq_clk.pclk3Freq;
    I2C_Init(pstcI2Cx, &stcI2cInit);
    
    I2C_Cmd(pstcI2Cx, Enable);
	return 0;
}
inline uint8_t I2C_Write_data(M4_I2C_TypeDef* pstcI2Cx,uint8_t DeviceAddr,uint8_t addr, const uint8_t *data, uint16_t len)
{
	uint32_t u32TimeOut;
	uint8_t pos;
	if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_NACKDETECTF))//启动I2C前先清除上一次读写失败的NAK标志，否则无法写数据
	{
		I2C_ClearStatus(pstcI2Cx,I2C_CLR_NACKFCLR);
	}
	if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
	{
		if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_ARLOF))//仲裁失败
		{
//			I2C_ClearStatus(I2C_CH, I2C_CLR_ARLOFCLR);
			pstcI2Cx->CLR = 0xFFFF;
			I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
		}
		return I2C_BUSY;
	}
	I2C_GenerateStart(pstcI2Cx , Enable);
	u32TimeOut = TIMEOUT;
	while(Reset == I2C_GetStatus(pstcI2Cx, I2C_SR_TEMPTYF))//数据寄存器为空
	{
		if(0==u32TimeOut--)
		{
			if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
			{
				I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
			}			
			return I2C_TIMEROUT;
		}
	}
	I2C_SendData(pstcI2Cx, DeviceAddr<<1);
	u32TimeOut = TIMEOUT;
	while(Reset == I2C_GetStatus(pstcI2Cx, I2C_SR_TENDF))//数据发送完成
	{
		if(0==(u32TimeOut--))
		{
			if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
			{
				I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
			}			
			return I2C_TIMEROUT;
		}
	}
	u32TimeOut = TIMEOUT;
	while(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_NACKDETECTF))//等待应答
	{
		if(0 == (u32TimeOut--)) 
		{
			if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
			{
				I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
			}			
			return I2C_BADADDR;
		}
	}
	I2C_SendData(pstcI2Cx, addr);
	u32TimeOut = TIMEOUT;
	while(Reset == I2C_GetStatus(pstcI2Cx, I2C_SR_TENDF))//数据发送完成
	{
		if(0==(u32TimeOut--))
		{
			if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
			{
				I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
			}			
			return I2C_TIMEROUT;
		}
	}
	u32TimeOut = TIMEOUT;
	while(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_NACKDETECTF))//等待应答
	{
		if(0 == (u32TimeOut--)) 
		{
			if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
			{
				I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
			}			
			return I2C_TIMEROUT;
		}
	}
	for(pos = 0;pos<len;pos++)
	{
		I2C_SendData(pstcI2Cx, data[pos]);
		u32TimeOut = TIMEOUT;
		while(Reset == I2C_GetStatus(pstcI2Cx, I2C_SR_TENDF))//数据寄存器为空
		{
			if(0==(u32TimeOut--))
			{
				if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
				{
					I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
				}			
				return I2C_TIMEROUT;
			}
		}
		u32TimeOut = TIMEOUT;
		while(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_NACKDETECTF))//等待应答
		{
			if(0 == (u32TimeOut--)) 
			{
				if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
				{
					I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
				}			
				return I2C_TIMEROUT;
			}
		}		
	}
	u32TimeOut = TIMEOUT;
	do{
		I2C_GenerateStop(pstcI2Cx, Enable);//产生停止位
		if(0 == (u32TimeOut--)) 
			{
				return I2C_TIMEROUT;
			}
	}while(Reset == I2C_GetStatus(pstcI2Cx, I2C_SR_STOPF));//等待停止
	return I2C_RET_OK;
}

inline uint8_t I2C_Read_data(M4_I2C_TypeDef* pstcI2Cx,uint8_t DeviceAddr,uint8_t addr, uint8_t *data, uint16_t len)
{
	uint32_t u32TimeOut;
	uint8_t pos;
#ifndef AUTOACK	
	pstcI2Cx->CR3_f.FACKEN = 1;//非自动写ACK
#endif
	if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_NACKDETECTF))//启动I2C前先清除上一次读写失败的NAK标志，否则无法写数据
	{
		I2C_ClearStatus(pstcI2Cx,I2C_CLR_NACKFCLR);
	}
	if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
	{
		return I2C_BUSY;
	}
	I2C_GenerateStart(pstcI2Cx , Enable);
	u32TimeOut = TIMEOUT;
	while(Reset == I2C_GetStatus(pstcI2Cx, I2C_SR_TEMPTYF))//数据寄存器为空
	{
		if(0==u32TimeOut--)
		{
			if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
			{
				I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
			}			
			return I2C_TIMEROUT;
		}
	}
	I2C_SendData(pstcI2Cx, DeviceAddr<<1);
	u32TimeOut = TIMEOUT;
	while(Reset == I2C_GetStatus(pstcI2Cx, I2C_SR_TENDF))//数据发送完成
	{
		if(0==(u32TimeOut--))
		{
			if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
			{
				I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
			}			
			return I2C_TIMEROUT;
		}
	}
	u32TimeOut = TIMEOUT;
	while(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_NACKDETECTF))//等待应答
	{
		if(0 == (u32TimeOut--)) 
		{
			if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
			{
				I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
			}			
			return I2C_BADADDR;
		}
	}
	I2C_SendData(pstcI2Cx, addr);
	u32TimeOut = TIMEOUT;
	while(Reset == I2C_GetStatus(pstcI2Cx, I2C_SR_TENDF))//数据发送完成
	{
		if(0==(u32TimeOut--))
		{
			if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
			{
				I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
			}			
			return I2C_TIMEROUT;
		}
	}
	u32TimeOut = TIMEOUT;
	while(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_NACKDETECTF))//等待应答
	{
		if(0 == (u32TimeOut--)) 
		{
			if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
			{
				I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
			}			
			return I2C_TIMEROUT;
		}
	}
	I2C_ClearStatus(pstcI2Cx, I2C_CLR_STARTFCLR);
	I2C_GenerateReStart(pstcI2Cx , Enable);//发送Restart;
	u32TimeOut = TIMEOUT;
	while((Reset == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY)) ||
            (Reset == I2C_GetStatus(pstcI2Cx, I2C_SR_STARTF)))
    {
        if(0 == (u32TimeOut--)) 
		{
			I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
			return I2C_RET_ERROR;
		}
    }
	u32TimeOut = TIMEOUT;
	while(Reset == I2C_GetStatus(pstcI2Cx, I2C_SR_TEMPTYF))//数据寄存器为空
	{
		if(0==u32TimeOut--)
		{
			if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
			{
				I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
			}			
			return I2C_TIMEROUT;
		}
	}
	I2C_SendData(pstcI2Cx, (DeviceAddr<<1)|0x01);
    u32TimeOut = TIMEOUT;
	while(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_NACKDETECTF))//等待应答
	{
		if(0 == (u32TimeOut--)) 
		{
			if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
			{
				I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
			}			
			return I2C_TIMEROUT;
		}
	}
    
	for(pos = 0;pos<len;pos++)
	{
		u32TimeOut = TIMEOUT;
		while(Reset == I2C_GetStatus(pstcI2Cx, I2C_SR_RFULLF))
		{
			if(0 == (u32TimeOut--))
			{
				I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
				return I2C_TIMEROUT;
			}
		} 
#ifndef AUTOACK	
		if(pos == (len-1))
		{
			I2C_NackConfig(pstcI2Cx, Enable);
		}
        else
		{
			I2C_NackConfig(pstcI2Cx, Disable);//ACK
		}
#endif        
		data[pos] = pstcI2Cx->DRR_f.DR;//I2C_ReadData(I2C1_UNIT);
#ifdef AUTOACK	
		if(pos == (len-1))
		{
			I2C_NackConfig(pstcI2Cx, Enable);//NAK
		}
		else
		{
			I2C_NackConfig(pstcI2Cx, Disable);//ACK
		}
#endif		
	}
	u32TimeOut = TIMEOUT;
	do{
		I2C_GenerateStop(pstcI2Cx, Enable);//产生停止位
		if(0 == (u32TimeOut--)) 
			{
				return I2C_TIMEROUT;
			}
	}while(Reset == I2C_GetStatus(pstcI2Cx, I2C_SR_STOPF));//等待停止
	return I2C_RET_OK;
}
inline uint8_t I2C_Write_Buffer(M4_I2C_TypeDef* pstcI2Cx,uint8_t DeviceAddr,const uint8_t *data, uint16_t len)
{
	uint32_t u32TimeOut;
	uint8_t pos;
	if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_NACKDETECTF))//启动I2C前先清除上一次读写失败的NAK标志，否则无法写数据
	{
		I2C_ClearStatus(pstcI2Cx,I2C_CLR_NACKFCLR);
	}
	if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
	{
		if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_ARLOF))//仲裁失败
		{
//			I2C_ClearStatus(I2C_CH, I2C_CLR_ARLOFCLR);
			pstcI2Cx->CLR = 0xFFFF;
			I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
		}
		return I2C_BUSY;
	}
	I2C_GenerateStart(pstcI2Cx , Enable);
	u32TimeOut = TIMEOUT;
	while(Reset == I2C_GetStatus(pstcI2Cx, I2C_SR_TEMPTYF))//数据寄存器为空
	{
		if(0==u32TimeOut--)
		{
			if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
			{
				I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
			}			
			return I2C_TIMEROUT;
		}
	}
	I2C_SendData(pstcI2Cx, DeviceAddr<<1);
	u32TimeOut = TIMEOUT;
	while(Reset == I2C_GetStatus(pstcI2Cx, I2C_SR_TENDF))//数据发送完成
	{
		if(0==(u32TimeOut--))
		{
			if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
			{
				I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
			}			
			return I2C_TIMEROUT;
		}
	}
	u32TimeOut = TIMEOUT;
	while(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_NACKDETECTF))//等待应答
	{
		if(0 == (u32TimeOut--)) 
		{
			if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
			{
				I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
			}			
			return I2C_BADADDR;
		}
	}
	for(pos = 0;pos<len;pos++)
	{
		I2C_SendData(pstcI2Cx, data[pos]);
		u32TimeOut = TIMEOUT;
		while(Reset == I2C_GetStatus(pstcI2Cx, I2C_SR_TENDF))//数据寄存器为空
		{
			if(0==(u32TimeOut--))
			{
				if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
				{
					I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
				}			
				return I2C_TIMEROUT;
			}
		}
		u32TimeOut = TIMEOUT;
		while(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_NACKDETECTF))//等待应答
		{
			if(0 == (u32TimeOut--)) 
			{
				if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
				{
					I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
				}			
				return I2C_TIMEROUT;
			}
		}		
	}
	u32TimeOut = TIMEOUT;
	do{
		I2C_GenerateStop(pstcI2Cx, Enable);//产生停止位
		if(0 == (u32TimeOut--)) 
			{
				return I2C_TIMEROUT;
			}
	}while(Reset == I2C_GetStatus(pstcI2Cx, I2C_SR_STOPF));//等待停止
	return I2C_RET_OK;
}
inline uint8_t I2C_Read_Buffer(M4_I2C_TypeDef* pstcI2Cx,uint8_t DeviceAddr, uint8_t *data, uint16_t len)
{
	uint32_t u32TimeOut;
	uint8_t pos;
#ifndef AUTOACK	
	pstcI2Cx->CR3_f.FACKEN = 1;//非自动写ACK
#endif
	if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_NACKDETECTF))//启动I2C前先清除上一次读写失败的NAK标志，否则无法写数据
	{
		I2C_ClearStatus(pstcI2Cx,I2C_CLR_NACKFCLR);
	}
	if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
	{
		return I2C_BUSY;
	}
	I2C_GenerateStart(pstcI2Cx , Enable);
	u32TimeOut = TIMEOUT;
	while((Reset == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY)) ||
            (Reset == I2C_GetStatus(pstcI2Cx, I2C_SR_STARTF)))
    {
        if(0 == (u32TimeOut--)) 
		{
			I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
			return I2C_RET_ERROR;
		}
    }
	u32TimeOut = TIMEOUT;
	while(Reset == I2C_GetStatus(pstcI2Cx, I2C_SR_TEMPTYF))//数据寄存器为空
	{
		if(0==u32TimeOut--)
		{
			if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
			{
				I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
			}			
			return I2C_TIMEROUT;
		}
	}
	I2C_SendData(pstcI2Cx, (DeviceAddr<<1)|0x01);
    u32TimeOut = TIMEOUT;
	while(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_NACKDETECTF))//等待应答
	{
		if(0 == (u32TimeOut--)) 
		{
			if(Set == I2C_GetStatus(pstcI2Cx, I2C_SR_BUSY))
			{
				I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
			}			
			return I2C_TIMEROUT;
		}
	}
	for(pos = 0;pos<len;pos++)
	{	
		u32TimeOut = TIMEOUT;
		while(Reset == I2C_GetStatus(pstcI2Cx, I2C_SR_RFULLF))
		{
			if(0 == (u32TimeOut--))
			{
				I2C_GenerateStop(pstcI2Cx, Enable);//停止释放总线
				return I2C_TIMEROUT;
			}
		}
#ifndef AUTOACK			
		if(pos == (len-1))
		{
			I2C_NackConfig(pstcI2Cx, Enable);
		}
        else
		{
			I2C_NackConfig(pstcI2Cx, Disable);//ACK
		}
#endif	        
		data[pos] = pstcI2Cx->DRR_f.DR;//I2C_ReadData(I2C1_UNIT);
#ifdef AUTOACK			
		if(pos == (len-1))
		{
			I2C_NackConfig(pstcI2Cx, Enable);//NAK
		}
		else
		{
			I2C_NackConfig(pstcI2Cx, Disable);//ACK
		}
#endif
	}
	u32TimeOut = TIMEOUT;
	do{
		I2C_GenerateStop(pstcI2Cx, Enable);//产生停止位
		if(0 == (u32TimeOut--)) 
			{
				return I2C_TIMEROUT;
			}
	}while(Reset == I2C_GetStatus(pstcI2Cx, I2C_SR_STOPF));//等待停止
	return I2C_RET_OK;
}
