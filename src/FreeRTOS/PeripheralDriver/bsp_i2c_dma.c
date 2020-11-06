#include "bsp_i2c_dma.h"
M4_I2C_TypeDef* pstc_int_I2Cx = NULL;
uint8_t * p_int_data = NULL;
void DMA_TX_TC_Callback(void)
{
    uint32_t u32TimeOut;
    u32TimeOut = TIMEOUT;
	while(Reset == I2C_GetStatus(pstc_int_I2Cx, I2C_SR_TENDF))//数据寄存器为空
	{
		if(0==(u32TimeOut--))
		{
			if(Set == I2C_GetStatus(pstc_int_I2Cx, I2C_SR_BUSY))
			{
					I2C_GenerateStop(pstc_int_I2Cx, Enable);//停止释放总线
			}			
			return;
		}
	}
    u32TimeOut = TIMEOUT;
	do{
		I2C_GenerateStop(pstc_int_I2Cx, Enable);//产生停止位
		if(0 == (u32TimeOut--)) 
			{
				return;
			}
	}while(Reset == I2C_GetStatus(pstc_int_I2Cx, I2C_SR_STOPF));//等待停止;
}
void DMA_RX_TC_Callback(void)
{
    uint32_t u32TimeOut;
    pstc_int_I2Cx->CR3_f.FACKEN = 1;
    u32TimeOut = TIMEOUT;
		while(Reset == I2C_GetStatus(pstc_int_I2Cx, I2C_SR_RFULLF))
		{
			if(0 == (u32TimeOut--))
			{
				I2C_GenerateStop(pstc_int_I2Cx, Enable);//停止释放总线
				return;
			}
		}

		I2C_NackConfig(pstc_int_I2Cx, Enable);//NAK
    *p_int_data = pstc_int_I2Cx->DRR_f.DR;   
    u32TimeOut = TIMEOUT;
	do{
		I2C_GenerateStop(pstc_int_I2Cx, Enable);//产生停止位
		if(0 == (u32TimeOut--)) 
			{
				return;
			}
	}while(Reset == I2C_GetStatus(pstc_int_I2Cx, I2C_SR_STOPF));//等待停止;
}
uint8_t  bsp_I2C_DMA_Init(M4_I2C_TypeDef* pstcI2Cx,uint32_t baudrate)
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
    bsp_dma_init(I2C_DMA_UNIT, DMA_RX_CH, DMA_Mode_RXsrc, DMA_Mode_RXdes, DataWidth);
    bsp_dma_init(I2C_DMA_UNIT, DMA_TX_CH, DMA_Mode_TXsrc, DMA_Mode_TXdes, DataWidth);
	return 0;
}

inline uint8_t I2C_DMA_Write_data(M4_I2C_TypeDef* pstcI2Cx,uint8_t DeviceAddr,uint8_t addr, const uint8_t *data, uint16_t len)
{
	uint32_t u32TimeOut;
	uint8_t pos;
    if(len == 0)
    {
        return I2C_BADPARA;
    }
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
    bsp_interrupt_callback_regist(DMA_TX_INT,Int001_IRQn,(void *)DMA_TX_TC_Callback);
    pstc_int_I2Cx = pstcI2Cx;
    bsp_dma_SetDesAddr(I2C_DMA_UNIT, DMA_TX_CH, (uint32_t)&(pstcI2Cx->DTR));
    bsp_dma_SetSrcAddr(I2C_DMA_UNIT, DMA_TX_CH, (uint32_t)data);
    bsp_dma_set_count(I2C_DMA_UNIT,DMA_TX_CH,len);//
    bsp_dma_ch_enable(I2C_DMA_UNIT,DMA_TX_CH,Enable);
    bsp_dma_set_TrigSrc(I2C_DMA_UNIT,DMA_TX_CH,EVT_I2C1_TXI);//先实现I2C1,其他后续再补上    
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
	return I2C_RET_OK;
}
inline uint8_t I2C_DMA_Read_data(M4_I2C_TypeDef* pstcI2Cx,uint8_t DeviceAddr,uint8_t addr, uint8_t *data, uint16_t len)
{
	uint32_t u32TimeOut;
	uint8_t pos;
    if(len == 0)
    {
        return I2C_BADPARA;
    }
	pstcI2Cx->CR3_f.FACKEN = 0;//自动写ACK
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
    if(len>1)//读长大于1时启用DMA
    {
        bsp_interrupt_callback_regist(DMA_RX_INT,Int002_IRQn,(void *)DMA_RX_TC_Callback);
        pstc_int_I2Cx = pstcI2Cx;
        p_int_data = (uint8_t*)data+len-1;
        bsp_dma_SetDesAddr(I2C_DMA_UNIT, DMA_RX_CH, (uint32_t)data);
        bsp_dma_SetSrcAddr(I2C_DMA_UNIT, DMA_RX_CH, (uint32_t)&(pstcI2Cx->DRR));
        bsp_dma_set_count(I2C_DMA_UNIT,DMA_RX_CH,len-1);
        bsp_dma_ch_enable(I2C_DMA_UNIT,DMA_RX_CH,Enable);
        bsp_dma_set_TrigSrc(I2C_DMA_UNIT,DMA_RX_CH,EVT_I2C1_RXI);//先实现I2C1,其他后续再补上 
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
    if(len==1)//读长为1时，不启用DMA直接读取，回NAK
    {
        pstc_int_I2Cx->CR3_f.FACKEN = 1;
        u32TimeOut = TIMEOUT;
            while(Reset == I2C_GetStatus(pstc_int_I2Cx, I2C_SR_RFULLF))
            {
                if(0 == (u32TimeOut--))
                {
                    I2C_GenerateStop(pstc_int_I2Cx, Enable);//停止释放总线
                    return I2C_TIMEROUT;
                }
            }

            I2C_NackConfig(pstc_int_I2Cx, Enable);//NAK
        *p_int_data = pstc_int_I2Cx->DRR_f.DR;   
        u32TimeOut = TIMEOUT;
        do{
            I2C_GenerateStop(pstc_int_I2Cx, Enable);//产生停止位
            if(0 == (u32TimeOut--)) 
                {
                    return I2C_TIMEROUT;
                }
        }while(Reset == I2C_GetStatus(pstc_int_I2Cx, I2C_SR_STOPF));//等待停止;
    }
	return I2C_RET_OK;
}