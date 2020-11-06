#include "hc32_ddl.h"
#define I2C_CH M4_I2C1

#define I2C1_SCL_PORT   PortC//PortA//PortC
#define I2C1_SCL_Pin    Pin04//Pin01//Pin04
#define I2C1_SDA_PORT   PortC//PortA//PortC
#define I2C1_SDA_Pin    Pin05//Pin00//Pin05

/* Define E2PROM device address */
#define E2_ADDRESS                      0x50
/* AT24C02 page length is 8byte*/
#define PAGE_LEN                        8
/* Define test address for read and write */
#define DATA_TEST_ADDR                  0x00
/* Define port and pin for SDA and SCL */

#define E2_ADDRESS_W                    0x00
#define E2_ADDRESS_R                    0x01

#define GENERATE_START                  0x00
#define GENERATE_RESTART                0x01

#define TIMEOUT                         ((uint32_t)0x100)

#define I2C_RET_OK                      0
#define I2C_RET_ERROR                   1

#define ENABLE_I2C_REG_WRITE()            (M4_MSTP->FCG1_f.IIC1 = 0u)

void delay(uint32_t u32Tmp)
{
    while(u32Tmp--);
}

/**
 ******************************************************************************
 ** \brief  Send start or restart condition
 **
 ** \param  none
 **
 ** \return Process result
 **         - I2C_RET_ERROR  Send start failed
 **         - I2C_RET_OK     Send start success
 ******************************************************************************/
static uint8_t E2_StartOrRestart(uint8_t u8Start)
{
    uint32_t u32TimeOut = TIMEOUT;
    
    /* generate start or restart signal */
    if(GENERATE_START == u8Start)
    {
        /* Wait I2C bus idle */
        while(Set == I2C_GetStatus(I2C_CH, I2C_SR_BUSY))
        {
            if(0 == (u32TimeOut--)) return I2C_RET_ERROR;
        }
      
        I2C_GenerateStart(I2C_CH , Enable);
    }
    else
    {
        /* Clear start status flag */
        I2C_ClearStatus(I2C_CH, I2C_CLR_STARTFCLR);
        /* Send restart condition */
        I2C_GenerateReStart(I2C_CH , Enable);
    }

    /* Judge if start success*/
    u32TimeOut = TIMEOUT;
    while((Reset == I2C_GetStatus(I2C_CH, I2C_SR_BUSY)) ||
            (Reset == I2C_GetStatus(I2C_CH, I2C_SR_STARTF)))
    {
        if(0 == (u32TimeOut--)) return I2C_RET_ERROR;
    }
    
    return I2C_RET_OK;
}

/**
 ******************************************************************************
 ** \brief  Send e2prom device address
 **
 ** \param  u16Adr  The slave address
 **
 ** \return Process result
 **         - I2C_RET_ERROR  Send start failed
 **         - I2C_RET_OK     Send start success
 ******************************************************************************/
;static uint8_t E2_SendAdr(uint8_t u8Adr)
{
    uint32_t u32TimeOut = TIMEOUT;

    /* Wait tx buffer empty */
    while(Reset == I2C_GetStatus(I2C_CH, I2C_SR_TEMPTYF))
    {
		I2C_SoftwareResetCmd(I2C_CH,1);
        if(0 == (u32TimeOut--)) return I2C_RET_ERROR;
    }
    
    /* Send I2C address */
    I2C_SendData(I2C_CH, u8Adr);
   
    if(E2_ADDRESS_W == (u8Adr & 0x01))
    {
        /* If in master transfer process, Need wait transfer end*/
        uint32_t u32TimeOut = TIMEOUT;
        while(Reset == I2C_GetStatus(I2C_CH, I2C_SR_TENDF))
        {
            if(0 == (u32TimeOut--)) return I2C_RET_ERROR;
        }
        
        /* Check ACK */
        u32TimeOut = TIMEOUT;
        while(Set == I2C_GetStatus(I2C_CH, I2C_SR_NACKDETECTF))
        {
            if(0 == (u32TimeOut--)) 
			return I2C_RET_ERROR;
        }
    }
    
    return I2C_RET_OK;
}


/**
 ******************************************************************************
 ** \brief  Send data to e2prom
 **
 ** \param  pTxData  Pointer to the data buffer
 ** \param  u32Size  Data size
 **
 ** \return Process result
 **         - I2C_RET_ERROR  Send start failed
 **         - I2C_RET_OK     Send start success
 ******************************************************************************/
static uint8_t E2_WriteData(uint8_t *pTxData, uint32_t u32Size)
{
    uint32_t u32TimeOut = TIMEOUT;
    
    while(u32Size--)
    {
        /* Wait tx buffer empty */
        u32TimeOut = TIMEOUT;
        while(Reset == I2C_GetStatus(I2C_CH, I2C_SR_TEMPTYF))
        {
            if(0 == (u32TimeOut--)) return I2C_RET_ERROR;
        }
        
        /* Send one byte data */
        I2C_SendData(I2C_CH, *pTxData++);

        /* Wait transfer end*/
        u32TimeOut = TIMEOUT;
        while(Reset == I2C_GetStatus(I2C_CH, I2C_SR_TENDF))
        {
            if(0 == (u32TimeOut--)) return I2C_RET_ERROR;
        }
        
        /* Check ACK */
        u32TimeOut = TIMEOUT;
        while(Set == I2C_GetStatus(I2C_CH, I2C_SR_NACKDETECTF))
        {
            if(0 == (u32TimeOut--)) return I2C_RET_ERROR;
        }
    }
    
    return I2C_RET_OK;
}

/**
 ******************************************************************************
 ** \brief  Write address and receive data from e2prom
 **
 ** \param  u8Adr    Device address and R/W bit
 ** \param  pTxData  Pointer to the data buffer
 ** \param  u32Size  Data size
 **
 ** \return Process result
 **         - I2C_RET_ERROR  Send start failed
 **         - I2C_RET_OK     Send start success
 ******************************************************************************/
static uint8_t E2_SendAdrRevData(uint8_t u8Adr, uint8_t *pRxData, uint32_t u32Size)
{
    uint32_t u32TimeOut = TIMEOUT;

    /* Wait tx buffer empty */
    while(Reset == I2C_GetStatus(I2C_CH, I2C_SR_TEMPTYF))
    {
        if(0 == (u32TimeOut--)) return I2C_RET_ERROR;
    }
    
    for(uint32_t i=0; i<u32Size; i++)
    {
        /* if the last byte receive, need config NACK*/
        if(i == (u32Size - 1))
        {
            I2C_NackConfig(I2C_CH, Enable);
        }
        
        /* if first byte receive, need send adr*/
        if(0 == i)
        {
            I2C_SendData(I2C_CH, u8Adr);
        }
    
        /* Wait receive full flag*/
        u32TimeOut = TIMEOUT;
        while(Reset == I2C_GetStatus(I2C_CH, I2C_SR_RFULLF))
        {
            if(0 == (u32TimeOut--)) return I2C_RET_ERROR;
        }
        
        /* read data from register*/
        *pRxData++ = I2C_ReadData(I2C_CH);
        
    }
    
    return I2C_RET_OK;  
}

/**
 ******************************************************************************
 ** \brief  General stop condition to e2prom
 **
 ** \param  None
 **
 ** \return Process result
 **         - I2C_RET_ERROR  Send start failed
 **         - I2C_RET_OK     Send start success
 ******************************************************************************/
uint8_t E2_Stop(void)
{
    uint32_t u32TimeOut;
    
    /* Wait I2C bus busy */
    u32TimeOut = TIMEOUT;
    while(Reset == I2C_GetStatus(I2C_CH, I2C_SR_BUSY))
    {
        if(0 == (u32TimeOut--)) return I2C_RET_ERROR;
    }
  
    I2C_GenerateStop(I2C_CH, Enable);
    
    /* Wait STOPF */
    u32TimeOut = TIMEOUT;
    while(Reset == I2C_GetStatus(I2C_CH, I2C_SR_STOPF))
    {
        if(0 == (u32TimeOut--)) return I2C_RET_ERROR;
    }
    
    return I2C_RET_OK;
}

/**
 ******************************************************************************
 ** \brief  Initialize the I2C peripheral for e2prom
 **
 ** \param  None
 **
 ** \return Process result
 **         - I2C_RET_ERROR  Send start failed
 **         - I2C_RET_OK     Send start success
 ******************************************************************************/
uint8_t E2_Initialize(void)
{
    stc_i2c_init_t stcI2cInit;
    
    I2C_DeInit(I2C_CH);
    
    MEM_ZERO_STRUCT(stcI2cInit);
    stcI2cInit.enI2cMode = I2cMaster;
    stcI2cInit.u32Baudrate = 400000;
    I2C_Init(I2C_CH, &stcI2cInit);
    
    I2C_Cmd(I2C_CH, Enable);
    
    return I2C_RET_OK;
}

/**
 ******************************************************************************
 ** \brief  Initialize the system clock for the sample
 **
 ** \param  None
 **
 ** \return None
 ******************************************************************************/   

void User_I2C1_Init(void)
{
//    stc_i2c_init_t stcI2cInit;
//    MEM_ZERO_STRUCT(stcI2cInit);
//    
    PORT_SetFunc(I2C1_SCL_PORT, I2C1_SCL_Pin, Func_I2c1_Scl, Disable);
    PORT_SetFunc(I2C1_SDA_PORT, I2C1_SDA_Pin, Func_I2c1_Sda, Disable);
//    
    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_I2C1,Enable);
//    ENABLE_I2C_REG_WRITE();   
//    I2C_DeInit(I2C_CH);
//       
//    stcI2cInit.enI2cMode = I2cMaster;
//    stcI2cInit.u32Baudrate = 50000;
//    
//    I2C_Init(I2C_CH, &stcI2cInit);
//    I2C_Cmd(I2C_CH,Enable);
    
    E2_Initialize();
}

/**
 ******************************************************************************
 ** \brief  Write address and receive data from e2prom
 **
 ** \param  DeviceAddr    Device address 0xFE, The last bit R/W ignore;
 ** \param  DataAddr      Data Address
 ** \param  RecData       Pointer to the data buffer
 **
 ** \return Process result
 **         - I2C_RET_ERROR  Send start failed
 **         - I2C_RET_OK     Send start success
 ******************************************************************************/
uint8_t User_I2C1_Master_Write(uint8_t DeviceAddr, uint8_t DataAddr,uint8_t *TxData, uint8_t Data_len)
{
    uint8_t status = I2C_RET_OK;
    status = E2_StartOrRestart(GENERATE_START);
    if(status != I2C_RET_OK)
        {
            return status;
        }
    status = E2_SendAdr((DeviceAddr<<1)&0xFE);
    if(status != I2C_RET_OK)
        {
            return status;
        }
    status = E2_WriteData(&DataAddr,1);
    if(status != I2C_RET_OK)
        {
            return status;
        }
    status = E2_WriteData(TxData, Data_len);
    if(status != I2C_RET_OK)
        {
            return status;
        }
    status = E2_Stop();
    if(status != I2C_RET_OK)
        {
            return status;
        }
    return I2C_RET_OK;
}

uint8_t User_I2C1_Master_Read(uint8_t DeviceAddr, uint8_t DataAddr,uint8_t *RxData, uint8_t Data_len)
{
    static uint8_t i,status = I2C_RET_OK;
    uint32_t u32TimeOut = TIMEOUT;
    status = E2_StartOrRestart(GENERATE_START);
    if(status != I2C_RET_OK)
        {
			E2_Stop();
            return status;
        }
    status = E2_SendAdr((DeviceAddr<<1)&0xFE);
    if(status != I2C_RET_OK)
        {
			E2_Stop();
			I2C_CH->CR1_f.PE = 0;
			I2C_CH->CR1_f.SWRST = 1;
			I2C_CH->CR1_f.SWRST = 0;
			I2C_CH->CR1_f.PE = 1;			
            return status;
        }
    status = E2_WriteData(&DataAddr,1);
    if(status != I2C_RET_OK)
        {
			E2_Stop();
            return status;
        }
    status = E2_StartOrRestart(GENERATE_RESTART);
    if(status != I2C_RET_OK)
        {
			E2_Stop();
            return status;
        }
     status = E2_SendAdr((DeviceAddr<<1)|0x01);
    if(status != I2C_RET_OK)
        {
			E2_Stop();
            return status;
        }
    for(i=0; i<Data_len; i++)
        {
            /* if the last byte receive, need config NACK*/
            if(i == (Data_len - 1))
            {
                I2C_NackConfig(I2C_CH, Enable);
            }        
            while(Reset == I2C_GetStatus(I2C_CH, I2C_SR_RFULLF))
                {
                    if(0 == (u32TimeOut--))
					{
						status = E2_Stop();
						return I2C_RET_ERROR;
					}
						
                }        
            /* read data from register*/
            RxData[i] = I2C_ReadData(I2C_CH);
        }
     status = E2_Stop();
    if(status != I2C_RET_OK)
        {
            return status;
        }
    return I2C_RET_OK;
    
}
void TestEEPROM(void)
{
   
    uint8_t i, u8TxBuf[PAGE_LEN],u8RxBuf[PAGE_LEN];
    uint8_t u8Ret = I2C_RET_OK;
    for(i=0; i<PAGE_LEN; i++)
    {
        u8TxBuf[i] = i+1;
        u8RxBuf[i] = 0;
    }
#if 1    
    u8Ret = User_I2C1_Master_Write(E2_ADDRESS,DATA_TEST_ADDR,u8TxBuf,PAGE_LEN);
    if(u8Ret != I2C_RET_OK)
    {
        M4_I2C1->CR1_f.SWRST = 1;
        Ddl_Delay1ms(1);
        M4_I2C1->CR1_f.SWRST = 0;
    }
#endif  
    Ddl_Delay1ms(100);    
#if 1    
    u8Ret = User_I2C1_Master_Read(E2_ADDRESS,DATA_TEST_ADDR,u8RxBuf,PAGE_LEN);
    if(u8Ret != I2C_RET_OK)
    {
        M4_I2C1->CR1_f.SWRST = 1;
        Ddl_Delay1ms(1);
        M4_I2C1->CR1_f.SWRST = 0;
    }
#endif
    printf("Read EEPROM: ");
    for(i=0; i<PAGE_LEN; i++)
    {
        printf("%d",u8RxBuf[i]);
        printf(",");
    }
    printf("\r\n");
//     /* E2prom byte write*/
//    u8Ret = E2_StartOrRestart(GENERATE_START);
//    u8Ret = E2_SendAdr((E2_ADDRESS<<1)|E2_ADDRESS_W);
////    JudgeResult(u8Ret);
//    u8Ret = E2_SendAdr(DATA_TEST_ADDR);
////    JudgeResult(u8Ret);
//    u8Ret = E2_WriteData(u8TxBuf, 1);
////    JudgeResult(u8Ret);
//    u8Ret = E2_Stop();
////    JudgeResult(u8Ret);
}


