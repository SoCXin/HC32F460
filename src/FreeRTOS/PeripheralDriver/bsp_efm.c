#include "bsp_efm.h"

en_result_t FlashWritePage(uint32_t u32Addr, EEFLASH_data_t *eeflashdata,uint32_t len)
{
    en_result_t result;
    uint32_t i,pageaddr;
    pageaddr = u32Addr&0xFFFFE000;//取页起始地址，以保证数据从起始地址写
		
   /* Unlock EFM. */ 
    EFM_Unlock();
    
    /* Enable flash. */
    EFM_FlashCmd(Enable);
    /* Wait flash ready. */
    while(Set != EFM_GetFlagStatus(EFM_FLAG_RDY));
		
    EFM_SectorErase(pageaddr);//擦除页
		
    pageaddr = u32Addr;    
    for(i = 0; i < len;i++)
    {
        result = EFM_SingleProgram(pageaddr,eeflashdata->U32_Data[i]);
        if(result!=Ok)
            return result;
        pageaddr += 4;
    }		
    EFM_Lock();
    return result;
}
en_result_t FlashReadPage(uint32_t u32Addr, EEFLASH_data_t *eeflashdata,uint32_t len)
{
    uint32_t i;
    uint32_t *pageaddr;
    pageaddr = (uint32_t *)(u32Addr&0xFFFFE000);//取页起始地址，以保证数据从起始地址写
    for(i = 0; i < len;i++)
    {
        eeflashdata->U32_Data[i] = *pageaddr;
        pageaddr ++;
    }	
    return Ok;
}
en_result_t FlashWritePageRB(uint32_t u32Addr, EEFLASH_data_t *eeflashdata,uint32_t len)
{
    en_result_t result;
    uint32_t i,pageaddr;
    pageaddr = u32Addr&0xFFFFE000;//取页起始地址，以保证数据从起始地址写
		
   /* Unlock EFM. */ 
    EFM_Unlock();
    
    /* Enable flash. */
    EFM_FlashCmd(Enable);
    /* Wait flash ready. */
    while(Set != EFM_GetFlagStatus(EFM_FLAG_RDY));
		
    EFM_SectorErase(pageaddr);//擦除页
		
    pageaddr = u32Addr;    
    for(i = 0; i < len;i++)
    {
        EFM_SingleProgramRB(pageaddr,eeflashdata->U32_Data[i]);
        if(result!=Ok)
            return result;
        pageaddr += 4;
    }		
    EFM_Lock();
    return result;
}
void TestEFM(void)
{
    en_result_t result;
    EEFLASH_data_t *p;
    p = (EEFLASH_data_t *) 0x00;
    static    EEFLASH_data_t readbuf;
    FlashReadPage(0x0,&readbuf,2048);
    result = FlashWritePage(0x18000,&readbuf,2048);
    printf("FlashWritePage Result %d\r\n",result);
    result = FlashWritePageRB(0x1A000,&readbuf,2048);
    printf("FlashWritePageRB Result %d\r\n",result);
    result = FlashWritePage(0x1C000,p,2048);
    printf("FlashWritePage Result %d\r\n",result);
}