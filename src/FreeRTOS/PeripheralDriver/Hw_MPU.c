#include "Hw_MPU.h"
/**
  * @brief  Disables the MPU
  * @retval None
  */
void HAL_MPU_Disable(void)
{
  /* Make sure outstanding transfers are done */
  __DMB();

  /* Disable fault exceptions */
  SCB->SHCSR &= ~SCB_SHCSR_MEMFAULTENA_Msk;
  
  /* Disable the MPU and clear the control register*/
  MPU->CTRL = 0U;
}
/**
  * @brief  Enable the MPU.
  * @param  MPU_Control Specifies the control mode of the MPU during hard fault, 
  *          NMI, FAULTMASK and privileged access to the default memory 
  *          This parameter can be one of the following values:
  *            @arg MPU_HFNMI_PRIVDEF_NONE
  *            @arg MPU_HARDFAULT_NMI
  *            @arg MPU_PRIVILEGED_DEFAULT
  *            @arg MPU_HFNMI_PRIVDEF
  * @retval None
  */
void HAL_MPU_Enable(uint32_t MPU_Control)
{
  /* Enable the MPU */
  MPU->CTRL = MPU_Control | MPU_CTRL_ENABLE_Msk;
  
  /* Enable fault exceptions */
  SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
  
  /* Ensure MPU setting take effects */
  __DSB();
  __ISB();
}
/**
  * @brief  Initializes and configures the Region and the memory to be protected.
  * @param  MPU_Init Pointer to a MPU_Region_InitTypeDef structure that contains
  *                the initialization and configuration information.
  * @retval None
  */
//void HAL_MPU_ConfigRegion(MPU_Region_InitTypeDef *MPU_Init)
//{
//  /* Check the parameters */
//  assert_param(IS_MPU_REGION_NUMBER(MPU_Init->Number));
//  assert_param(IS_MPU_REGION_ENABLE(MPU_Init->Enable));

//  /* Set the Region number */
//  MPU->RNR = MPU_Init->Number;

//  if ((MPU_Init->Enable) != RESET)
//  {
//    /* Check the parameters */
//    assert_param(IS_MPU_INSTRUCTION_ACCESS(MPU_Init->DisableExec));
//    assert_param(IS_MPU_REGION_PERMISSION_ATTRIBUTE(MPU_Init->AccessPermission));
//    assert_param(IS_MPU_TEX_LEVEL(MPU_Init->TypeExtField));
//    assert_param(IS_MPU_ACCESS_SHAREABLE(MPU_Init->IsShareable));
//    assert_param(IS_MPU_ACCESS_CACHEABLE(MPU_Init->IsCacheable));
//    assert_param(IS_MPU_ACCESS_BUFFERABLE(MPU_Init->IsBufferable));
//    assert_param(IS_MPU_SUB_REGION_DISABLE(MPU_Init->SubRegionDisable));
//    assert_param(IS_MPU_REGION_SIZE(MPU_Init->Size));
//    
//    MPU->RBAR = MPU_Init->BaseAddress;
//    MPU->RASR = ((uint32_t)MPU_Init->DisableExec             << MPU_RASR_XN_Pos)   |
//                ((uint32_t)MPU_Init->AccessPermission        << MPU_RASR_AP_Pos)   |
//                ((uint32_t)MPU_Init->TypeExtField            << MPU_RASR_TEX_Pos)  |
//                ((uint32_t)MPU_Init->IsShareable             << MPU_RASR_S_Pos)    |
//                ((uint32_t)MPU_Init->IsCacheable             << MPU_RASR_C_Pos)    |
//                ((uint32_t)MPU_Init->IsBufferable            << MPU_RASR_B_Pos)    |
//                ((uint32_t)MPU_Init->SubRegionDisable        << MPU_RASR_SRD_Pos)  |
//                ((uint32_t)MPU_Init->Size                    << MPU_RASR_SIZE_Pos) |
//                ((uint32_t)MPU_Init->Enable                  << MPU_RASR_ENABLE_Pos);
//  }
//  else
//  {
//    MPU->RBAR = 0x00U;
//    MPU->RASR = 0x00U;
//  }
//}
void Hw_MPU_Init(void)
{
	MPU_CTRL_Filed_t	ctrl_cfg;
	MPU_RNR_Filed_t		rnr_cfg;
	MPU_RBAR_Filed_t	rbar_cfg;
	MPU_RASR_Filed_t	rasr_cfg;
	MPU_MPU_TYPE_Filed_t type_data;
	MEM_ZERO_STRUCT(ctrl_cfg);
	MEM_ZERO_STRUCT(rnr_cfg);
	MEM_ZERO_STRUCT(rbar_cfg);
	MEM_ZERO_STRUCT(rasr_cfg);
	MEM_ZERO_STRUCT(type_data);
	type_data.MPU_TYPE = MPU->TYPE;
	if(type_data.MPU_TYPE_f.Dregion == 0)
	{
		return;//已经配置
	}
//	__DMB();
//	ctrl_cfg.MPU_CTRL_f.Enable = Disable;
//	MPU->CTRL = ctrl_cfg.MPU_CTRL;//关闭MPU
	HAL_MPU_Disable();
	
	rnr_cfg.MPU_RNR_f.Region = 0;//region 0
	rbar_cfg.MPU_RBAR_f.addr = 0x0;
	rasr_cfg.MPU_RASR_f.size = 16;//空间大小等于2^(size+1) 128K空间
	rasr_cfg.MPU_RASR_f.B = Enable;//Bufferable
	rasr_cfg.MPU_RASR_f.C = Enable;//Cacheable;
	rasr_cfg.MPU_RASR_f.S = Enable;//Non-Shareable;
	rasr_cfg.MPU_RASR_f.AP = 0x01;//特权许可，用户禁止
	rasr_cfg.MPU_RASR_f.TEX = 0x01;
	rasr_cfg.MPU_RASR_f.XN = 0;//Instruction fetches Enable;
	MPU->RNR = rnr_cfg.MPU_RNR;
	MPU->RBAR = rbar_cfg.MPU_RBAR;
	MPU->RASR = rasr_cfg.MPU_RASR;
	
	ctrl_cfg.MPU_CTRL_f.Enable = Enable;
	ctrl_cfg.MPU_CTRL_f.Hfnmiena_en = Enable;
	ctrl_cfg.MPU_CTRL_f.Privdefena_en = Enable;//特权模式下打开背景region
	SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
	MPU->CTRL = ctrl_cfg.MPU_CTRL;//打开MPU	
	__DSB();
	__ISB();
}
void MPU_Init(MPU_CFG_t *p_cfg)
{
	MPU->CTRL = p_cfg->MPU_CTRL;
	MPU->RASR = p_cfg->MPU_RASR;
	MPU->RBAR = p_cfg->MPU_RBAR;
	MPU->RNR  = p_cfg->MPU_RNR;
}


