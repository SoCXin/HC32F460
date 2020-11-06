#ifndef HW_MPU_H
#define HW_MPU_H
#include "hc32_ddl.h"
//typedef struct
//{
//    __IO uint32_t MPURG4SIZE                : 5;
//    __IO uint32_t MPURG4ADDR                :27;
//} stc_mpu_rgd4_field_t;

typedef struct
{
	__IO uint32_t Enable			: 1;
	__IO uint32_t Privdefena_en		: 1;
	__IO uint32_t Hfnmiena_en		: 1;
	__IO uint32_t RESERVED			: 29;
}MPU_CTRL_CFG_t;

typedef struct
{
	__IO uint32_t Separate_en	: 1;
	__IO uint32_t RESERVED		: 7;
	__IO uint32_t Dregion		: 8;
	__IO uint32_t Iregion		: 8;
	__IO uint32_t RESERVED2		: 8;
}MPU_TYPE_t;
typedef struct
{
	__IO uint32_t  Region	: 8;
	__IO uint32_t  RESERVED	: 24;
}MPU_RNR_t;
typedef struct
{
	__IO uint32_t Region	: 4;
	__IO uint32_t Valid		: 1; 
//	__IO uint32_t  RESERVED	: ;
	__IO uint32_t addr		: 27;
}MPU_RBAR_t;
typedef struct
{
	__IO uint32_t Enable	:1;
	__IO uint32_t size		:5; 
	__IO uint32_t RESERVED	:2;
	__IO uint32_t SRD		:8;
	__IO uint32_t B			:1;
	__IO uint32_t C			:1;
	__IO uint32_t S			:1;
	__IO uint32_t TEX		:3;
	__IO uint32_t RESERVED2	:2;
	__IO uint32_t AP		:3;
	__IO uint32_t RESERVED3	:1;
	__IO uint32_t XN		:1;
	__IO uint32_t RESERVED4	:3;
}MPU_RASR_t;
typedef struct
{
	 union
	 {
		__IO uint32_t MPU_CTRL;
		MPU_CTRL_CFG_t MPU_CTRL_f;
	 };
//	 union
//	 {
//		__IO uint32_t MPU_TYPE;
//		MPU_TYPE_t MPU_TYPE_f;
//	 };
	 union
	 {
		__IO uint32_t MPU_RNR;
		MPU_RNR_t MPU_RNR_f;
	 };
	 union
	 {
		__IO uint32_t MPU_RBAR;
		MPU_RBAR_t MPU_RBAR_f;
	 };
	 union
	 {
		__IO uint32_t MPU_RASR;
		MPU_RASR_t MPU_RASR_f;
	 };
}MPU_CFG_t;
typedef struct
{
	union
	 {
		__IO uint32_t MPU_CTRL;
		MPU_CTRL_CFG_t MPU_CTRL_f;
	 };
}MPU_CTRL_Filed_t;
typedef struct
{
	union
	 {
		__IO uint32_t MPU_RNR;
		MPU_RNR_t MPU_RNR_f;
	 };
}MPU_RNR_Filed_t;
typedef struct
{
	 union
	 {
		__IO uint32_t MPU_RBAR;
		MPU_RBAR_t MPU_RBAR_f;
	 };
}MPU_RBAR_Filed_t;
typedef struct
{
	 union
	 {
		__IO uint32_t MPU_RASR;
		MPU_RASR_t MPU_RASR_f;
	 };
}MPU_RASR_Filed_t;
typedef struct
{
	 union
	 {
		__IO uint32_t MPU_TYPE;
		MPU_TYPE_t MPU_TYPE_f;
	 };
}MPU_MPU_TYPE_Filed_t;
void Hw_MPU_Init(void);
#endif
