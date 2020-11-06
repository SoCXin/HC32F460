#include "Class_SPI.h"
#include "Class_I2C.h"
SPI_Class myspi;
#ifdef __cplusplus
extern "C" {
#endif


rt_device_ops_t I2C_Ops = 
{
	HW_I2C_Init,
	NULL,
	I2C_Read_data,
	I2C_Write_data
};
void Testcpp(void)
{
//	I2C_Class I2C_DV1(M4_I2C1,&I2C_Ops);
	myspi.SPI_Open(0,0,Mode0);
//	I2C_DV1.I2C_init(1000);
}

#ifdef __cplusplus
};
#endif
