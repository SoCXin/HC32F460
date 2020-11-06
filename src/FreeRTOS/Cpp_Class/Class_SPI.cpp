#include "Class_SPI.h"
#include "../Tasks/Task_ADC.h"

SPI_Class ::SPI_Class()
{
	;
}
SPI_Class::~SPI_Class()
{
	;
}
uint8_t SPI_Class::SPI_Open(uint8_t DeviceNum,uint32_t speed, spi_mode_t Mode)
{
	return 0;
}
uint8_t SPI_Class::SPI_Write(uint8_t *buff, uint16_t len)
{
	return 0;
}
uint8_t SPI_Class::SPI_Read(uint8_t *buff, uint16_t len)
{
	return 0;
}
uint8_t SPI_Class::SetReadInterrupt(bool En,IRQn_Type IRQnum)
{
	return 0;
}
uint8_t SPI_Class::SetWriteInterrupt(bool En,IRQn_Type IRQnum)
{
	return 0;
}
void SPI_Class::SPIReadCallback(void)
{
	;
}
void SPI_Class::SPIWriteCallback(void)
{
	;
}

