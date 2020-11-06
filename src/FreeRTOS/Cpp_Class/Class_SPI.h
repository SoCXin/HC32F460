#ifndef CLASS_SPI_H
#define CLASS_SPI_H
#include "hc32_ddl.h"
typedef enum SPIMODE
{
	Mode0,//CPOL 0, CPHA 0
	Mode1,//CPOL 0, CPHA 1
	Mode2,//CPOL 1, CPHA 0
	Mode4 //CPOL 1, CPHA 1
}spi_mode_t;
class SPI_Class
{
	public :
		SPI_Class();
		~SPI_Class();
		uint8_t SPI_Open(uint8_t DeviceNum,uint32_t speed, spi_mode_t Mode);
		uint8_t GetDeviceID(void);
		uint8_t SPI_Write(uint8_t *buff,uint16_t len);
		uint8_t SPI_Read(uint8_t *buff,uint16_t len);
		uint8_t SetReadInterrupt(bool En,IRQn_Type IRQnum);
		uint8_t SetWriteInterrupt(bool En,IRQn_Type IRQnum);
	private:
		uint8_t DeviceID;
		void SPIReadCallback(void);
		void SPIWriteCallback(void);
		IRQn_Type SPI_READ_IRQnum;
		IRQn_Type SPI_Write_IRQnum;
};




#endif

