#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H
void I2C_slave_init(void);
/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* Define I2C unit used for the example */
#define I2C_SLAVE_CH                          (M4_I2C1)
/* Define slave device address for example */
#define SLAVE_ADDRESS                   0x06u
/* Define port and pin for SDA and SCL */
#define I2C1_SCL_PORT                   (PortC)
#define I2C1_SCL_PIN                    (Pin04)
#define I2C1_SDA_PORT                   (PortC)
#define I2C1_SDA_PIN                    (Pin05)

#define TIMEOUT                         ((uint32_t)0x10000)

#define I2C_RET_OK                      0u
#define I2C_RET_ERROR                   1u

#define GENERATE_START                  0x00u
#define GENERATE_RESTART                0x01u

#define ADDRESS_W                       0x00u
#define ADDRESS_R                       0x01u

/* Define Write and read data length for the example */
#define SLAVE_DATA_LEN                   64u
/* Define i2c baudrate */
#define I2C_BAUDRATE                    400000ul

#endif
