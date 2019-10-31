#ifndef MY_RTOS_I2C_DRIVER_H_
#define MY_RTOS_I2C_DRIVER_H_

/* Standard fixed-size data types */
#include "stdint.h"
/* MCU macros */
#include "MK64F12.h"
/* SDK drivers */
#include "fsl_clock.h"
#include "fsl_port.h"
#include "fsl_i2c.h"
/* Amazon FreeRTOS */
#include "FreeRTOS.h"
#include "semphr.h"


/* Flags for returned values from functions */
typedef enum {rtos_i2c_sucess = 0,rtos_i2c_fail} rtos_i2c_flag_t;
/* My driver configuration structure */
typedef struct
{
	uint32_t baudrate;
	I2C_Type* i2c_channel;
	PORT_Type* port;
	uint8_t scl_pin;
	uint8_t sda_pin;
	uint8_t pin_mux;
}rtos_i2c_config_t;

rtos_i2c_flag_t rtos_i2c_init(rtos_i2c_config_t* config);
rtos_i2c_flag_t rtos_i2c_transfer( I2C_Type* i2c_channel, i2c_master_transfer_t* xfer);


#endif /* MY_RTOS_I2C_DRIVER_H_ */
