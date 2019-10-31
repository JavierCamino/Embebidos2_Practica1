#ifndef MY_RTOS_UART_DRIVER_H_
#define MY_RTOS_UART_DRIVER_H_

/* Standard fixed-size data types */
#include "stdint.h"
/* MCU macros */
#include "MK64F12.h"
/* SDK drivers */
#include "fsl_clock.h"
#include "fsl_port.h"
#include "fsl_uart.h"
/* Amazon FreeRTOS */
#include "FreeRTOS.h"
#include "semphr.h"


/* Flags for returned values from functions */
typedef enum {rtos_uart_sucess = 0,rtos_uart_fail} rtos_uart_flag_t;
/* My driver configuration structure */
typedef struct
{
	uint32_t baudrate;
	UART_Type* uart_channel;
	PORT_Type* port;
	uint8_t rx_pin;
	uint8_t tx_pin;
	uint8_t pin_mux;
}rtos_uart_config_t;

rtos_uart_flag_t rtos_uart_init(rtos_uart_config_t* config);
rtos_uart_flag_t rtos_uart_send(UART_Type* uart_channel, uint8_t* buffer, uint16_t lenght);
rtos_uart_flag_t rtos_uart_receive(UART_Type* uart_channel, uint8_t* buffer, uint16_t lenght);


#endif /* MY_RTOS_UART_DRIVER_H_ */
