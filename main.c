#include "stdio.h"
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"

#include "my_RTOS_UART_driver.h"
#include "my_RTOS_I2C_driver.h"
#include "BMI160.h"


/////////////////////////////////////////
/* Macros */
/////////////////////////////////////////
#define TEST_MODE

#define APP_I2C_CHANNEL				(I2C0)
#define APP_UART_CHANNEL			(UART0)

#define BMI160_INIT_TASK_NAME		"BMI160_init_task"

#define BMI160_INIT_TASK_PRIORITY	(configMAX_PRIORITIES - 1U)

#define BMI160_INIT_TRANSFER_LENGTH	(1U)

#define CREATE_TASK(a,b,c,d,e,f)	{												\
										if( pdPASS != xTaskCreate(a,b,c,d,e,f) )	\
										{											\
											PRINTF("%s creation failed.\r\n",#b);	\
											for(;;);								\
										}											\
									}
#define CREATE_SEMAPHORE(r)			{												\
										r = xSemaphoreCreateBinary();				\
										if(NULL == r)								\
										{											\
											PRINTF("%s creation failed.\r\n",#r);	\
											for(;;);								\
										}											\
									}
#define CREATE_MUTEX(r)				{												\
										r = xSemaphoreCreateMutex();				\
										if(NULL == r)								\
										{											\
											PRINTF("%s creation failed.\r\n",#r);	\
											for(;;);								\
										}											\
									}
#define CREATE_EVENTGROUP(r)		{												\
										r = xEventGroupCreate();					\
										if(NULL == r)								\
										{											\
											PRINTF("%s creation failed.\r\n",#r);	\
											for(;;);								\
										}											\
									}
#define CREATE_QUEUE(r,a,b)			{												\
										r = xQueueCreate(a,b);						\
										if(NULL == r)								\
										{											\
											PRINTF("%s creation failed.\r\n",#r);	\
											for(;;);								\
										}											\
									}


TaskHandle_t BMI160_init_task_handle = 0;


void BMI160_init_task(void* args)
{
/* Task local variables */
	/* Transfer structures for reading and from/to the sensor */
	i2c_master_transfer_t write_xfer = {0};
	i2c_master_transfer_t read_xfer  = {0};
	/* Buffers to store the transfers' payload */
	uint8_t write_buffer[BMI160_INIT_TRANSFER_LENGTH];
	uint8_t read_buffer[BMI160_INIT_TRANSFER_LENGTH];





/* Perform initial reading of PM_Status */
#ifdef TEST_MODE
	/* Define write transfer */
	read_xfer.data           = ((uint8_t* volatile) (read_buffer));
	read_xfer.dataSize       = BMI160_INIT_TRANSFER_LENGTH;
	read_xfer.direction      = kI2C_Read;
	read_xfer.flags          = kI2C_TransferDefaultFlag;
	read_xfer.slaveAddress   = BMI160_I2C_SLAVE_ADDR;
	read_xfer.subaddress     = BMI160_PMU_STATUS_REG_ADDR;
	read_xfer.subaddressSize = 1U;
	/* Read transfer */
	rtos_i2c_transfer(APP_I2C_CHANNEL, &read_xfer);

	/* Print result */
	uint8_t ansA[28];
	sprintf( ((char*) ansA), "PMU_STATUS inicial = 0x%02x\r\n", read_buffer[0]);
	rtos_uart_send(APP_UART_CHANNEL, ((uint8_t*) ansA), 28U);
#endif





/* Configure ACC to normal mode */
	/* Define write transfer */
	write_buffer[0]			  = BMI160_CMD_ACC_INIT;
	write_xfer.data           = ((uint8_t* volatile) (write_buffer));
	write_xfer.dataSize       = BMI160_INIT_TRANSFER_LENGTH;
	write_xfer.direction      = kI2C_Write;
	write_xfer.flags          = kI2C_TransferDefaultFlag;
	write_xfer.slaveAddress   = BMI160_I2C_SLAVE_ADDR;
	write_xfer.subaddress     = BMI160_CMD_REG_ADDR;
	write_xfer.subaddressSize = 1U;
	/* Write transfer */
	rtos_i2c_transfer(APP_I2C_CHANNEL, &write_xfer);



/* Leave task until the sensor has accepted the command, so a new one can be sent */
vTaskDelay( pdMS_TO_TICKS(BMI160_CMD_DELAY) );



/* Configure GYR to normal mode */
	/* Define write transfer */
	write_buffer[0]			  = BMI160_CMD_GYR_INIT;
	write_xfer.data           = ((uint8_t* volatile) (write_buffer));
	write_xfer.dataSize       = BMI160_INIT_TRANSFER_LENGTH;
	write_xfer.direction      = kI2C_Write;
	write_xfer.flags          = kI2C_TransferDefaultFlag;
	write_xfer.slaveAddress   = BMI160_I2C_SLAVE_ADDR;
	write_xfer.subaddress     = BMI160_CMD_REG_ADDR;
	write_xfer.subaddressSize = 1U;
	/* Write transfer */
	rtos_i2c_transfer(APP_I2C_CHANNEL, &write_xfer);



/* Leave task until the sensor has accepted the command, so a new one can be sent */
vTaskDelay( pdMS_TO_TICKS(BMI160_CMD_DELAY) );





/* Perform final reading of PM_Status */
#ifdef TEST_MODE
	/* Define write transfer */
	read_xfer.data           = ((uint8_t* volatile) (read_buffer));
	read_xfer.dataSize       = BMI160_INIT_TRANSFER_LENGTH;
	read_xfer.direction      = kI2C_Read;
	read_xfer.flags          = kI2C_TransferDefaultFlag;
	read_xfer.slaveAddress   = BMI160_I2C_SLAVE_ADDR;
	read_xfer.subaddress     = BMI160_PMU_STATUS_REG_ADDR;
	read_xfer.subaddressSize = 1U;
	/* Read transfer */
	rtos_i2c_transfer(APP_I2C_CHANNEL, &read_xfer);

	/* Print result */
	uint8_t ansB[32];
	sprintf( ((char*) ansB), "PMU_STATUS final   = 0x%02x\r\n", read_buffer[0]);
	rtos_uart_send(APP_UART_CHANNEL, ((uint8_t*) ansB), 32U);
#endif





/* Suspend current task */
	vTaskSuspend(BMI160_init_task_handle);
}






int main(void) {
	/* Initialize board clocks */
	BOARD_InitBootClocks();
/* Block execution before power-off */
//	while(1);





/* Initialize I2C module */
	/* I2C module configuration structure */
	rtos_i2c_config_t i2c_config;
	/* Define channel configuration */
	i2c_config.baudrate    = 100000U;
	i2c_config.i2c_channel = APP_I2C_CHANNEL;
	i2c_config.pin_mux     = 2U;
	i2c_config.port        = PORTB;
	i2c_config.scl_pin     = 2U;
	i2c_config.sda_pin     = 3U;
	/* Initialize channel */
	rtos_i2c_init(&i2c_config);


/* Initialize UART module */
	rtos_uart_config_t uart_config;
	/* Define channel configuration */
	uart_config.baudrate      = 115200U;
	uart_config.rx_pin        = 16U;
	uart_config.tx_pin        = 17U;
	uart_config.pin_mux       = kPORT_MuxAlt3;
	uart_config.uart_channel  = APP_UART_CHANNEL;
	uart_config.port          = PORTB;
	/* Initialize channel */
	rtos_uart_init(&uart_config);







    /* Create tasks. */
	CREATE_TASK(BMI160_init_task, BMI160_INIT_TASK_NAME, configMINIMAL_STACK_SIZE, NULL, BMI160_INIT_TASK_PRIORITY, &BMI160_init_task_handle);





	/* Start scheduler*/
    vTaskStartScheduler();




    /* Delete Tasks*/
	vTaskDelete(BMI160_init_task_handle);


	for(;;);
    return 0;
}
