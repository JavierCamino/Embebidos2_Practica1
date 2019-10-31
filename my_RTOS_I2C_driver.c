/* Header file */
#include "my_RTOS_I2C_driver.h"

/* Fixed amount of I2C channels on the MCU */
#define NUMBER_OF_I2C_CHANNELS (3U)

/* Static handles for I2C channels */
typedef struct
{
	uint8_t is_init;
	i2c_master_handle_t fsl_i2c_handle;
	SemaphoreHandle_t mutex;
	SemaphoreHandle_t semaphore;
}rtos_i2c_hanlde_t;

static rtos_i2c_hanlde_t i2c_handles[NUMBER_OF_I2C_CHANNELS] = {0};


/* Getters */
static inline uint8_t get_i2c_number(I2C_Type* i2c_channel)
{
	uint8_t retval = 0xFF;

	switch( ((uint32_t) (i2c_channel)) )
	{
	case I2C0_BASE: retval = 0U; break;
	case I2C1_BASE: retval = 1U; break;
	case I2C2_BASE: retval = 2U; break;
	default:
		/* Error trap */
		for(;;);
		break;
	}

	return retval;
}
static inline clock_name_t get_i2c_clock_source(I2C_Type* i2c_channel)
{
	clock_name_t retval;

	switch( ((uint32_t) (i2c_channel)) )
	{
	case I2C0_BASE: retval = I2C0_CLK_SRC; break;
	case I2C1_BASE: retval = I2C1_CLK_SRC; break;
	case I2C2_BASE: retval = I2C2_CLK_SRC; break;
	default:
		/* Error trap */
		for(;;);
		break;
	}

	return retval;
}
static inline IRQn_Type get_i2c_IRQ_ID(I2C_Type* i2c_channel)
{
	IRQn_Type retval;

	switch( ((uint32_t) (i2c_channel)) )
	{
	case I2C0_BASE: retval = I2C0_IRQn; break;
	case I2C1_BASE: retval = I2C1_IRQn; break;
	case I2C2_BASE: retval = I2C2_IRQn; break;
	default:
		/* Error trap */
		for(;;);
		break;
	}

	return retval;
}
/* Clock Gating */
static inline void enable_port_clock(PORT_Type* port)
{
	/* Enable the clock for the specified port */
	switch( ((uint32_t) (port)) )
	{
	case PORTA_BASE: CLOCK_EnableClock(kCLOCK_PortA); break;
	case PORTB_BASE: CLOCK_EnableClock(kCLOCK_PortB); break;
	case PORTC_BASE: CLOCK_EnableClock(kCLOCK_PortC);	break;
	case PORTD_BASE: CLOCK_EnableClock(kCLOCK_PortD);	break;
	case PORTE_BASE: CLOCK_EnableClock(kCLOCK_PortE);	break;
	default:
		/* Error trap */
		for(;;);
		break;
	}
}
/* Callback's */
static void fsl_i2c_callback(I2C_Type* base, i2c_master_handle_t* handle, status_t status, void* userData)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* Calculate i2c number just once */
	uint8_t i2c_number = get_i2c_number(base);

	/* Give internal semaphore if the i2c channel is on idle state */
	if(kStatus_I2C_Busy != status)
	{
		xSemaphoreGiveFromISR(i2c_handles[i2c_number].semaphore, &xHigherPriorityTaskWoken );
	}


	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/* Public functions */
rtos_i2c_flag_t rtos_i2c_init(rtos_i2c_config_t* config)
{
	/* Failed assumed until proven success */
	rtos_i2c_flag_t retval = rtos_i2c_fail;
	/* SDK UART driver configuration structure */
	i2c_master_config_t fsl_config;

	/* Calculate useful data at the beginning so it will only be calculated once. */
	uint8_t i2c_number            = get_i2c_number(config->i2c_channel);
	clock_name_t i2c_clock_source = get_i2c_clock_source(config->i2c_channel);
	IRQn_Type i2c_IRQ_ID          = get_i2c_IRQ_ID(config->i2c_channel);


	if(!i2c_handles[i2c_number].is_init)
	{
		/* Fill in handle*/
		i2c_handles[i2c_number].mutex     = xSemaphoreCreateMutex();
		i2c_handles[i2c_number].semaphore = xSemaphoreCreateBinary();

		/* Configure Port */
		enable_port_clock(config->port);
		PORT_SetPinMux(config->port, config->scl_pin, config->pin_mux);
		PORT_SetPinMux(config->port, config->sda_pin, config->pin_mux);

		/* Configure I2C channel */
		I2C_MasterGetDefaultConfig(&fsl_config);
		fsl_config.baudRate_Bps = config->baudrate;
		fsl_config.enableMaster = true;


		/* Initialize I2C and enable its ISR */
		I2C_MasterInit(config->i2c_channel, &fsl_config, CLOCK_GetFreq(i2c_clock_source) );
		NVIC_SetPriority(i2c_IRQ_ID,5);

		/* Create a transference handle */
		I2C_MasterTransferCreateHandle( config->i2c_channel, &(i2c_handles[i2c_number].fsl_i2c_handle), fsl_i2c_callback, NULL);

		/* Set initialization flag and signal success */
		i2c_handles[i2c_number].is_init = 1;
		retval = rtos_i2c_sucess;
	}


	return retval;
}
rtos_i2c_flag_t rtos_i2c_transfer( I2C_Type* i2c_channel, i2c_master_transfer_t* xfer)
{
	/* Failed assumed until proven success */
	rtos_i2c_flag_t flag = rtos_i2c_fail;
	/* Calculate useful data at the beginning so it will only be calculated once. */
	uint8_t i2c_number = get_i2c_number(i2c_channel);


	if(i2c_handles[i2c_number].is_init)
	{
		/* Take serial port mutex */
		xSemaphoreTake(i2c_handles[i2c_number].mutex, portMAX_DELAY);

			/* Excecute transfer */
			I2C_MasterTransferNonBlocking(i2c_channel, &(i2c_handles[i2c_number].fsl_i2c_handle), xfer);
			/* Take i2c transmission semaphore */
			xSemaphoreTake(i2c_handles[i2c_number].semaphore,portMAX_DELAY);

		/* Release serial port mutex */
		xSemaphoreGive(i2c_handles[i2c_number].mutex);

		/* Signal success */
		flag = rtos_i2c_sucess;
	}


	return flag;
}
