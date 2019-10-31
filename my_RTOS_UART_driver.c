/* Header file */
#include "my_RTOS_UART_driver.h"

/* Fixed amount of UART channels on the MCU */
#define NUMBER_OF_UART_CHANNELS (6U)

/* Static handles for UART channels */
typedef struct
{
	uint8_t is_init;
	uart_handle_t fsl_uart_handle;
	SemaphoreHandle_t mutex_rx;
	SemaphoreHandle_t mutex_tx;
	SemaphoreHandle_t rx_sem;
	SemaphoreHandle_t tx_sem;
}rtos_uart_hanlde_t;

static rtos_uart_hanlde_t uart_handles[NUMBER_OF_UART_CHANNELS] = {0};

/* Getters */
static inline uint8_t get_uart_number(UART_Type* uart_channel)
{
	uint8_t retval = 0xFF;

	switch( ((uint32_t) (uart_channel)) )
	{
	case UART0_BASE: retval = 0U; break;
	case UART1_BASE: retval = 1U; break;
	case UART2_BASE: retval = 2U; break;
	case UART3_BASE: retval = 3U; break;
	case UART4_BASE: retval = 4U; break;
	case UART5_BASE: retval = 5U; break;
	default:
		/* Error trap */
		for(;;);
		break;
	}

	return retval;
}
static inline clock_name_t get_uart_clock_source(UART_Type* uart_channel)
{
	clock_name_t retval;

	switch( ((uint32_t) (uart_channel)) )
	{
	case UART0_BASE: retval = UART0_CLK_SRC; break;
	case UART1_BASE: retval = UART1_CLK_SRC; break;
	case UART2_BASE: retval = UART2_CLK_SRC; break;
	case UART3_BASE: retval = UART3_CLK_SRC; break;
	case UART4_BASE: retval = UART4_CLK_SRC; break;
	case UART5_BASE: retval = UART5_CLK_SRC; break;
	default:
		/* Error trap */
		for(;;);
		break;
	}

	return retval;
}
static inline IRQn_Type get_uart_IRQ_ID(UART_Type* uart_channel)
{
	IRQn_Type retval;

	switch( ((uint32_t) (uart_channel)) )
	{
	case UART0_BASE: retval = UART0_RX_TX_IRQn; break;
	case UART1_BASE: retval = UART1_RX_TX_IRQn; break;
	case UART2_BASE: retval = UART2_RX_TX_IRQn; break;
	case UART3_BASE: retval = UART3_RX_TX_IRQn; break;
	case UART4_BASE: retval = UART4_RX_TX_IRQn; break;
	case UART5_BASE: retval = UART5_RX_TX_IRQn; break;
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
static void fsl_uart_callback(UART_Type* base, uart_handle_t* handle, status_t status, void* userData)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* Calculate UART number just once */
	uint8_t uart_number = get_uart_number(base);

	/* Give internal semaphore if the UART channel is on idle state */
	if (kStatus_UART_TxIdle == status)
	{
		xSemaphoreGiveFromISR(uart_handles[uart_number].tx_sem, &xHigherPriorityTaskWoken );
	}

	/* Give internal semaphore if the UART channel is on idle state */
	if (kStatus_UART_RxIdle == status)
	{
		xSemaphoreGiveFromISR(uart_handles[uart_number].rx_sem, &xHigherPriorityTaskWoken );
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/* Public functions */
rtos_uart_flag_t rtos_uart_init(rtos_uart_config_t* config)
{
	/* Failed assumed until proven success */
	rtos_uart_flag_t retval = rtos_uart_fail;
	/* SDK UART driver configuration structure */
	uart_config_t fsl_config;

	/* Calculate useful data at the beginning so it will only be calculated once. */
	uint8_t uart_number            = get_uart_number(config->uart_channel);
	clock_name_t uart_clock_source = get_uart_clock_source(config->uart_channel);
	IRQn_Type uart_IRQ_ID          = get_uart_IRQ_ID(config->uart_channel);


	if(!uart_handles[uart_number].is_init)
	{
		/* Fill in handle*/
		uart_handles[uart_number].mutex_rx = xSemaphoreCreateMutex();
		uart_handles[uart_number].mutex_tx = xSemaphoreCreateMutex();
		uart_handles[uart_number].rx_sem   = xSemaphoreCreateBinary();
		uart_handles[uart_number].tx_sem   = xSemaphoreCreateBinary();

		/* Configure Port */
		enable_port_clock(config->port);
		PORT_SetPinMux(config->port, config->rx_pin, config->pin_mux);
		PORT_SetPinMux(config->port, config->tx_pin, config->pin_mux);

		/* Configure UART channel */
		UART_GetDefaultConfig(&fsl_config);
		fsl_config.baudRate_Bps = config->baudrate;
		fsl_config.enableTx     = true;
		fsl_config.enableRx     = true;

		/* Initialize UART and enable its ISR */
		UART_Init(config->uart_channel, &fsl_config, CLOCK_GetFreq(uart_clock_source) );
		NVIC_SetPriority(uart_IRQ_ID,5);

		/* Create a transference handle */
		UART_TransferCreateHandle( config->uart_channel, &uart_handles[uart_number].fsl_uart_handle, fsl_uart_callback, NULL);

		/* Set initialization flag and signal success */
		uart_handles[uart_number].is_init = 1;
		retval = rtos_uart_sucess;
	}


	return retval;
}
rtos_uart_flag_t rtos_uart_send(UART_Type* uart_channel, uint8_t* buffer, uint16_t lenght)
{
	/* Failed assumed until proven success */
	rtos_uart_flag_t flag = rtos_uart_fail;
	uart_transfer_t xfer;

	/* Calculate useful data at the beginning so it will only be calculated once. */
	uint8_t uart_number = get_uart_number(uart_channel);

	if(uart_handles[uart_number].is_init)
	{
		/* Define transmission settings */
		xfer.data     = buffer;
		xfer.dataSize = lenght;

		/* Take serial port mutex */
		xSemaphoreTake(uart_handles[uart_number].mutex_tx, portMAX_DELAY);

			/* Excecute transfer */
			UART_TransferSendNonBlocking(uart_channel , &uart_handles[uart_number].fsl_uart_handle, &xfer);
			/* Take uart transmission semaphore */
			xSemaphoreTake(uart_handles[uart_number].tx_sem,portMAX_DELAY);

		/* Release serial port mutex */
		xSemaphoreGive(uart_handles[uart_number].mutex_tx);

		/* Signal success */
		flag = rtos_uart_sucess;
	}

	return flag;
}
rtos_uart_flag_t rtos_uart_receive(UART_Type* uart_channel, uint8_t* buffer, uint16_t lenght)
{
	/* Failed assumed until proven success */
	rtos_uart_flag_t flag = rtos_uart_fail;
	uart_transfer_t xfer;

	/* Calculate useful data at the beginning so it will only be calculated once. */
	uint8_t uart_number = get_uart_number(uart_channel);

	if(uart_handles[uart_number].is_init)
	{
		/* Define transmission settings */
		xfer.data     = buffer;
		xfer.dataSize = lenght;

		/* Take serial port mutex */
		xSemaphoreTake(uart_handles[uart_number].mutex_rx, portMAX_DELAY);

			/* Excecute reception */
			UART_TransferReceiveNonBlocking(uart_channel, &uart_handles[uart_number].fsl_uart_handle, &xfer, NULL);
			/* Take reception semaphore */
			xSemaphoreTake(uart_handles[uart_number].rx_sem,portMAX_DELAY);

		/* Release serial port mutex */
		xSemaphoreGive(uart_handles[uart_number].mutex_rx);

		/* Signal success */
		flag = rtos_uart_sucess;
	}

	return flag;
}
