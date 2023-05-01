/************************************************************************
* 5 semestre - Eng. da Computao - Insper
*
* 2021 - Exemplo com HC05 com RTOS
*
*/

#include <asf.h>
#include "conf_board.h"
#include <string.h>

/************************************************************************/
/* defines                                                              */
/************************************************************************/

// LEDs
#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

// One Button
#define ONE_BUT_PIO      PIOD
#define ONE_BUT_PIO_ID   ID_PIOD
#define ONE_BUT_IDX      30
#define ONE_BUT_IDX_MASK (1 << ONE_BUT_IDX)

// Two  Button
#define TWO_BUT_PIO      PIOC
#define TWO_BUT_PIO_ID   ID_PIOC
#define TWO_BUT_IDX      19
#define TWO_BUT_IDX_MASK (1 << TWO_BUT_IDX)

// Three  Button
#define THREE_BUT_PIO      PIOD
#define THREE_BUT_PIO_ID   ID_PIOD
#define THREE_BUT_IDX      11
#define THREE_BUT_IDX_MASK (1 << THREE_BUT_IDX)

// Four  Button
#define FOUR_BUT_PIO      PIOA
#define FOUR_BUT_PIO_ID   ID_PIOA
#define FOUR_BUT_IDX      24
#define FOUR_BUT_IDX_MASK (1 << FOUR_BUT_IDX)

// usart (bluetooth ou serial)
// Descomente para enviar dados
// pela serial debug

//#define DEBUG_SERIAL //comente para ativar o bluetooth na placa; não esqueça de mudar de 115200->960 0

#ifdef DEBUG_SERIAL
#define USART_COM USART1
#define USART_COM_ID ID_USART1
#else
#define USART_COM USART0
#define USART_COM_ID ID_USART0
#endif

/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

#define TASK_BLUETOOTH_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_BLUETOOTH_STACK_PRIORITY        (tskIDLE_PRIORITY)

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/************************************************************************/
/* recursos RTOS                                                        */
/************************************************************************/

/** Semaforo a ser usado pela task led */
SemaphoreHandle_t xSemaphoreOneButton;
SemaphoreHandle_t xSemaphoreTwoButton;
SemaphoreHandle_t xSemaphoreThreeButton;
SemaphoreHandle_t xSemaphoreFourButton;

/************************************************************************/
/* local prototypes                                                     */
/************************************************************************/
void one_button_callback(void);
void two_button_callback(void);
void three_button_callback(void);
void four_button_callback(void);

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* RTOS application HOOK                                                */
/************************************************************************/

/* Called if stack overflow during execution */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	* identify which task has overflowed its stack.
	*/
	for (;;) {
	}
}

/* This function is called by FreeRTOS idle task */
extern void vApplicationIdleHook(void) {
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/* This function is called by FreeRTOS each tick */
extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void one_button_callback(void){
	  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	  xSemaphoreGiveFromISR(xSemaphoreOneButton, &xHigherPriorityTaskWoken);
}

void two_button_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreTwoButton, &xHigherPriorityTaskWoken);
}

void three_button_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreThreeButton, &xHigherPriorityTaskWoken);
}

void four_button_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreFourButton, &xHigherPriorityTaskWoken);
}


/************************************************************************/
/* init                                                              */
/************************************************************************/

//To do: simplifque os inits como está no io_init
void io_init(void) {

	// Ativa PIOs
	pmc_enable_periph_clk(LED_PIO_ID);
	
	pmc_enable_periph_clk(ONE_BUT_PIO_ID);
	pmc_enable_periph_clk(TWO_BUT_PIO_ID);
	pmc_enable_periph_clk(THREE_BUT_PIO_ID);
	pmc_enable_periph_clk(FOUR_BUT_PIO_ID);

	// Configura Pinos
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
	
	pio_configure(ONE_BUT_PIO, PIO_INPUT, ONE_BUT_IDX_MASK, PIO_PULLUP);
	pio_configure(TWO_BUT_PIO, PIO_INPUT, TWO_BUT_IDX_MASK, PIO_PULLUP);
	pio_configure(THREE_BUT_PIO, PIO_INPUT, THREE_BUT_IDX_MASK, PIO_PULLUP);
	pio_configure(FOUR_BUT_PIO, PIO_INPUT, FOUR_BUT_IDX_MASK, PIO_PULLUP);
}

void one_button_init(void) {
	pmc_enable_periph_clk(ONE_BUT_PIO_ID);
	pio_configure(ONE_BUT_PIO, PIO_INPUT, ONE_BUT_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(ONE_BUT_PIO, ONE_BUT_IDX_MASK, 60);
	pio_handler_set(ONE_BUT_PIO,ONE_BUT_PIO_ID,ONE_BUT_IDX_MASK,PIO_IT_FALL_EDGE, one_button_callback);// ALTERAÇÃO DE PIO_IT_FALL_EDGE PARA PIO_IT_RISE_EDGE
	pio_enable_interrupt(ONE_BUT_PIO, ONE_BUT_IDX_MASK);
	pio_get_interrupt_status(ONE_BUT_PIO);
	NVIC_EnableIRQ(ONE_BUT_PIO_ID);
	NVIC_SetPriority(ONE_BUT_PIO_ID, 4); 
}

void two_button_init(void) {
	pmc_enable_periph_clk(TWO_BUT_PIO_ID);
	pio_configure(TWO_BUT_PIO, PIO_INPUT, TWO_BUT_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(TWO_BUT_PIO, TWO_BUT_IDX_MASK, 60);
	pio_handler_set(TWO_BUT_PIO,TWO_BUT_PIO_ID,TWO_BUT_IDX_MASK,PIO_IT_FALL_EDGE, two_button_callback);// ALTERAÇÃO DE PIO_IT_FALL_EDGE PARA PIO_IT_RISE_EDGE
	pio_enable_interrupt(TWO_BUT_PIO, TWO_BUT_IDX_MASK);
	pio_get_interrupt_status(TWO_BUT_PIO);
	NVIC_EnableIRQ(TWO_BUT_PIO_ID);
	NVIC_SetPriority(TWO_BUT_PIO_ID, 4);
}

void three_button_init(void) {
	pmc_enable_periph_clk(THREE_BUT_PIO_ID);
	pio_configure(THREE_BUT_PIO, PIO_INPUT, THREE_BUT_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(THREE_BUT_PIO, THREE_BUT_IDX_MASK, 60);
	pio_handler_set(THREE_BUT_PIO,THREE_BUT_PIO_ID,THREE_BUT_IDX_MASK,PIO_IT_FALL_EDGE, three_button_callback);// ALTERAÇÃO DE PIO_IT_FALL_EDGE PARA PIO_IT_RISE_EDGE
	pio_enable_interrupt(THREE_BUT_PIO, THREE_BUT_IDX_MASK);
	pio_get_interrupt_status(THREE_BUT_PIO);
	NVIC_EnableIRQ(THREE_BUT_PIO_ID);
	NVIC_SetPriority(THREE_BUT_PIO_ID, 4);
}

void four_button_init(void) {
	pmc_enable_periph_clk(FOUR_BUT_PIO_ID);
	pio_configure(FOUR_BUT_PIO, PIO_INPUT, FOUR_BUT_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(FOUR_BUT_PIO, FOUR_BUT_IDX_MASK, 60);
	pio_handler_set(FOUR_BUT_PIO,FOUR_BUT_PIO_ID,FOUR_BUT_IDX_MASK,PIO_IT_FALL_EDGE, four_button_callback);// ALTERAÇÃO DE PIO_IT_FALL_EDGE PARA PIO_IT_RISE_EDGE
	pio_enable_interrupt(FOUR_BUT_PIO, FOUR_BUT_IDX_MASK);
	pio_get_interrupt_status(FOUR_BUT_PIO);
	NVIC_EnableIRQ(FOUR_BUT_PIO_ID);
	NVIC_SetPriority(FOUR_BUT_PIO_ID, 4);
}

/************************************************************************/
/* configure console                                                    */
/************************************************************************/


static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
		#endif
		.paritytype = CONF_UART_PARITY,
		#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
		#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	#if defined(__GNUC__)
	setbuf(stdout, NULL);
	#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	* emits one character at a time.
	*/
	#endif
}


/************************************************************************/
/* usart things                                                         */
/************************************************************************/

uint32_t usart_puts(uint8_t *pstring) {
	uint32_t i ;

	while(*(pstring + i))
	if(uart_is_tx_empty(USART_COM))
	usart_serial_putchar(USART_COM, *(pstring+i++));
}

void usart_put_string(Usart *usart, char str[]) {
	usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, uint timeout_ms) {
	uint timecounter = timeout_ms;
	uint32_t rx;
	uint32_t counter = 0;

	while( (timecounter > 0) && (counter < bufferlen - 1)) {
		if(usart_read(usart, &rx) == 0) {
			buffer[counter++] = rx;
		}
		else{
			timecounter--;
			vTaskDelay(1);
		}
	}
	buffer[counter] = 0x00;
	return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen,
char buffer_tx[], int timeout) {
	usart_put_string(usart, buffer_tx);
	usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

void config_usart0(void) {
	sysclk_enable_peripheral_clock(ID_USART0);
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART0, &config);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);

	// RX - PB0  TX - PB1
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

int hc05_init(void) {
	char buffer_rx[128];
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+NAMEControle", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+PIN0000", 100);
}

/************************************************************************/
/* Taks                                                                */
/************************************************************************/

void task_bluetooth(void) {
	config_usart0();
	hc05_init();

	io_init();
	
	//printf("\n ---starting... ---\n");

	char button1 = '0';
	char button2 = '2';
	char button3 = '4';
	char button4 = '6';
	char eof = 'X';

	while(1) {
		/************************************************************************/
		/* ONE_BUT                                                              */
		/************************************************************************/
		if(pio_get(ONE_BUT_PIO, PIO_INPUT, ONE_BUT_IDX_MASK) == 0) {
			button1 = '1';
		} else {
			button1 = '0';
		}
		
		while(!usart_is_tx_ready(USART_COM)) {
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		usart_write(USART_COM, button1);
		

		while(!usart_is_tx_ready(USART_COM)) {
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		usart_write(USART_COM, eof);
		/************************************************************************/
		/* TWO_BUT                                                              */
		/************************************************************************/
		if(pio_get(TWO_BUT_PIO, PIO_INPUT, TWO_BUT_IDX_MASK) == 0) {
			button2 = '3';
			} else {
			button2 = '2';
		}

		while(!usart_is_tx_ready(USART_COM)) {
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		usart_write(USART_COM, button2);
		

		while(!usart_is_tx_ready(USART_COM)) {
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		usart_write(USART_COM, eof);
		
		/************************************************************************/
		/* THREE_BUT                                                              */
		/************************************************************************/
		if(pio_get(THREE_BUT_PIO, PIO_INPUT, THREE_BUT_IDX_MASK) == 0) {
			button3 = '5';
			} else {
			button3 = '4';
		}

		while(!usart_is_tx_ready(USART_COM)) {
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		usart_write(USART_COM, button3);
		

		while(!usart_is_tx_ready(USART_COM)) {
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		usart_write(USART_COM, eof);	
		
		/************************************************************************/
		/* FOUR_BUT                                                             */
		/************************************************************************/
		if(pio_get(FOUR_BUT_PIO, PIO_INPUT, FOUR_BUT_IDX_MASK) == 0) {
			button4 = '7';
			} else {
			button4 = '6';
		}

		while(!usart_is_tx_ready(USART_COM)) {
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		usart_write(USART_COM, button4);
		

		while(!usart_is_tx_ready(USART_COM)) {
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		usart_write(USART_COM, eof);	
		
		/************************************************************************/
		/* TASK DELAY                                                           */
		/************************************************************************/
		
		// dorme por 500 ms
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}


static void task_one_button(void *pvParameters) {
	one_button_init();
	for(;;){
		if (xSemaphoreTake(xSemaphoreOneButton, 1000)) {
			printf("1\n");
		}
	}
}

static void task_two_button(void *pvParameters) {
	two_button_init();
	for(;;){
		if (xSemaphoreTake(xSemaphoreTwoButton, 1000)) {
			printf("2\n");
		}
	}
}

static void task_three_button(void *pvParameters) {
	three_button_init();
	for(;;){
		if (xSemaphoreTake(xSemaphoreThreeButton, 1000)) {
			printf("3\n");
		}
	}
}

static void task_four_button(void *pvParameters) {
	four_button_init();
	for(;;){
		if (xSemaphoreTake(xSemaphoreFourButton, 1000)) {
			printf("4\n");
		}
	}
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	
	// Desactive watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;

	board_init();

	configure_console();
	
	/************************************************************************/
	/* semaphores                                                           */
	/************************************************************************/

	 
// 	xSemaphoreOneButton = xSemaphoreCreateBinary();
// 	if (xSemaphoreOneButton == NULL)
// 	printf("falha em criar xSemaphoreOneButton \n");
// 
// 	xSemaphoreTwoButton = xSemaphoreCreateBinary();
// 	if (xSemaphoreTwoButton == NULL)
// 	printf("falha em criar xSemaphoreTwoButton \n");
// 
// 	xSemaphoreThreeButton = xSemaphoreCreateBinary();
// 	if (xSemaphoreThreeButton == NULL)
// 	printf("falha em criar xSemaphoreThreeButton \n");
// 	
// 	xSemaphoreFourButton = xSemaphoreCreateBinary();
// 	if (xSemaphoreFourButton == NULL)
// 	printf("falha em criar xSemaphoreThreeButton \n");
		
	/************************************************************************/
	/* task bluetooh                                                        */
	/************************************************************************/
	
	/* Create task to make led blink */
	xTaskCreate(task_bluetooth, "BLT", TASK_BLUETOOTH_STACK_SIZE, NULL,	TASK_BLUETOOTH_STACK_PRIORITY, NULL);

	/************************************************************************/
	/* task buttons                                                         */
	/************************************************************************/

//   if (xTaskCreate(task_one_button, "One Button", TASK_BLUETOOTH_STACK_SIZE, NULL,TASK_BLUETOOTH_STACK_PRIORITY, NULL) != pdPASS) {
// 	  printf("Failed to create task_one_button\r\n");
// 	  } else {
// 	  printf("task_one_button created \r\n");
// 	  }
// 	  
//   if (xTaskCreate(task_two_button, "Two Button", TASK_BLUETOOTH_STACK_SIZE, NULL,TASK_BLUETOOTH_STACK_PRIORITY, NULL) != pdPASS) {
// 	  printf("Failed to create task_two_button\r\n");
// 	  } else {
// 	  printf("task_two_button created \r\n");
//   }
// 
//   if (xTaskCreate(task_three_button, "Three Button", TASK_BLUETOOTH_STACK_SIZE, NULL,TASK_BLUETOOTH_STACK_PRIORITY, NULL) != pdPASS) {
// 	  printf("Failed to create task_three_button\r\n");
// 	  } else {
// 	  printf("task_three_button created \r\n");
//   }
//   
//   if (xTaskCreate(task_four_button, "Four Button", TASK_BLUETOOTH_STACK_SIZE, NULL,TASK_BLUETOOTH_STACK_PRIORITY, NULL) != pdPASS) {
// 	  printf("Failed to create task_four_button\r\n");
// 	  } else {
// 	  printf("task_four_button created \r\n");
//   }
// 	 	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
