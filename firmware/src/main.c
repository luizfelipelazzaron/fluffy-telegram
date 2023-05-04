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
/* button defines                                                       */
/************************************************************************/

// LEDs
#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

// One Button
#define ONE_BUT_PIO      PIOA
#define ONE_BUT_PIO_ID   ID_PIOA
#define ONE_BUT_IDX      6
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
// Descomente pa
// pela serial debug

#define DEBUG_SERIAL //comente para ativar o bluetooth na placa; não esqueça de mudar de 115200->960 0

#ifdef DEBUG_SERIAL
#define USART_COM USART1
#define USART_COM_ID ID_USART1
#else
#define USART_COM USART0
#define USART_COM_ID ID_USART0
#endif

/************************************************************************/
/* afec defines                                                         */
/************************************************************************/
#define AFEC_POT_Y AFEC0
#define AFEC_POT_ID_Y ID_AFEC0
#define AFEC_POT_CHANNEL_Y 0 // Canal do pino PD30

#define AFEC_POT_X AFEC0
#define AFEC_POT_ID_X ID_AFEC0
#define AFEC_POT_CHANNEL_X 5 // Canal do pino PB2

/************************************************************************/
/* QUEUES AFEC                                                          */
/************************************************************************/


TimerHandle_t xTimer_Y;

TimerHandle_t xTimer_X;

QueueHandle_t xQueueMain;

typedef struct {
	volatile char value;
} Data;

volatile Data oldData;

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
/* local prototypes                                                     */
/************************************************************************/

static void config_AFEC_pot(void);

void AFEC_pot_Y_callback(void);
void AFEC_pot_X_callback(void);

void one_button_callback(void);
void two_button_callback(void);
void three_button_callback(void);
void four_button_callback(void);

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
void AFEC_pot_Y_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;

	volatile Data data;
	
	int afecValue = (int) afec_channel_get_value(AFEC_POT_Y, AFEC_POT_CHANNEL_Y);
	
	if (afecValue > 3000) {
		data.value = 'U'; //UP +
		
	} else if (afecValue < 1000){
		data.value = 'D'; // DOWN -
		
	} else {
		data.value = 'B'; // BOTTON
	}
	
	if (data.value == 'B'){
		if (oldData.value != 'C' && oldData.value != 'B'){
			xQueueSendFromISR(xQueueMain, &data, &xHigherPriorityTaskWoken);
		}
	} else {
		xQueueSendFromISR(xQueueMain, &data, &xHigherPriorityTaskWoken);
	}
}

void AFEC_pot_X_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;

	volatile Data data;
	
	int afecValue = (int)  afec_channel_get_value(AFEC_POT_X, AFEC_POT_CHANNEL_X);
	
	if (afecValue > 3000) {
		data.value = 'L'; //LEFT +
		
		} else if (afecValue < 1000){
		data.value = 'R'; // LEFT -
		
		} else {
		data.value = 'C'; // CENTER
	}
	
	if (data.value == 'C'){
		if (oldData.value != 'B' && oldData.value != 'C'){
			xQueueSendFromISR(xQueueMain, &data, &xHigherPriorityTaskWoken);
		}
		} else {
		xQueueSendFromISR(xQueueMain, &data, &xHigherPriorityTaskWoken);
	}
}

void one_button_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	Data data;
	data.value = '0';
	
	if(pio_get(ONE_BUT_PIO, PIO_INPUT, ONE_BUT_IDX_MASK) == 0) {
		data.value = '1';
	} else {
		data.value = '0';
	}
	xQueueSendFromISR(xQueueMain, &data, &xHigherPriorityTaskWoken);
}

void two_button_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	Data data;
	data.value = '2';
	
	if(pio_get(TWO_BUT_PIO, PIO_INPUT, TWO_BUT_IDX_MASK) == 0) {
		data.value = '3';
	} else {
		data.value = '2';
	}
	
	xQueueSendFromISR(xQueueMain, &data, &xHigherPriorityTaskWoken);
}

void three_button_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	Data data;
	data.value = '4';
	
	if(pio_get(THREE_BUT_PIO, PIO_INPUT, THREE_BUT_IDX_MASK) == 0) {
		data.value = '5';
		} else {
		data.value = '4';
	}

	xQueueSendFromISR(xQueueMain, &data, &xHigherPriorityTaskWoken);
}

void four_button_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	Data data;
	data.value = '6';
	
	if(pio_get(FOUR_BUT_PIO, PIO_INPUT, FOUR_BUT_IDX_MASK) == 0) {
		data.value = '7';
		} else {
		data.value = '6';
	}

	xQueueSendFromISR(xQueueMain, &data, &xHigherPriorityTaskWoken);
}


void TC0_Handler(void){
	volatile uint32_t ul_dummy;

	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);
	
	afec_channel_enable(AFEC_POT_Y, AFEC_POT_CHANNEL_Y);
	afec_start_software_conversion(AFEC_POT_Y);
	
	afec_channel_enable(AFEC_POT_X, AFEC_POT_CHANNEL_X);
	afec_start_software_conversion(AFEC_POT_X);
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
	
	pio_configure(ONE_BUT_PIO, PIO_INPUT, ONE_BUT_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(ONE_BUT_PIO, ONE_BUT_IDX_MASK, 60);
	
	pio_configure(TWO_BUT_PIO, PIO_INPUT, TWO_BUT_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(TWO_BUT_PIO, TWO_BUT_IDX_MASK, 60);
	
	pio_configure(THREE_BUT_PIO, PIO_INPUT, THREE_BUT_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(THREE_BUT_PIO, THREE_BUT_IDX_MASK, 60);
	
	pio_configure(FOUR_BUT_PIO, PIO_INPUT, FOUR_BUT_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(FOUR_BUT_PIO, FOUR_BUT_IDX_MASK, 60);
	
	
	pio_handler_set(ONE_BUT_PIO,ONE_BUT_PIO_ID,ONE_BUT_IDX_MASK,PIO_IT_EDGE, one_button_callback);// ALTERAÇÃO DE PIO_IT_FALL_EDGE PARA PIO_IT_RISE_EDGE
	pio_handler_set(TWO_BUT_PIO,TWO_BUT_PIO_ID,TWO_BUT_IDX_MASK,PIO_IT_EDGE, two_button_callback);// ALTERAÇÃO DE PIO_IT_FALL_EDGE PARA PIO_IT_RISE_EDGE
	pio_handler_set(THREE_BUT_PIO,THREE_BUT_PIO_ID,THREE_BUT_IDX_MASK,PIO_IT_EDGE, three_button_callback);// ALTERAÇÃO DE PIO_IT_FALL_EDGE PARA PIO_IT_RISE_EDGE
	pio_handler_set(FOUR_BUT_PIO,FOUR_BUT_PIO_ID,FOUR_BUT_IDX_MASK,PIO_IT_EDGE, four_button_callback);// ALTERAÇÃO DE PIO_IT_FALL_EDGE PARA PIO_IT_RISE_EDGE

	pio_enable_interrupt(ONE_BUT_PIO, ONE_BUT_IDX_MASK);
	pio_enable_interrupt(TWO_BUT_PIO, TWO_BUT_IDX_MASK);
	pio_enable_interrupt(THREE_BUT_PIO, THREE_BUT_IDX_MASK);
	pio_enable_interrupt(FOUR_BUT_PIO, FOUR_BUT_IDX_MASK);

	
	pio_get_interrupt_status(ONE_BUT_PIO);
	pio_get_interrupt_status(TWO_BUT_PIO);
	pio_get_interrupt_status(THREE_BUT_PIO);
	pio_get_interrupt_status(FOUR_BUT_PIO);

	NVIC_EnableIRQ(ONE_BUT_PIO_ID);
	NVIC_EnableIRQ(TWO_BUT_PIO_ID);
	NVIC_EnableIRQ(THREE_BUT_PIO_ID);
	NVIC_EnableIRQ(FOUR_BUT_PIO_ID);

	NVIC_SetPriority(ONE_BUT_PIO_ID, 4);
	NVIC_SetPriority(TWO_BUT_PIO_ID, 4);
	NVIC_SetPriority(THREE_BUT_PIO_ID, 4);
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

static void config_AFEC_pot(void) {
	
	afec_enable(AFEC0);
  
  /* struct de configuracao do AFEC */
  struct afec_config afec_cfg;

  /* Carrega parametros padrao */
  afec_get_config_defaults(&afec_cfg);

  /* Configura AFEC */
  afec_init(AFEC_POT_Y, &afec_cfg);
  afec_init(AFEC_POT_X, &afec_cfg);

  /*** Configuracao específica do canal AFEC ***/
  struct afec_ch_config afec_ch_cfg;
  afec_ch_get_config_defaults(&afec_ch_cfg);
  afec_ch_cfg.gain = AFEC_GAINVALUE_0;
  
  afec_ch_set_config(AFEC_POT_Y, AFEC_POT_CHANNEL_Y, &afec_ch_cfg);
  afec_ch_set_config(AFEC_POT_X, AFEC_POT_CHANNEL_X, &afec_ch_cfg);
  
  /*
  * Calibracao:
  * Because the internal ADC offset is 0x200, it should cancel it and shift
  down to 0.
  */
  afec_channel_set_analog_offset(AFEC_POT_Y, AFEC_POT_CHANNEL_Y, 0x200);
  afec_channel_set_analog_offset(AFEC_POT_X, AFEC_POT_CHANNEL_X, 0x200);
  
 
   afec_set_callback(AFEC_POT_Y, AFEC_POT_CHANNEL_Y, AFEC_pot_Y_callback, 1);
   afec_set_callback(AFEC_POT_X, AFEC_POT_CHANNEL_X, AFEC_pot_X_callback, 1);
   
   NVIC_SetPriority(AFEC_POT_ID_Y, 4);
   NVIC_EnableIRQ(AFEC_POT_ID_Y);
   
	NVIC_SetPriority(AFEC_POT_ID_X, 4);
	NVIC_EnableIRQ(AFEC_POT_ID_X);
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	uint32_t channel = 1;

	/* Configura o PMC */
	/* O TimerCounter é meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);

	tc_init(TC, TC_CHANNEL, ul_tcclks
							| TC_CMR_WAVE /* Waveform mode is enabled */
							| TC_CMR_ACPA_SET /* RA Compare Effect: set */
							| TC_CMR_ACPC_CLEAR /* RC Compare Effect: clear */
							| TC_CMR_CPCTRG /* UP mode with automatic trigger on RC Compare */
	);

	tc_write_ra(TC, TC_CHANNEL, 2*65532/3);
	tc_write_rc(TC, TC_CHANNEL, 3*65532/3);

	//tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupçcão no TC canal 0 */
	/* Interrupção no C */
#ifdef DEBUG
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
#endif
	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
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
/* Tasks                                                                */
/************************************************************************/

void task_bluetooth(void) {
	config_usart0();
	hc05_init();
	io_init();
	
	config_AFEC_pot();
	
	TC_init(TC0, ID_TC0, 0, 3);
	tc_start(TC0, 0);

   
	volatile Data data;
	char eof = 'X';

	while(1) {
		if (xQueueReceive(xQueueMain, &data, 0)) {
					while(!usart_is_tx_ready(USART_COM)) {
						vTaskDelay(10 / portTICK_PERIOD_MS);
					}
					usart_write(USART_COM, data.value);
					
					while(!usart_is_tx_ready(USART_COM)) {
						vTaskDelay(10 / portTICK_PERIOD_MS);
					}
					usart_write(USART_COM, eof);
					
					oldData.value = data.value;
		} 
		
		// dorme por 500 ms
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}


/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;
	board_init();
	configure_console();
	
	xQueueMain = xQueueCreate(100, sizeof(Data));
	
	if (xQueueMain == NULL){
		printf("xQueueMain failure\n");
	}
	
	/* Create task to make led blink */
	xTaskCreate(task_bluetooth, "BLT", TASK_BLUETOOTH_STACK_SIZE, NULL,	TASK_BLUETOOTH_STACK_PRIORITY, NULL);

// 	 	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
