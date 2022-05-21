#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"



// Definindo tudo do botão 1:
#define BUT1_PIO PIOD
#define BUT1_PIO_ID ID_PIOD
#define BUT1_PIO_IDX 28
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX) // esse já está pronto.

// Definindo tudo do LED 1:
#define LED1_PIO           PIOA                 // periferico que controla o LED
// #
#define LED1_PIO_ID        ID_PIOA                 // ID do periférico PIOC (controla LED)
#define LED1_PIO_IDX       0                    // ID do LED no PIO
#define LED1_PIO_IDX_MASK  (1 << LED1_PIO_IDX)   // Mascara para CONTROLARMOS o LED

// Definindo tudo do botão 2:
#define BUT2_PIO PIOC
#define BUT2_PIO_ID ID_PIOC
#define BUT2_PIO_IDX 31
#define BUT2_PIO_IDX_MASK (1u << BUT2_PIO_IDX) // esse já está pronto.

// Definindo tudo do LED 2:
#define LED2_PIO           PIOC                 // periferico que controla o LED
// #
#define LED2_PIO_ID        ID_PIOC                 // ID do periférico PIOC (controla LED)
#define LED2_PIO_IDX       30                    // ID do LED no PIO
#define LED2_PIO_IDX_MASK  (1 << LED2_PIO_IDX)   // Mascara para CONTROLARMOS o LED

// Definindo tudo do botão 3:
#define BUT3_PIO PIOA
#define BUT3_PIO_ID ID_PIOA
#define BUT3_PIO_IDX 19
#define BUT3_PIO_IDX_MASK (1u << BUT3_PIO_IDX) // esse já está pronto.

// Definindo tudo do LED 3:
#define LED3_PIO           PIOB                 // periferico que controla o LED
// #
#define LED3_PIO_ID        ID_PIOB                 // ID do periférico PIOC (controla LED)
#define LED3_PIO_IDX       2                    // ID do LED no PIO
#define LED3_PIO_IDX_MASK  (1 << LED3_PIO_IDX)   // Mascara para CONTROLARMOS o LED

#define LED_PIO						PIOC
#define LED_PIO_ID					ID_PIOC
#define LED_IDX						8
#define LED_IDX_MASK				(1 << LED_IDX)


// AV2
#include "coffee/coffee.h"

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_ADC_STACK_SIZE (1024*10 / sizeof(portSTACK_TYPE))
#define TASK_ADC_STACK_PRIORITY (tskIDLE_PRIORITY)

#define USART_COM_ID ID_USART1
#define USART_COM USART1

#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 0 // Canal do pino PD30

/************************************************************************/
/* glboals                                                               */
/************************************************************************/
QueueHandle_t xQueueADC;
volatile int cafe_sucesso = 0;
volatile int temp;
volatile int i = 0;
volatile int j = 0;
volatile int but3_flag = 0;
SemaphoreHandle_t xSemaphoreBut1;
SemaphoreHandle_t xSemaphoreBut2;
SemaphoreHandle_t xSemaphoreBut3;

typedef struct {
	uint value;
} adcData;


/************************************************************************/
/* PROTOtypes                                                           */
/************************************************************************/

static void USART1_init(void);
void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq);
static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel, afec_callback_t callback);
static void configure_console(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) { configASSERT( ( volatile void * ) NULL ); }

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/


void TC1_Handler(void) {
	volatile uint32_t ul_dummy;

	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/* Selecina canal e inicializa conversão */
	afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
	afec_start_software_conversion(AFEC_POT);
}



void RTT_Handler(void) {
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
	}

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		cafe_sucesso = 1;
	}
}


void TC4_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC1, 1);
	i++;
	
}

void TC0_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC0, 0);
	j++;
}

static void AFEC_pot_Callback(void) {
	adcData adc;
	adc.value = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueueADC, &adc, &xHigherPriorityTaskWoken);
}

void but1_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreBut1, &xHigherPriorityTaskWoken);
}

void but2_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreBut2, &xHigherPriorityTaskWoken);
}

void but3_callback(void){
	but3_flag = 1;
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_adc(void *pvParameters) {

	// configura ADC e TC para controlar a leitura
	config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_Callback);
	TC_init(TC0, ID_TC1, 1, 10);
	tc_start(TC0, 1);

	// variável para recever dados da fila
	adcData adc;

	while (1) {
		if (xQueueReceive(xQueueADC, &(adc), 1000)) {
			temp = ((adc.value * 100) / 4095);
			printf("ADC: %d \n", temp);
		} else {
			printf("Nao chegou um novo dado em 1 segundo");
		}
	}
}

int confere_temp(int pronto){
	char temp_str[20];

	while (temp < 80){
		sprintf(temp_str, "Aquecendo. Temp: %d C", temp);
		gfx_mono_draw_string(temp_str, 0,0, &sysfont);
		if ((xSemaphoreTake(xSemaphoreBut1, 10 / portTICK_PERIOD_MS) && !pronto) || (xSemaphoreTake(xSemaphoreBut2, 10 / portTICK_PERIOD_MS) && !pronto)){
			gfx_mono_draw_string("Calma, nao ta pronto", 0,20, &sysfont);
		}
	}
		
	if (temp >= 80){
		gfx_mono_generic_draw_filled_rect(0, 0, 127, 31, GFX_PIXEL_CLR);
		gfx_mono_draw_string("Pronto!", 0,0, &sysfont);
		pronto = 1;
	}
	
	return pronto;
}


static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
	//gfx_mono_draw_string("Exemplo RTOS", 0, 0, &sysfont);
	//gfx_mono_draw_string("oii", 0, 20, &sysfont);	
	
	int pronto = 0;
	
	coffee_heat_on();
	
	int longo = 0;
	int curto = 0;
	int inativo = 0;
		
	for (;;)  {
		if (temp < 80 && !inativo){
			gfx_mono_generic_draw_filled_rect(0, 0, 127, 31, GFX_PIXEL_CLR);
			pronto = confere_temp(pronto);
		} else if (temp >= 80){
			pronto = confere_temp(pronto);
		}
		
		int contagem = 0; 
		while (pronto){
			
			if (!contagem){
				contagem = 1;
				i = 0;
				TC_init(TC1, ID_TC4, 1, 1);
				tc_start(TC1, 1);
			}
			
			if (i == 20){
				inativo = 1;
				pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
				pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
				pio_set(LED3_PIO, LED3_PIO_IDX_MASK);				
				gfx_mono_generic_draw_filled_rect(0, 0, 127, 31, GFX_PIXEL_CLR);
				gfx_mono_draw_string("Inativo...", 0,0, &sysfont);
				coffee_heat_off();
			}

			if ((xSemaphoreTake(xSemaphoreBut1, 10 / portTICK_PERIOD_MS)) && !longo){
				if (inativo){
					inativo = 0;
					pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
					pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
					pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
					coffee_heat_on();
					if (temp < 80 && !inativo){
						gfx_mono_generic_draw_filled_rect(0, 0, 127, 31, GFX_PIXEL_CLR);
						pronto = confere_temp(pronto);
					}
				}
				curto = 1;
				gfx_mono_generic_draw_filled_rect(0, 0, 127, 31, GFX_PIXEL_CLR);
				gfx_mono_draw_string("Fazendo cafe...", 0,0, &sysfont);
				gfx_mono_draw_string("Cafe curto", 0,20, &sysfont);
				coffe_pump_on();

				gfx_mono_generic_draw_filled_rect(81, 20, 19, 9, GFX_PIXEL_CLR);
				gfx_mono_generic_draw_rect(80, 19, 20, 10, GFX_PIXEL_SET);
				RTT_init(1, 5, RTT_MR_ALMIEN);
				TC_init(TC1, ID_TC4, 1, 1);
				tc_start(TC1, 1);
				for (i = 0; i < 5;){
					gfx_mono_generic_draw_filled_rect(80+4*i, 19, 4, 10, GFX_PIXEL_SET);
				}
			} if ((xSemaphoreTake(xSemaphoreBut2, 10 / portTICK_PERIOD_MS)) && !curto){
				if (inativo){
					inativo = 0;
					pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
					pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
					pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
					coffee_heat_on();
					if (temp < 80 && !inativo){
						gfx_mono_generic_draw_filled_rect(0, 0, 127, 31, GFX_PIXEL_CLR);
						pronto = confere_temp(pronto);
					}
				}
				longo = 1;
				gfx_mono_generic_draw_filled_rect(0, 0, 127, 31, GFX_PIXEL_CLR);
				gfx_mono_draw_string("Fazendo cafe...", 0,0, &sysfont);
				gfx_mono_draw_string("Cafe longo", 0,20, &sysfont);
				coffe_pump_on();
				gfx_mono_generic_draw_filled_rect(81, 20, 19, 9, GFX_PIXEL_CLR);
				gfx_mono_generic_draw_rect(80, 19, 20, 10, GFX_PIXEL_SET);				
				RTT_init(1, 10, RTT_MR_ALMIEN);
				TC_init(TC1, ID_TC4, 1, 1);
				tc_start(TC1, 1);
				for (i = 0; i < 10;){
					gfx_mono_generic_draw_filled_rect(80+2*i, 19, 2, 10, GFX_PIXEL_SET);
				}
			} if (cafe_sucesso){
				cafe_sucesso = 0;
				coffe_pump_off();
				gfx_mono_generic_draw_filled_rect(0, 0, 127, 31, GFX_PIXEL_CLR);
				gfx_mono_draw_string("Feito, aproveite", 0,0, &sysfont);
				contagem = 0;
				vTaskDelay(100);
				pronto = 0;
				curto = 0;
				longo = 0;
			}
	    }
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}



static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
                            afec_callback_t callback) {
  /*************************************
   * Ativa e configura AFEC
   *************************************/
  /* Ativa AFEC - 0 */
  afec_enable(afec);

  /* struct de configuracao do AFEC */
  struct afec_config afec_cfg;

  /* Carrega parametros padrao */
  afec_get_config_defaults(&afec_cfg);

  /* Configura AFEC */
  afec_init(afec, &afec_cfg);

  /* Configura trigger por software */
  afec_set_trigger(afec, AFEC_TRIG_SW);

  /*** Configuracao específica do canal AFEC ***/
  struct afec_ch_config afec_ch_cfg;
  afec_ch_get_config_defaults(&afec_ch_cfg);
  afec_ch_cfg.gain = AFEC_GAINVALUE_0;
  afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

  /*
  * Calibracao:
  * Because the internal ADC offset is 0x200, it should cancel it and shift
  down to 0.
  */
  afec_channel_set_analog_offset(afec, afec_channel, 0x200);

  /***  Configura sensor de temperatura ***/
  struct afec_temp_sensor_config afec_temp_sensor_cfg;

  afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
  afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);

  /* configura IRQ */
  afec_set_callback(afec, afec_channel, callback, 1);
  NVIC_SetPriority(afec_id, 4);
  NVIC_EnableIRQ(afec_id);
}

void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq) {
  uint32_t ul_div;
  uint32_t ul_tcclks;
  uint32_t ul_sysclk = sysclk_get_cpu_hz();

  pmc_enable_periph_clk(ID_TC);

  tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
  tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
  tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

  NVIC_SetPriority((IRQn_Type)ID_TC, 4);
  NVIC_EnableIRQ((IRQn_Type)ID_TC);
  tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}


static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}


void configure_pio_input(Pio *pio, const pio_type_t ul_type, const uint32_t ul_mask, const uint32_t ul_attribute, uint32_t ul_id){
	pmc_enable_periph_clk(ul_id);
	pio_configure(pio, ul_type, ul_mask, ul_attribute);
	pio_set_debounce_filter(pio, ul_mask, 60);
}

void configure_interruption(Pio *pio, uint32_t ul_id, const uint32_t ul_mask,  uint32_t ul_attr, void (*p_handler) (uint32_t, uint32_t), uint32_t priority){
	pio_handler_set(pio, ul_id, ul_mask , ul_attr, p_handler);
	pio_enable_interrupt(pio, ul_mask);
	pio_get_interrupt_status(pio);
	NVIC_EnableIRQ(ul_id);
	NVIC_SetPriority(ul_id, priority);
}


void io_init(void) {

	// Ativa PIOs
	pmc_enable_periph_clk(LED_PIO_ID);
	pmc_enable_periph_clk(LED1_PIO_ID);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pmc_enable_periph_clk(LED3_PIO_ID);

	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);

	// Configura Pinos
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);

	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_PIO_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_PIO_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
	pio_configure(LED3_PIO, PIO_OUTPUT_0, LED3_PIO_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);


	configure_pio_input(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP|PIO_DEBOUNCE, BUT1_PIO_ID);
	configure_interruption(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_EDGE, but1_callback, 4);
	
	configure_pio_input(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP|PIO_DEBOUNCE, BUT2_PIO_ID);
	configure_interruption(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_EDGE, but2_callback, 4);
	
	configure_pio_input(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP|PIO_DEBOUNCE, BUT3_PIO_ID);
	configure_interruption(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_EDGE, but3_callback, 4);
	
}



/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	WDT->WDT_MR = WDT_MR_WDDIS;
	io_init();

	/* Initialize the console uart */
	configure_console();
	
	
	
	xSemaphoreBut1 = xSemaphoreCreateBinary();
	if (xSemaphoreBut1 == NULL)
	printf("falha em criar o semaforo \n");
	
	xSemaphoreBut2 = xSemaphoreCreateBinary();
	if (xSemaphoreBut2 == NULL)
	printf("falha em criar o semaforo \n");

	xSemaphoreBut3 = xSemaphoreCreateBinary();
	if (xSemaphoreBut3 == NULL)
	printf("falha em criar o semaforo \n");
		
	xQueueADC = xQueueCreate(100, sizeof(adcData));
	if (xQueueADC == NULL)
	printf("falha em criar a queue xQueueADC \n");

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}
	
	if (xTaskCreate(task_av2, "av2", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create oled task\r\n");
	}	

	if (xTaskCreate(task_adc, "ADC", TASK_ADC_STACK_SIZE, NULL,
	TASK_ADC_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test ADC task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
