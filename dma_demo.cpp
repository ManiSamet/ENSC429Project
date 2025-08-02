#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "mcp4725/mcp4725.hpp"


#define ONBOARD_LED 25
#define ONBOARD_LED_SLICE 4
#define ONBOARD_LED_CHANNEL PWM_CHAN_B

#define EXTERNAL_LED 22
#define EXTERNAL_LED_SLICE 3 // pg 524 rp2040 datasheet
#define EXTERNAL_LED_CHANNEL PWM_CHAN_A

#define INDICATOR EXTERNAL_LED
#define INDICATOR_SLICE EXTERNAL_LED_SLICE

#define OUTPUT_LED ONBOARD_LED
#define OUTPUT_SLICE ONBOARD_LED_SLICE
#define OUTPUT_CHANNEL ONBOARD_LED_CHANNEL

#define POT_PIN 27
#define POT_CHANNEL 1

// matched with the max adc value so that the raw adc
// value directly maps to a PWM duty cycle
#define PWM_MAX_COUNT 4095 

#define ADC_RESOLUTION 12
#define ADC_IRQ_FIFO 22 // pg 60 rp2040 datasheet

#define SAMPLE_SIZE 256

volatile uint16_t sample_buffer[2][SAMPLE_SIZE];
volatile bool flag = false;
volatile uint8_t buffer = 0;


/*I2C Interface*/
uint8_t SDA = 4; //GP4
uint8_t SCL = 5; //GP5
uint32_t timeout = 5000; // 5 mSec timeout
i2c_inst_t* i2c = i2c0;

/*MCP4725 DAC*/
MCP4725_PICO dac(3.3); //External DAC set to 3.3V reference voltage


void init_led(){
	gpio_init(EXTERNAL_LED);
	gpio_set_dir(EXTERNAL_LED, GPIO_OUT);

	gpio_init(ONBOARD_LED);
	gpio_set_dir(ONBOARD_LED, GPIO_OUT);
}

void init_pwm(){

	gpio_set_function(OUTPUT_LED, GPIO_FUNC_PWM);

	// set to slowest frequency possible
	pwm_set_clkdiv_int_frac(OUTPUT_SLICE, 255, 0);

	// set wrap value to 100 so that channel value 
	// level directly corresponds to duty cycle
	pwm_set_wrap(OUTPUT_SLICE, PWM_MAX_COUNT);
	pwm_set_chan_level(OUTPUT_SLICE, OUTPUT_CHANNEL, 0);

	pwm_set_enabled(OUTPUT_SLICE, true);
}
 
void adc_callback();
 
void init_adc(){

	adc_init();

	adc_gpio_init(POT_PIN);

	adc_select_input(POT_CHANNEL);

	adc_fifo_setup( 
		true, // enable FIFO
		false, // disable DMA
		1, // trigger irq on one sample
		false, // disable error sample flag
		false // doesnt shift down to 8 bytes
	);

	adc_set_clkdiv(96.0f); // arbitrary large value to be slow enough

	// enable interrupt

	adc_irq_set_enabled(true);

	irq_set_exclusive_handler(ADC_IRQ_FIFO, adc_callback);

	irq_set_enabled(ADC_IRQ_FIFO, true);
}


int main(){

	stdio_init_all();
	init_led();
	// init_pwm();
	init_adc();

	if(!dac.begin(MCP4725_PICO::MCP4725A0_Addr_A00, i2c, 400, SDA, SCL, timeout)){
		printf("Initialization failed\n");
		return 1;
	}
	// gpio_set_mask((1 << EXTERNAL_LED));

	// dac.setInputCode(4090, MCP4725_PICO::MCP4725_FastMode, MCP4725_PICO::MCP4725_PowerDown_Off);
	// dac.setVoltage(3.3, MCP4725_PICO::MCP4725_FastMode, MCP4725_PICO::MCP4725_PowerDown_Off);	
	// printf("MCP4725 Initialized\n");


	adc_run(true);
	while (1){

		if (flag){
			flag = false;
			// __disable_irq(); 
			uint32_t sum = 0;
			int buffer_number = buffer;
			buffer = !buffer;

			// for (int i = 0; i < SAMPLE_SIZE; i++){
			// 	sum += sample_buffer[buffer_number][i];
			// }
			// printf("Average ADC value: %u\n", sample_buffer[buffer_number][127]);
			dac.setInputCode(sample_buffer[buffer_number][127], MCP4725_PICO::MCP4725_FastMode, MCP4725_PICO::MCP4725_PowerDown_Off);
			// toggle gpio indicator to measure sampling rate
			// gpio_set_mask(1 << INDICATOR);

		}
		// tight_loop_contents();
	}
}

void adc_callback(){

	uint16_t raw_data = adc_fifo_get();
	adc_fifo_drain(); // resets event
	static uint8_t index = 0;
	

	sample_buffer[buffer][index] = raw_data;

	index++;
	if (index == SAMPLE_SIZE){
		index = 0;
		flag = true;
		// gpio_xor_mask(1 << INDICATOR); // toggle indicator
	}

	irq_clear(ADC_IRQ_FIFO);
}




