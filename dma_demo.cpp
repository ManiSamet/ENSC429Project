/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <math.h>

#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/structs/clocks.h"
#include "hardware/pio.h"
#include "hardware/adc.h"

#include "pico/stdlib.h"
#include "pico/audio.h"
#include "pico/audio_i2s.h"

#define SINE_WAVE_TABLE_LEN 2048
#define SAMPLES_PER_BUFFER 1156 // Samples / channel


static const uint32_t PIN_DCDC_PSM_CTRL = 23;

#define ADC_SAMPLE_SIZE 256
#define ADC_PIN 27
#define ADC_CHANNEL 1

/* ADC Interrupt variables*/
volatile uint16_t sample_buffer[2][ADC_SAMPLE_SIZE];
volatile bool adc_flag = false;
volatile uint8_t buffer = 0;
static uint16_t index = 0;
volatile bool buffer_full[2] = {false, false};

volatile bool switch_buffers = true;  //adc should startfist with this

audio_buffer_pool_t *ap;
static bool decode_flg = false;
static constexpr int32_t DAC_ZERO = 1;

#define audio_pio __CONCAT(pio, PICO_AUDIO_I2S_PIO)

static audio_format_t audio_format = {
    // .sample_freq = 20000,
    .sample_freq = 44100,
    .pcm_format = AUDIO_PCM_FORMAT_S16,
    .channel_count = AUDIO_CHANNEL_STEREO
};

static audio_buffer_format_t producer_format = {
    .format = &audio_format,
    .sample_stride = 8
};

static audio_i2s_config_t i2s_config = {
    .data_pin = PICO_AUDIO_I2S_DATA_PIN,
    .clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE,
    .dma_channel0 = 0,
    .dma_channel1 = 1,
    .pio_sm = 0
};

static int16_t sine_wave_table[SINE_WAVE_TABLE_LEN];
uint32_t step0 = 0x200000;
uint32_t step1 = 0x200000;
uint32_t pos0 = 0;
uint32_t pos1 = 0;
const uint32_t pos_max = 0x10000 * SINE_WAVE_TABLE_LEN;
uint vol = 20;

void adc_callback();
void decode();
extern "C" {
// callback from:
//   void __isr __time_critical_func(audio_i2s_dma_irq_handler)()
//   defined at my_pico_audio_i2s/audio_i2s.c
//   where i2s_callback_func() is declared with __attribute__((weak))
void i2s_callback_func()
{
    if (decode_flg && adc_flag) {
		adc_flag = false;
        decode();
    }
}
}
 
void init_adc(){

	adc_init();

	adc_gpio_init(ADC_PIN);
	adc_select_input(ADC_CHANNEL);

	adc_fifo_setup( 
		true, // enable FIFO
		false, // disable DMA
		1, // trigger irq on one sample
		false, // disable error sample flag
		false // doesnt shift down to 8 bytes
	);

	adc_set_clkdiv((48e6 / 44100) - 1);  

	// enable interrupt

	adc_irq_set_enabled(true);

	irq_set_exclusive_handler(ADC_IRQ_FIFO, adc_callback);

	irq_set_enabled(ADC_IRQ_FIFO, true);
}


#if 0
audio_buffer_pool_t *init_audio() {

    static audio_format_t audio_format = {
        .pcm_format = AUDIO_PCM_FORMAT_S32,
        .sample_freq = 44100,
        .channel_count = 2
    };

    static audio_buffer_format_t producer_format = {
        .format = &audio_format,
        .sample_stride = 8
    };

    audio_buffer_pool_t *producer_pool = audio_new_producer_pool(&producer_format, 3,
                                                                      SAMPLES_PER_BUFFER); // todo correct size
    bool __unused ok;
    const audio_format_t *output_format;
#if USE_AUDIO_I2S
    audio_i2s_config_t config = {
        .data_pin = PICO_AUDIO_I2S_DATA_PIN,
        .clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE,
        .dma_channel = 0,
        .pio_sm = 0
    };

    output_format = audio_i2s_setup(&audio_format, &audio_format, &config);
    if (!output_format) {
        panic("PicoAudio: Unable to open audio device.\n");
    }

    ok = audio_i2s_connect(producer_pool);
    assert(ok);
    { // initial buffer data
        audio_buffer_t *buffer = take_audio_buffer(producer_pool, true);
        int32_t *samples = (int32_t *) buffer->buffer->bytes;
        for (uint i = 0; i < buffer->max_sample_count; i++) {
            samples[i*2+0] = 0;
            samples[i*2+1] = 0;
        }
        buffer->sample_count = buffer->max_sample_count;
        give_audio_buffer(producer_pool, buffer);
    }
    audio_i2s_set_enabled(true);
#elif USE_AUDIO_PWM
    output_format = audio_pwm_setup(&audio_format, -1, &default_mono_channel_config);
    if (!output_format) {
        panic("PicoAudio: Unable to open audio device.\n");
    }
    ok = audio_pwm_default_connect(producer_pool, false);
    assert(ok);
    audio_pwm_set_enabled(true);
#elif USE_AUDIO_SPDIF
    output_format = audio_spdif_setup(&audio_format, &audio_spdif_default_config);
    if (!output_format) {
        panic("PicoAudio: Unable to open audio device.\n");
    }
    //ok = audio_spdif_connect(producer_pool);
    ok = audio_spdif_connect(producer_pool);
    assert(ok);
    audio_spdif_set_enabled(true);
#endif
    return producer_pool;
}
#endif

static inline uint32_t _millis(void)
{
	return to_ms_since_boot(get_absolute_time());
}

void i2s_audio_deinit()
{
    decode_flg = false;

    audio_i2s_set_enabled(false);
    audio_i2s_end();

    audio_buffer_t* ab;
    ab = take_audio_buffer(ap, false);
    while (ab != nullptr) {
        free(ab->buffer->bytes);
        free(ab->buffer);
        ab = take_audio_buffer(ap, false);
    }
    ab = get_free_audio_buffer(ap, false);
    while (ab != nullptr) {
        free(ab->buffer->bytes);
        free(ab->buffer);
        ab = get_free_audio_buffer(ap, false);
    }
    ab = get_full_audio_buffer(ap, false);
    while (ab != nullptr) {
        free(ab->buffer->bytes);
        free(ab->buffer);
        ab = get_full_audio_buffer(ap, false);
    }
    free(ap);
    ap = nullptr;
}

audio_buffer_pool_t *i2s_audio_init(uint32_t sample_freq)
{
    audio_format.sample_freq = sample_freq;

    audio_buffer_pool_t *producer_pool = audio_new_producer_pool(&producer_format, 3, SAMPLES_PER_BUFFER);
    ap = producer_pool;

    bool __unused ok;
    const audio_format_t *output_format;

    output_format = audio_i2s_setup(&audio_format, &audio_format, &i2s_config);
    if (!output_format) {
        panic("PicoAudio: Unable to open audio device.\n");
    }

    ok = audio_i2s_connect(producer_pool);
    assert(ok);
    { // initial buffer data
        audio_buffer_t *ab = take_audio_buffer(producer_pool, true);
        int32_t *samples = (int32_t *) ab->buffer->bytes;
        for (uint i = 0; i < ab->max_sample_count; i++) {
            samples[i*2+0] = DAC_ZERO;
            samples[i*2+1] = DAC_ZERO;
        }
        ab->sample_count = ab->max_sample_count;
        give_audio_buffer(producer_pool, ab);
    }
    audio_i2s_set_enabled(true);

    decode_flg = true;
    return producer_pool;
}

int main() {

    stdio_init_all();
    init_adc();

    // Set PLL_USB 96MHz
    pll_init(pll_usb, 1, 1536 * MHZ, 4, 4);
    clock_configure(clk_usb,
        0,
        CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
        96 * MHZ,
        48 * MHZ);
    // Change clk_sys to be 96MHz.
    clock_configure(clk_sys,
        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
        96 * MHZ,
        96 * MHZ);
    // CLK peri is clocked from clk_sys so need to change clk_peri's freq
    clock_configure(clk_peri,
        0,
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
        96 * MHZ,
        96 * MHZ);
    // Reinit uart now that clk_peri has changed
    stdio_init_all();

    // DCDC PSM control
    // 0: PFM mode (best efficiency)
    // 1: PWM mode (improved ripple)
    // gpio_init(PIN_DCDC_PSM_CTRL);
    // gpio_set_dir(PIN_DCDC_PSM_CTRL, GPIO_OUT);
    // gpio_put(PIN_DCDC_PSM_CTRL, 1); // PWM mode for less Audio noise

    // for (int i = 0; i < SINE_WAVE_TABLE_LEN; i++) {
    //     sine_wave_table[i] = 32767 * cosf(i * 2 * (float) (M_PI / SINE_WAVE_TABLE_LEN));
    // }

    ap = i2s_audio_init(41000); //TODO: Possible change to fix
	adc_run(true);
    while (true) {
    }
    return 0;
}
/*zeros*/
// void decode(){
//  audio_buffer_t *buffer = take_audio_buffer(ap, false);
//     if (buffer == NULL) return;

//     int16_t *samples = (int16_t *) buffer->buffer->bytes;
//     for (uint i = 0; i < buffer->max_sample_count; i++) {
//         samples[i*2 + 0] = 0;  // left
//         samples[i*2 + 1] = 0;  // right
//     }
//     buffer->sample_count = buffer->max_sample_count;
//     give_audio_buffer(ap, buffer);
// }
void decode(){
	   audio_buffer_t *buffer = take_audio_buffer(ap, false);
    if (buffer == NULL) return;

    int16_t *samples = (int16_t *) buffer->buffer->bytes;

    int buf_to_read = -1;
    if (buffer_full[0]) buf_to_read = 0;
    else if (buffer_full[1]) buf_to_read = 1;

    if (buf_to_read == -1) {
		//silent buffer
        for (uint i = 0; i < buffer->max_sample_count; i++) {
            samples[i*2 + 0] = 0;
            samples[i*2 + 1] = 0;
        }
    } else {

        for (uint i = 0; i < buffer->max_sample_count; i++) {
            int16_t val = ((int32_t)sample_buffer[buf_to_read][i] - 2048) << 4;
            samples[i*2 + 0] = val;
            samples[i*2 + 1] = val;
        }
        buffer_full[buf_to_read] = false;
    }

    buffer->sample_count = buffer->max_sample_count;
    give_audio_buffer(ap, buffer);
}


// void decode()
// {
// 	// Ping pong buffering; grab from the one that isn't being touched by producer
// 	int buffer_number = buffer;
// 	buffer = !buffer; // signal to switch buffers in the adc
// 	switch_buffers = true;

//     audio_buffer_t *buffer = take_audio_buffer(ap, false);
//     if (buffer == NULL) { return; }
//     int32_t *samples = (int32_t *) buffer->buffer->bytes;
//     for (uint i = 0; i < buffer->max_sample_count; i++) {

// 		// int32_t value0 = (vol * sine_wave_table[pos0 >> 16u]) << 8u;
//         // int32_t value1 = (vol * sine_wave_table[pos1 >> 16u]) << 8u;
//         // // use 32bit full scale
//         // samples[i*2+0] = value0 + (value0 >> 16u);  // L
//         // samples[i*2+1] = value1 + (value1 >> 16u);  // R
//         // pos0 += step0;
//         // pos1 += step1;
//         // if (pos0 >= pos_max) pos0 -= pos_max;
//         // if (pos1 >= pos_max) pos1 -= pos_max;

// 		// int32_t centered = ((int32_t)(sample_buffer[buffer_number][i])-2048) << 20;
// 		// int32_t centered = *(int32_t*)&sample_buffer[buffer_number][i];

//         // int32_t value0 = (centered); //<< 8u;
//         // int32_t value1 = (centered); //<< 8u;

//         // // use 32bit full scale
//         // samples[i*2+0] = centered; //+ (value0 >> 16u);  // L
//         // samples[i*2+1] = centered; //+ (value1 >> 16u);  // R
// 		int16_t raw_data = adc_read();
//         int32_t centered = ((int32_t)raw_data - 2048) << 20;
//         samples[i*2 + 0] = centered; // Left
//         samples[i*2 + 1] = centered; // Right
// 		// int32_t centered = ((int32_t)sample_buffer[buffer_number][i] - 2048) << 20;
//         // samples[i*2 + 0] = centered; // Left
//         // samples[i*2 + 1] = centered; // Right
//     }
//     buffer->sample_count = buffer->max_sample_count;
//     give_audio_buffer(ap, buffer);
//     return;
// }
void adc_callback(){

	uint16_t raw = adc_fifo_get();
    sample_buffer[buffer][index++] = raw;

    if (index >= ADC_SAMPLE_SIZE) {
        buffer_full[buffer] = true;   // mark current buffer as ready
        buffer = !buffer;             // switch to other buffer
        index = 0;
    }

    irq_clear(ADC_IRQ_FIFO);

	// uint16_t raw_data = adc_fifo_get();
	// adc_fifo_drain(); // resets event
	// static uint8_t index = 0;
	

	// if (switch_buffers){

	// 	sample_buffer[buffer][index] = raw_data;
	// 	index++;
	// }

	// if (index == ADC_SAMPLE_SIZE){
	// 	index = 0;
	// 	adc_flag = true;
	// 	switch_buffers = false;
	// 	// gpio_xor_mask(1 << INDICATOR); // toggle indicator
	// }

	// irq_clear(ADC_IRQ_FIFO);
}

