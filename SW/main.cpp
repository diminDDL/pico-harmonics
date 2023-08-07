#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/malloc.h"
#include <cstdlib>
#include <math.h>
#include "pico/mem_ops.h"
#include "hardware/pio.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "lib/pwm.hpp"
#include "pwm.pio.h"
#include "lib/fix_fft.h"

// Channel 0 is GPIO26
#define CAPTURE_CHANNEL 0
#define CAPTURE_DEPTH 1024
#define MAX_FFT_DATA_LEN CAPTURE_DEPTH  // Maximum data length. Change this according to your needs.
#define SAMPLE_RATE 500000.0

short capture_buf[CAPTURE_DEPTH];

void compute_fft_and_print_magnitudes(short data[], int data_len, double sampling_rate) {
    if (data_len > MAX_FFT_DATA_LEN) {
        printf("Data size exceeds maximum limit!");
        return;
    }

    int m = 0;
    while ((1 << m) < data_len) m++;  // Calculate the value of m for FFT, based on data length.
    short real[MAX_FFT_DATA_LEN];
    short imag[MAX_FFT_DATA_LEN];

    // Copy the data to the real array and set imaginary parts to 0.
    for (int i = 0; i < data_len; i++) {
        real[i] = data[i];
        imag[i] = 0;
    }

    // Perform the FFT.
    fix_fft(real, imag, m, 0);

    // Print the frequency bins and their magnitudes.
    for (int i = 0; i < data_len/2; i++) {  // Only up to Nyquist frequency.
        double frequency = i * sampling_rate / data_len;
        double magnitude = sqrt(real[i] * real[i] + imag[i] * imag[i]);
        //printf("Frequency %f Hz: Magnitude = %f\n", frequency, magnitude);
        printf("%.2f,%.2f\n", frequency, magnitude);
    }
}

int main() {
    //vreg_set_voltage(VREG_VOLTAGE_1_20);
    set_sys_clock_khz(100000, true);
    stdio_init_all();
    sleep_ms(3000);
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 1);

    PWM pwm(16, pwm_gpio_to_slice_num(16));
    pwm.init();
    pwm.set_freq(15000);
    pwm.set_duty_cycle(50);
    pwm.set_enabled(true);
    
    adc_gpio_init(26 + CAPTURE_CHANNEL);

    adc_init();
    adc_select_input(CAPTURE_CHANNEL);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit
        false    // Don't shift
    );

    adc_set_clkdiv(0);

    printf("Arming DMA\n");
    // Set up the DMA to start transferring data as soon as it appears in FIFO
    uint dma_chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);

    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&cfg, DREQ_ADC);

    dma_channel_configure(dma_chan, &cfg,
        capture_buf,    // dst
        &adc_hw->fifo,  // src
        CAPTURE_DEPTH,  // transfer count
        true            // start immediately
    );

    printf("Starting capture\n");
    adc_run(true);

    // Once DMA finishes, stop any new conversions from starting, and clean up
    // the FIFO in case the ADC was still mid-conversion.
    dma_channel_wait_for_finish_blocking(dma_chan);
    printf("Capture finished\n");
    adc_run(false);
    adc_fifo_drain();

    sleep_ms(100);

    // go through the array and mask the lower 12 bits because the ADC is 12 bits
    for (int i = 0; i < CAPTURE_DEPTH; ++i) {
        capture_buf[i] &= 0x0FFF;
    }

    // Print samples to stdout so you can display them in pyplot, excel, matlab
    for (int i = 0; i < CAPTURE_DEPTH; ++i) {
        printf("%d,", capture_buf[i]);
        //printf("%d\n", i);
    }

    printf("RUNNING FFT\n");

    int data_len = sizeof(capture_buf) / sizeof(capture_buf[0]);
    compute_fft_and_print_magnitudes(capture_buf, CAPTURE_DEPTH, SAMPLE_RATE);

    while (true) {
        tight_loop_contents();
    }

}