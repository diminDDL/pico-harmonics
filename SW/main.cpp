#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/malloc.h"
#include <cstdlib>
#include <math.h>
#include "pico/mem_ops.h"
#include "pico/divider.h"
#include "hardware/pio.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/interp.h"
#include "lib/pwm.hpp"
#include "adc.pio.h"
#include "lib/fix_fft.h"
#include "lib/windows.h"

// Channel 0 is GPIO26
#define CAPTURE_CHANNEL 0
#define CAPTURE_DEPTH 1024
#define MAX_FFT_DATA_LEN CAPTURE_DEPTH // Maximum data length. Change this according to your needs.
#define SAMPLE_RATE 350000.0

uint16_t capture_buf[CAPTURE_DEPTH];

void apply_window(int16_t *signal)
{
    for (int i = 0; i < CAPTURE_DEPTH; i++)
    {
        int32_t temp = (int32_t)signal[i] * window_hft90d_1024[i];
        signal[i] = (int16_t)(temp >> 15); // Convert back from Q15 to int16_t
        // printf("%d\n", signal[i]);
    }
}

void __time_critical_func(compute_fft_and_print_magnitudes)(uint16_t data[], int data_len, double sampling_rate)
{
    if (data_len > MAX_FFT_DATA_LEN)
    {
        printf("Data size exceeds maximum limit!");
        return;
    }

    int m = 0;
    while ((1 << m) < data_len)
        m++; // Calculate the value of m for FFT, based on data length.
    short real[MAX_FFT_DATA_LEN];
    short imag[MAX_FFT_DATA_LEN];

    // Copy the data to the real array and set imaginary parts to 0.
    for (int i = 0; i < data_len; i++)
    {
        real[i] = (int32_t)data[i] - 32768;
        imag[i] = 0;
    }

    // Apply window function.
    apply_window(real);

    // Perform the FFT.
    fix_fft(real, imag, m, 0);

    // Print the frequency bins and their magnitudes.
    printf("=====\n");
    for (int i = 0; i < data_len / 2; i++)
    { // Only up to Nyquist frequency.
        double frequency = i * sampling_rate / data_len;
        double magnitude = sqrt(real[i] * real[i] + imag[i] * imag[i]);
        // printf("Frequency %f Hz: Magnitude = %f\n", frequency, magnitude);
        printf("%.2f,%.2f\n", frequency, magnitude);
    }
    printf("===\n");
}

int main()
{
    vreg_set_voltage(VREG_VOLTAGE_MAX);
    set_sys_clock_khz(200000, true);
    stdio_init_all();

    sleep_ms(1000);

    PWM pwm(15, pwm_gpio_to_slice_num(15));
    pwm.init();
    pwm.set_freq(15000);
    pwm.set_duty_cycle(50);
    pwm.set_enabled(true);

    // configure AD4000

#define CS_PIN 9
#define SCLK_PIN 10
#define SDI_PIN 11
#define SDO_PIN 12

    PIO pio = pio0;
    uint adc_sm = pio_claim_unused_sm(pio, true);
    uint adc_offset = pio_add_program(pio, &adc_program);
    adc_program_init(pio, adc_sm, adc_offset, 5.0, SCLK_PIN, CS_PIN, SDO_PIN);

    pio_sm_set_enabled(pio, adc_sm, true);

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 1);

    const uint dma_rx = dma_claim_unused_channel(true);

    while (true)
    {

        // start the DMA
        pio_sm_drain_tx_fifo(pio, adc_sm);
        dma_channel_config c = dma_channel_get_default_config(dma_rx);
        channel_config_set_read_increment(&c, false);
        channel_config_set_write_increment(&c, true);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
        channel_config_set_dreq(&c, pio_get_dreq(pio, adc_sm, false));
        dma_channel_configure(dma_rx, &c,
                              capture_buf,       // Destinatinon pointer
                              &pio->rxf[adc_sm], // Source pointer
                              CAPTURE_DEPTH,     // Number of transfers
                              true               // Start immediately
        );

        // wait for the DMA to finish
        // dma_channel_wait_for_finish_blocking(dma_rx);
        while (dma_channel_is_busy(dma_rx))
            tight_loop_contents();

        int data_len = sizeof(capture_buf) / sizeof(capture_buf[0]);
        compute_fft_and_print_magnitudes(capture_buf, CAPTURE_DEPTH, SAMPLE_RATE);

    }
}
