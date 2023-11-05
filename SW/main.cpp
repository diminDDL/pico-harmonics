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
#include "lib/pwm.hpp"
#include "adc.pio.h"
#include "lib/fix_fft.h"
#include "lib/windows.h"

// Channel 0 is GPIO26
#define CAPTURE_CHANNEL 0
#define CAPTURE_DEPTH 1024
#define MAX_FFT_DATA_LEN CAPTURE_DEPTH // Maximum data length. Change this according to your needs.
#define SAMPLE_RATE 500000.0

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

    // adc_gpio_init(26 + CAPTURE_CHANNEL);

    // adc_init();
    // adc_select_input(CAPTURE_CHANNEL);
    // adc_fifo_setup(
    //     true,    // Write each completed conversion to the sample FIFO
    //     true,    // Enable DMA data request (DREQ)
    //     1,       // DREQ (and IRQ) asserted when at least 1 sample present
    //     false,   // We won't see the ERR bit
    //     false    // Don't shift
    // );

    // adc_set_clkdiv(0);

    // printf("Arming DMA\n");
    //  Set up the DMA to start transferring data as soon as it appears in FIFO
    //  uint dma_chan = dma_claim_unused_channel(true);
    //  dma_channel_config cfg = dma_channel_get_default_config(dma_chan);

    // // Reading from constant address, writing to incrementing byte addresses
    // channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    // channel_config_set_read_increment(&cfg, false);
    // channel_config_set_write_increment(&cfg, true);

    // // Pace transfers based on availability of ADC samples
    // channel_config_set_dreq(&cfg, DREQ_ADC);

    // while (true) {
    //     dma_channel_configure(dma_chan, &cfg,
    //        capture_buf,    // dst
    //        &adc_hw->fifo,  // src
    //        CAPTURE_DEPTH,  // transfer count
    //        true            // start immediately
    //    );

    //    //printf("Starting capture\n");
    //    adc_run(true);

    //    // Once DMA finishes, stop any new conversions from starting, and clean up
    //    // the FIFO in case the ADC was still mid-conversion.
    //    dma_channel_wait_for_finish_blocking(dma_chan);
    //    //printf("Capture finished\n");
    //    adc_run(false);
    //    adc_fifo_drain();

    //    sleep_ms(100);

    //    // go through the array and mask the lower 12 bits because the ADC is 12 bits
    //    for (int i = 0; i < CAPTURE_DEPTH; ++i) {
    //        capture_buf[i] &= 0x0FFF;
    //    }

    //    //printf("RUNNING FFT\n");

    //    int data_len = sizeof(capture_buf) / sizeof(capture_buf[0]);
    //    compute_fft_and_print_magnitudes(capture_buf, CAPTURE_DEPTH, SAMPLE_RATE);

    //    //sleep_ms(1000);
    //     //return 0;
    // }

    // configure AD4000

#define CS_PIN 9
#define SCLK_PIN 10
#define SDI_PIN 11
#define SDO_PIN 12

    PIO pio = pio0;
    uint adc_sm = pio_claim_unused_sm(pio, true);
    uint adc_offset = pio_add_program(pio, &adc_program);
    adc_program_init(pio, adc_sm, adc_offset, 100.0, SCLK_PIN, CS_PIN, SDO_PIN);

    pio_sm_set_enabled(pio, adc_sm, true);

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 1);

    // spi_init(spi1, 10e6); // set the clock to 1 MHz, the chip can handle up to 80 MHz
    // gpio_set_function(SDO_PIN, GPIO_FUNC_SPI);
    // gpio_set_function(SCLK_PIN, GPIO_FUNC_SPI);
    // gpio_set_function(SDI_PIN, GPIO_FUNC_SPI);
    // gpio_set_function(CS_PIN, GPIO_FUNC_SPI);

    // configure the SPI instance, sampling on falling edge, MSB first
    // spi_set_format(spi1, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    // // CS is actually conv and is active high
    // gpio_init(CS_PIN);
    // gpio_set_dir(CS_PIN, GPIO_OUT);
    // gpio_put(CS_PIN, 0);

    uint16_t repeat = 0x0001;
    uint16_t result = 0x0000;

    const uint dma_tx = dma_claim_unused_channel(true);
    const uint dma_rx = dma_claim_unused_channel(true);

    uint16_t zero = 0x0000;

    while (true)
    {
        // //pull the conv pin high
        // gpio_put(CS_PIN, 1);
        // //wait for a microsecond
        // sleep_us(1);
        // //pull the conv pin low
        // gpio_put(CS_PIN, 0);
        // //wait for a microsecond
        // sleep_us(1);
        // //read 16 bits from the spi bus
        // uint16_t len = spi_read16_blocking(spi1, zero, &result, 1);
        // //print the result
        // printf("%d - %16b\n", result, result);

        // printf("Starting capture\n");

        // dma_channel_config c = dma_channel_get_default_config(dma_tx);
        // channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
        // channel_config_set_dreq(&c, spi_get_dreq(spi1, true));
        // channel_config_set_read_increment(&c, false);
        // channel_config_set_write_increment(&c, false);
        // dma_channel_configure(dma_tx, &c,
        //                       &spi_get_hw(spi1)->dr, // write address
        //                       &repeat,                        // read address
        //                       CAPTURE_DEPTH,                    // element count (each element is of size transfer_data_size)
        //                       false);                       // don't start yet

        // c = dma_channel_get_default_config(dma_rx);
        // channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
        // channel_config_set_dreq(&c, spi_get_dreq(spi1, false));
        // channel_config_set_read_increment(&c, false);
        // channel_config_set_write_increment(&c, true);
        // dma_channel_configure(dma_rx, &c,
        //                       capture_buf,           // write address
        //                       &spi_get_hw(spi1)->dr, // read address
        //                       CAPTURE_DEPTH,         // element count (each element is of size transfer_data_size)
        //                       false);                 // don't start yet

        // gpio_set_function(SDO_PIN, GPIO_FUNC_SPI);

        // dma_start_channel_mask((1u << dma_tx) | (1u << dma_rx));

        // // wait until dma is done
        // dma_channel_wait_for_finish_blocking(dma_rx);

        // gpio_set_function(SDO_PIN, GPIO_FUNC_SIO);
        // gpio_init(SDO_PIN);
        // gpio_set_dir(SDO_PIN, GPIO_OUT);
        // gpio_put(SDO_PIN, 1);

        // // print 100 results to the console
        // for (int i = 0; i < 100; i++)
        // {
        //     printf("%d - %16b\n", capture_buf[i], capture_buf[i]);
        // }

        // printf("Capture finished\n");

        // // wait for a while
        // sleep_ms(1000);

        // drain the pio fifo
        pio_sm_drain_tx_fifo(pio, adc_sm);
        uint16_t data_array[128] = {0}; // Renamed the array to data_array
        size_t i = 0;
        while (i < 128)
        {
            uint16_t data_value = pio_sm_get_blocking(pio, adc_sm); // Renamed to data_value
            data_array[i] = data_value; // Using the renamed array
            i += 1;
        }
        printf("%d\n%016b\n", data_array[20], data_array[20]);
    }
}