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

/*
  FIX_MPY() - fixed-point multiplication & scaling.
  Substitute inline assembly for hardware-specific
  optimization suited to a particluar DSP processor.
  Scaling ensures that result remains 16-bit.
*/
inline short __time_critical_func(FIX_MPY)(short a, short b)
{
	/* shift right one less bit (i.e. 15-1) */
    int c = ((int)a * (int)b);
    //printf("c - %d\n", c);
    interp0->accum[0] = c;
    interp0->accum[1] = c;

    //a = interp0->peek[0];
    //printf("lane 0 - %d\n", a);
    //b = interp0->peek[1];
    //printf("lane 1 - %d\n", b);

	return interp0->pop[2];
}

inline short __time_critical_func(FIX_MPY_ORIG)(short a, short b)
{
	/* shift right one less bit (i.e. 15-1) */
	int c = ((int)a * (int)b) >> 14;
    //printf("c - %d\n", c);
	/* last bit shifted out = rounding-bit */
	b = c & 0x01;
    //printf("b - %d\n", b);
	/* last shift + rounding bit */
	a = (c >> 1) + b;
    //printf("a - %d\n", a);
	return a;
}

int main()
{
    vreg_set_voltage(VREG_VOLTAGE_MAX);
    set_sys_clock_khz(200000, true);
    stdio_init_all();

    // configure the interpolators
    // int0 lane 0
    interp_config cfg = interp_default_config();
    interp_config_set_shift(&cfg, 15);
    interp_config_set_mask(&cfg, 0, 15);
    interp_config_set_signed(&cfg, true);
    interp_set_config(interp0, 0, &cfg);

    // int0 lane 1
    cfg = interp_default_config();
    interp_config_set_shift(&cfg, 14);
    interp_config_set_mask(&cfg, 0, 0);
    interp_set_config(interp0, 1, &cfg);

    // // int1 lane 0
    // cfg = interp_default_config();
    // interp_config_set_shift(&cfg, 14);
    // interp_set_config(interp1, 0, &cfg);

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
    adc_program_init(pio, adc_sm, adc_offset, 5.0, SCLK_PIN, CS_PIN, SDO_PIN);

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

        // drain the pio fifo

        // uint16_t data_array[128] = {0}; // Renamed the array to data_array
        // size_t i = 0;
        // while (i < 128)
        // {
        //     uint16_t data_value = pio_sm_get_blocking(pio, adc_sm); // Renamed to data_value
        //     data_array[i] = data_value; // Using the renamed array
        //     i += 1;
        // }
        // printf("%d\n%016b\n", capture_buf[20], capture_buf[20]);
        
        int data_len = sizeof(capture_buf) / sizeof(capture_buf[0]);
        //compute_fft_and_print_magnitudes(capture_buf, CAPTURE_DEPTH, SAMPLE_RATE);
        
        // measure performance of FIX_MPY
        uint32_t start = time_us_32();
        short a = 0x7FFF;
        short b = 0x7FFF - 1;
        for(short i = 0; i <= b; i++) {
            FIX_MPY(a, i);
        }
        uint32_t end = time_us_32();

        printf("FIX_MPY took %d us to do %d calculations\n", end - start, b);

        // for(short d = 0; d <= 10; d++) {
        //     printf("%d * %d = %d\n", 0x7FFF, d, FIX_MPY(0x7FFF, d));
        // }

        sleep_ms(500);

        // printf("=====\n");
        // // print all the data in the capture
        // for (int i = 0; i < CAPTURE_DEPTH; i++)
        // {
        //     printf("%d,\n", capture_buf[i]);
        // }
        // printf("=====\n");
        // sleep_ms(1000);
    }
}
