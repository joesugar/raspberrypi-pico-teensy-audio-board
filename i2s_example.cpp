/* 
 * This example is based upon Daniel Collins RP2040 I2S example code
 * and is distributed in the hope that it will be useful, but WITHOUT 
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public 
 * License for more details.
 *
 * This example is free software: you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License, version 3 as 
 * published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this. If not, see <https://www.gnu.org/licenses/>.
 *
 * Author: Joseph A Consugar
 * Date:   October 2024
 *
 * Copyright (c) 2024 Joseph A Consugar
 *
 */
#include <iostream>
#include <optional>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "i2s.h"

// I2C defines
//
// This code uses I2C1 on GPIO14 (SDA) and GPIO15 (SCL) running at 100KHz.
// 
#define I2C_PORT i2c1
#define I2C_SDA  14
#define I2C_SCL  15

#ifndef PICO_DEFAULT_LED_PIN
#warning blink example requires a board with a regular LED
#else
const uint LED_PIN = PICO_DEFAULT_LED_PIN;
#endif

static __attribute__((aligned(8))) pio_i2s i2s;


/**
 * @brief Callback for processing the incoming audio
 */
static void process_audio(const int32_t* input, int32_t* output, size_t num_frames) 
{
    // Just copy the input to the output
    //
    for (size_t i = 0; i < num_frames * 2; i++) {
        output[i] = input[i];
    }
}


/**
 * @brief Handler for when the DMA is ready
 */
static void dma_i2s_in_handler(void) 
{
    // We're double buffering using chained TCBs. By checking which buffer the
    // DMA is currently reading from, we can identify which buffer it has just
    // finished reading (the completion of which has triggered this interrupt).
    //
    if (*(int32_t**)dma_hw->ch[i2s.dma_ch_in_ctrl].read_addr == i2s.input_buffer) 
    {
        // It is inputting to the second buffer so we can overwrite the first
        //
        process_audio(i2s.input_buffer, i2s.output_buffer, AUDIO_BUFFER_FRAMES);
    } 
    else 
    {
        // It is currently inputting the first buffer, so we write to the second
        //
        process_audio(&i2s.input_buffer[STEREO_BUFFER_SIZE], &i2s.output_buffer[STEREO_BUFFER_SIZE], AUDIO_BUFFER_FRAMES);
    }
    dma_hw->ints0 = 1u << i2s.dma_ch_in_data;  // clear the IRQ
}


/**
 * @brief Scan the I2C bus to return the codec I2C address
 * @return Codec I2C address
 */
static std::optional<uint8_t> i2c_address()
{
    std::optional<uint8_t> i2c_addr {};

    for (uint8_t addr = 0; addr < (1 << 7); ++addr) 
    {
        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.
        //
        uint8_t rxdata;
        if (i2c_read_blocking(I2C_PORT, addr, &rxdata, 1, false) >= 0)
        {
            i2c_addr = std::make_optional(addr);
            break;
        }
    }

    return i2c_addr;
}


/**
 * @brief Main program
 */
int main() 
{
    // Set a 132.000 MHz system clock to more evenly divide the audio frequencies
    //
    set_sys_clock_khz(132000, true);
    stdio_init_all();

    std::cout << std::endl;
    std::cout << "Pi Pico Teensy Audio Adapter (SGTL5000) Example" << std::endl;
    std::cout << "System Clock: " << clock_get_hz(clk_sys) << std::endl;

    // Init GPIO LED
    //
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // I2S initialiation.
    //
    // You can set the I2S configuration by changing values in the
    // i2s_configf_default structure contained in i2s.c.
    //
    i2s_program_start_synched(pio0, &i2s_config_default, dma_i2s_in_handler, &i2s);

    // I2C Initialisation at 100Khz.
    //
    i2c_init(I2C_PORT, 100 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_set_pulls(I2C_SDA, true, false);
    gpio_set_pulls(I2C_SCL, true, false);
    gpio_set_drive_strength(I2C_SDA, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(I2C_SCL, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_slew_rate(I2C_SDA, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(I2C_SCL, GPIO_SLEW_RATE_FAST);

    // Get the codec I2C address.
    //
    auto i2c_addr = i2c_address();
    if (i2c_addr.has_value())
        std::cout << "Codec found on I2C address " << i2c_addr.value() << std::endl;
    else
        std::cout << "Codec not found on I2C bus" << std::endl;

    // Complete the code initialiation.
    //

    // Blink the LED so we know we started everything correctly.
    //
    uint32_t led_delay = (i2c_addr.has_value()) ? 250 : 2000;
    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(led_delay);
        gpio_put(LED_PIN, 0);
        sleep_ms(led_delay);
    }

    return 0;
}
