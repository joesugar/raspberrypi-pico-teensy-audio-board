#include "sgtl5000.h"
#include <iostream>

/**
 * Code programming document can be found at 
 * https://www.nxp.com/docs/en/application-note/AN3663.pdf
 * 
 * Data sheet can be found at
 * https://www.pjrc.com/teensy/SGTL5000.pdf
 */
/**
 * Constructor
 */
sgtl5000::sgtl5000(i2c_inst_t* i2c, uint16_t addr)
    : i2c_(i2c)
    , addr_(addr)
    , adc_dac_ctrl_(0x0000)     // Enable & mute headphone output, select & unmute line in & enable ZCD
    , analog_ctrl_(0x0036)      // Unmute DAC, ADC normal operations, disable volume ramp
{

}

sgtl5000::sgtl5000(i2c_inst_t* i2c)
    : i2c_(i2c)
    , addr_(0x000A)             // Default I2C address
    , adc_dac_ctrl_(0x0000)
    , analog_ctrl_(0x0036)
{

}

/**
 * Initialize the codec.
 */
auto sgtl5000::init(void) -> void
{
    // Configuration commands taken from the micropython SGTL5000 driver
    // that can be found at https://github.com/rdagger/micropython-sgtl5000/blob/main/sgtl5000.py
    //
    // VDDD externally driven with 1.8V
    write_reg(CHIP_ANA_POWER, 0x4060);

    // VDDA & VDDIO both over 3.1V
    write_reg(CHIP_LINREG_CTRL, 0x006C);

    // VAG=1.575, normal ramp, +12.5% bias current
    write_reg(CHIP_REF_CTRL, 0x01F2);

    // LO_VAGCNTRL=1.65V, OUT_CURRENT=0.54mA
    write_reg(CHIP_LINE_OUT_CTRL, 0x0F22);

    // Allow up to 125mA
    write_reg(CHIP_SHORT_CTRL, 0x4446);

    // SGTL is I2S Slave (power up: lineout, hp, adc, dac)
    write_reg(CHIP_ANA_POWER, 0x40FF);

    // Power up all digital stuff
    write_reg(CHIP_DIG_POWER, 0x0073);
    sleep_ms(400);

    // Default approx 1.3 volts peak-to-peak
    write_reg(CHIP_LINE_OUT_VOL, 0x1D1D);

    // Set the Sysclk parameters
    // Assuming a 48000 Hz sample rate.
    uint16_t mclk_mode = 0;
    uint16_t sys_fs = 2;
    uint16_t rate_mode = 0;
    write_reg(CHIP_CLK_CTRL, (rate_mode << 4) | (sys_fs << 2) | mclk_mode);

    // Fsclk=Fs*64, 32bit samples, I2S format (data length)
    write_reg(CHIP_I2S_CTRL, 0x0030);

    // ADC->I2S, I2S->DAC
    write_reg(CHIP_SSS_CTRL, 0x0010);

    // Unmute DAC, ADC normal operations, disable volume ramp
    write_reg(CHIP_ADCDAC_CTRL, adc_dac_ctrl_);

    // Digital gain, 0dB
    write_reg(CHIP_DAC_VOL, 0x3C3C);

    // Set volume (lowest level)
    write_reg(CHIP_ANA_HP_CTRL, 0x7F7F);

    // Enable & mute headphone output, select & unmute line in & enable ZCD
    write_reg(CHIP_ANA_CTRL, analog_ctrl_);
}

/**
 * Mute or unmute the left and right DAC channels.
 * 
 * If mute == true then mute the DAC channels.
 */
auto sgtl5000::mute_dac(bool mute) -> void
{
    if (mute)
        adc_dac_ctrl_ |=  (3 << 2);
    else
        adc_dac_ctrl_ &= ~(3 << 2);
    write_reg(CHIP_ADCDAC_CTRL, adc_dac_ctrl_);
}

/**
 * Set the DAC right and left channel volume.
 */
auto sgtl5000::dac_volume(float left, float right) -> void
{
    // Make sure the volume is within thew proper range.
    // Range is from 0x3c (0 dB) to 0xFC (muted)
    if ((left < 0.0) || (right < 0.0))
        return;
    if ((left > 1.0) || (right > 1.0))
        return;

    uint16_t right_channel = 0x00FC - calc_volume(right, 0x00C0);
    uint16_t left_channel  = 0x00FC - calc_volume(left,  0x00C0);
    uint16_t volume = (right_channel << 8) | (left_channel << 0);
    write_reg(CHIP_DAC_VOL, volume);
}

/**
 * Select the headphone input (DAC or LINEIN)
 */
auto sgtl5000::headphone_select(uint16_t input) -> void
{
    // Make sure the input is within the proper range.
    //
    if ((input != AUDIO_HEADPHONE_DAC) && (input != AUDIO_HEADPHONE_LINEIN))
        return;

    if (input == AUDIO_HEADPHONE_DAC)
        analog_ctrl_ &= ~(1 << 6);
    else if (input == AUDIO_HEADPHONE_LINEIN)
        analog_ctrl_ |= (1 << 6);
    write_reg(CHIP_ANA_CTRL, analog_ctrl_);
}

/**
 * Mute or unmute the headphone outputs.
 */
auto sgtl5000::mute_headphone(bool mute) -> void
{
    if (mute)
        analog_ctrl_ |= (1 << 4);
    else
        analog_ctrl_ &= ~(1 << 4);
    write_reg(CHIP_ANA_CTRL, analog_ctrl_);
}

/**
 * Set headphone left and right channel volume.
 */
auto sgtl5000::volume(float left, float right) -> void
{
    // Make sure the volume is in the proper range.
    //
    if ((left < 0.0) || (right < 0.0))
        return;
    if ((left > 1.0) || (right > 1.0))
        return;

    uint16_t left_chan  = 0x007F - calc_volume(left);
    uint16_t right_chan = 0x007F - calc_volume(right);
    uint16_t volume = (right_chan << 8) | (left_chan << 0);
    write_reg(CHIP_ANA_HP_CTRL, volume);
}


/**
 * Write a value to a codec register via I2C
 */
auto sgtl5000::write_reg(uint16_t reg_addr, uint16_t value) -> void
{
    uint8_t src[4] = { 
        static_cast<uint8_t>((reg_addr >> 8) & 0x00FF),
        static_cast<uint8_t>((reg_addr >> 0) & 0x00FF),
        static_cast<uint8_t>((value >> 8) & 0x00FF), 
        static_cast<uint8_t>((value >> 0) & 0x00FF) 
    };

    i2c_write_blocking(i2c_, addr_, src, 4, false);
}

/**
 * Read a value from a codec register via I2C
 */
auto sgtl5000::read_reg(uint16_t reg_addr) -> uint16_t
{
    uint8_t src[2] = { 
        static_cast<uint8_t>((reg_addr >> 8) & 0x00FF), 
        static_cast<uint8_t>((reg_addr >> 0) & 0x00FF) 
    };
    uint8_t dst[2];

    i2c_write_blocking(i2c_, addr_, src, 2, true);
    i2c_read_blocking(i2c_, addr_, dst, 2, false);

    uint16_t high = static_cast<uint16_t>(dst[0]);
    uint16_t low  = static_cast<uint16_t>(dst[1]);
    return (high << 8) | (low << 0);
}

/**
 * Scale the volume from 0 : 1 to 0 : range
 */
auto sgtl5000::calc_volume(float volume, uint16_t range) -> uint16_t
{
    uint16_t cvol = static_cast<uint16_t>((volume * float(range)) + .499);
    return (cvol > range) ? range : cvol;
}

auto sgtl5000::calc_volume(float volume) -> uint16_t
{
    return sgtl5000::calc_volume(volume, 0x007F);
}
