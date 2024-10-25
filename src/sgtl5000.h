#include "hardware/i2c.h"

class sgtl5000
{
    public:
        const uint16_t AUDIO_HEADPHONE_DAC = 0x0000;
        const uint16_t AUDIO_HEADPHONE_LINEIN = 0x0001;

        sgtl5000(i2c_inst_t* i2c, uint16_t addr);
        sgtl5000(i2c_inst_t* i2c);

        auto init(void) -> void;
        auto dac_volume(float left, float right) -> void;
        auto headphone_select(uint16_t input) -> void;
        auto mute_dac(bool mute) -> void;
        auto mute_headphone(bool mute) -> void;
        auto volume(float left, float right) -> void;

    private:
        const uint8_t CHIP_DIG_POWER = 0x0002;
        const uint8_t CHIP_CLK_CTRL = 0x0004;
        const uint8_t CHIP_I2S_CTRL = 0x0006;
        const uint8_t CHIP_SSS_CTRL = 0x000A;
        const uint8_t CHIP_ADCDAC_CTRL = 0x000E;
        const uint8_t CHIP_DAC_VOL = 0x0010;
        const uint8_t CHIP_ANA_HP_CTRL = 0x0022;
        const uint8_t CHIP_ANA_CTRL = 0x0024;
        const uint8_t CHIP_LINREG_CTRL = 0x0026;
        const uint8_t CHIP_REF_CTRL = 0x0028;
        const uint8_t CHIP_LINE_OUT_CTRL = 0x002C;
        const uint8_t CHIP_LINE_OUT_VOL = 0x002E;
        const uint8_t CHIP_ANA_POWER = 0x0030;
        const uint8_t CHIP_SHORT_CTRL = 0x003C;

        i2c_inst_t* i2c_;
        uint8_t  addr_;
        uint16_t adc_dac_ctrl_;
        uint16_t analog_ctrl_;

        auto write_reg(uint16_t addr, uint16_t value) -> void;
        auto read_reg(uint16_t addr) -> uint16_t;

        auto calc_volume(float volume, uint16_t range) -> uint16_t;
        auto calc_volume(float volume) -> uint16_t;
};
