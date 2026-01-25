"""
Bee Audio Component for ESPHome

Custom component for FFT-based bee colony audio analysis using INMP441 microphone.
Uses ESP-IDF I2S driver and ESP-DSP for FFT processing.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import (
    CONF_ID,
)
from esphome.components import esp32
from esphome.core import CORE

CODEOWNERS = ["@johnf"]
DEPENDENCIES = ["esp32"]

CONF_I2S_LRCLK_PIN = "i2s_lrclk_pin"
CONF_I2S_BCLK_PIN = "i2s_bclk_pin"
CONF_I2S_DIN_PIN = "i2s_din_pin"
CONF_SAMPLE_RATE = "sample_rate"
CONF_FFT_SIZE = "fft_size"

bee_audio_ns = cg.esphome_ns.namespace("bee_audio")
BeeAudioComponent = bee_audio_ns.class_("BeeAudioComponent", cg.PollingComponent)

# Valid FFT sizes (must be power of 2)
VALID_FFT_SIZES = [256, 512, 1024, 2048, 4096]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BeeAudioComponent),
        cv.Required(CONF_I2S_LRCLK_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_I2S_BCLK_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_I2S_DIN_PIN): pins.internal_gpio_input_pin_number,
        cv.Optional(CONF_SAMPLE_RATE, default=8000): cv.int_range(min=4000, max=48000),
        cv.Optional(CONF_FFT_SIZE, default=2048): cv.one_of(*VALID_FFT_SIZES, int=True),
    }
).extend(cv.polling_component_schema("never"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_i2s_lrclk_pin(config[CONF_I2S_LRCLK_PIN]))
    cg.add(var.set_i2s_bclk_pin(config[CONF_I2S_BCLK_PIN]))
    cg.add(var.set_i2s_din_pin(config[CONF_I2S_DIN_PIN]))
    cg.add(var.set_sample_rate(config[CONF_SAMPLE_RATE]))
    cg.add(var.set_fft_size(config[CONF_FFT_SIZE]))

    # ESP-DSP dependency is declared in idf_component.yml
