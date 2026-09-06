"""
Bee Audio Component for ESPHome

Custom component for FFT-based bee colony audio analysis using INMP441 microphone.
Uses ESP-IDF I2S driver and ESP-DSP for FFT processing.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import CONF_ID
from esphome.components import esp32

CODEOWNERS = ["@johnf"]
DEPENDENCIES = ["esp32"]

CONF_I2S_LRCLK_PIN = "i2s_lrclk_pin"
CONF_I2S_BCLK_PIN = "i2s_bclk_pin"
CONF_I2S_DIN_PIN = "i2s_din_pin"
CONF_SAMPLE_RATE = "sample_rate"
CONF_FFT_SIZE = "fft_size"
CONF_FRAMES = "frames"
CONF_BANDS = "bands"
CONF_LOW = "low"
CONF_HIGH = "high"
CONF_QUEENLESS_THRESHOLD = "queenless_threshold"
CONF_QUEEN_PIPING_THRESHOLD = "queen_piping_threshold"
CONF_ACTIVE_THRESHOLD = "active_threshold"
CONF_NORMAL_THRESHOLD = "normal_threshold"
CONF_PRE_SWARM_CENTROID = "pre_swarm_centroid"

bee_audio_ns = cg.esphome_ns.namespace("bee_audio")
BeeAudioComponent = bee_audio_ns.class_("BeeAudioComponent", cg.PollingComponent)
Band = bee_audio_ns.enum("Band")

# Band config key -> C++ enum value. Order matches the Band enum.
BANDS = {
    "low_freq": Band.BAND_LOW_FREQ,
    "baseline": Band.BAND_BASELINE,
    "worker": Band.BAND_WORKER,
    "quacking": Band.BAND_QUACKING,
    "tooting": Band.BAND_TOOTING,
    "queenless_mid": Band.BAND_QUEENLESS_MID,
    "queenless_high": Band.BAND_QUEENLESS_HIGH,
}

VALID_FFT_SIZES = [256, 512, 1024, 2048, 4096]


def _validate_band(value):
    if value[CONF_LOW] >= value[CONF_HIGH]:
        raise cv.Invalid("Band 'low' frequency must be below 'high' frequency")
    return value


BAND_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Required(CONF_LOW): cv.frequency,
            cv.Required(CONF_HIGH): cv.frequency,
        }
    ),
    _validate_band,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BeeAudioComponent),
        cv.Required(CONF_I2S_LRCLK_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_I2S_BCLK_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_I2S_DIN_PIN): pins.internal_gpio_input_pin_number,
        cv.Optional(CONF_SAMPLE_RATE, default=8000): cv.int_range(min=4000, max=48000),
        cv.Optional(CONF_FFT_SIZE, default=2048): cv.one_of(*VALID_FFT_SIZES, int=True),
        cv.Optional(CONF_FRAMES, default=4): cv.int_range(min=1, max=32),
        cv.Optional(CONF_BANDS, default={}): cv.Schema(
            {cv.Optional(key): BAND_SCHEMA for key in BANDS}
        ),
        cv.Optional(CONF_QUEENLESS_THRESHOLD, default="6dB"): cv.decibel,
        cv.Optional(CONF_QUEEN_PIPING_THRESHOLD, default="10dB"): cv.decibel,
        cv.Optional(CONF_ACTIVE_THRESHOLD, default="-95dB"): cv.decibel,
        cv.Optional(CONF_NORMAL_THRESHOLD, default="-105dB"): cv.decibel,
        cv.Optional(CONF_PRE_SWARM_CENTROID, default="400Hz"): cv.frequency,
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
    cg.add(var.set_frames(config[CONF_FRAMES]))

    for key, band in BANDS.items():
        if band_conf := config[CONF_BANDS].get(key):
            cg.add(var.set_band(band, band_conf[CONF_LOW], band_conf[CONF_HIGH]))

    cg.add(var.set_queenless_threshold(config[CONF_QUEENLESS_THRESHOLD]))
    cg.add(var.set_queen_piping_threshold(config[CONF_QUEEN_PIPING_THRESHOLD]))
    cg.add(var.set_active_threshold(config[CONF_ACTIVE_THRESHOLD]))
    cg.add(var.set_normal_threshold(config[CONF_NORMAL_THRESHOLD]))
    cg.add(var.set_pre_swarm_centroid(config[CONF_PRE_SWARM_CENTROID]))

    esp32.add_idf_component(
        name="esp-dsp",
        repo="https://github.com/espressif/esp-dsp.git",
        ref="v1.4.0",
    )
