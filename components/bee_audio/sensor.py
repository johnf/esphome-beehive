"""
Bee Audio Sensor Platform

Exposes frequency band power sensors, dominant frequency, sound level, and spectral centroid.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_SIGNAL_STRENGTH,
    DEVICE_CLASS_FREQUENCY,
    STATE_CLASS_MEASUREMENT,
    UNIT_DECIBEL,
    UNIT_HERTZ,
)
from . import bee_audio_ns, BeeAudioComponent

DEPENDENCIES = ["bee_audio"]

CONF_BEE_AUDIO_ID = "bee_audio_id"

# Frequency band sensors
CONF_BAND_LOW_FREQ = "band_low_freq"
CONF_BAND_BASELINE = "band_baseline"
CONF_BAND_WORKER = "band_worker"
CONF_BAND_QUACKING = "band_quacking"
CONF_BAND_TOOTING = "band_tooting"
CONF_BAND_QUEENLESS_MID = "band_queenless_mid"
CONF_BAND_QUEENLESS_HIGH = "band_queenless_high"

# Derived metrics
CONF_DOMINANT_FREQUENCY = "dominant_frequency"
CONF_SOUND_LEVEL_RMS = "sound_level_rms"
CONF_SPECTRAL_CENTROID = "spectral_centroid"

# Sensor schema for dB power bands
POWER_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_DECIBEL,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_SIGNAL_STRENGTH,
    state_class=STATE_CLASS_MEASUREMENT,
)

# Sensor schema for frequency measurements
FREQUENCY_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_HERTZ,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_FREQUENCY,
    state_class=STATE_CLASS_MEASUREMENT,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BEE_AUDIO_ID): cv.use_id(BeeAudioComponent),
        # Frequency band sensors
        cv.Optional(CONF_BAND_LOW_FREQ): POWER_SENSOR_SCHEMA,
        cv.Optional(CONF_BAND_BASELINE): POWER_SENSOR_SCHEMA,
        cv.Optional(CONF_BAND_WORKER): POWER_SENSOR_SCHEMA,
        cv.Optional(CONF_BAND_QUACKING): POWER_SENSOR_SCHEMA,
        cv.Optional(CONF_BAND_TOOTING): POWER_SENSOR_SCHEMA,
        cv.Optional(CONF_BAND_QUEENLESS_MID): POWER_SENSOR_SCHEMA,
        cv.Optional(CONF_BAND_QUEENLESS_HIGH): POWER_SENSOR_SCHEMA,
        # Derived metrics
        cv.Optional(CONF_DOMINANT_FREQUENCY): FREQUENCY_SENSOR_SCHEMA,
        cv.Optional(CONF_SOUND_LEVEL_RMS): POWER_SENSOR_SCHEMA,
        cv.Optional(CONF_SPECTRAL_CENTROID): FREQUENCY_SENSOR_SCHEMA,
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_BEE_AUDIO_ID])

    # Frequency band sensors
    if band_low_freq := config.get(CONF_BAND_LOW_FREQ):
        sens = await sensor.new_sensor(band_low_freq)
        cg.add(parent.set_band_low_freq_sensor(sens))

    if band_baseline := config.get(CONF_BAND_BASELINE):
        sens = await sensor.new_sensor(band_baseline)
        cg.add(parent.set_band_baseline_sensor(sens))

    if band_worker := config.get(CONF_BAND_WORKER):
        sens = await sensor.new_sensor(band_worker)
        cg.add(parent.set_band_worker_sensor(sens))

    if band_quacking := config.get(CONF_BAND_QUACKING):
        sens = await sensor.new_sensor(band_quacking)
        cg.add(parent.set_band_quacking_sensor(sens))

    if band_tooting := config.get(CONF_BAND_TOOTING):
        sens = await sensor.new_sensor(band_tooting)
        cg.add(parent.set_band_tooting_sensor(sens))

    if band_queenless_mid := config.get(CONF_BAND_QUEENLESS_MID):
        sens = await sensor.new_sensor(band_queenless_mid)
        cg.add(parent.set_band_queenless_mid_sensor(sens))

    if band_queenless_high := config.get(CONF_BAND_QUEENLESS_HIGH):
        sens = await sensor.new_sensor(band_queenless_high)
        cg.add(parent.set_band_queenless_high_sensor(sens))

    # Derived metrics
    if dominant_frequency := config.get(CONF_DOMINANT_FREQUENCY):
        sens = await sensor.new_sensor(dominant_frequency)
        cg.add(parent.set_dominant_frequency_sensor(sens))

    if sound_level_rms := config.get(CONF_SOUND_LEVEL_RMS):
        sens = await sensor.new_sensor(sound_level_rms)
        cg.add(parent.set_sound_level_rms_sensor(sens))

    if spectral_centroid := config.get(CONF_SPECTRAL_CENTROID):
        sens = await sensor.new_sensor(spectral_centroid)
        cg.add(parent.set_spectral_centroid_sensor(sens))
