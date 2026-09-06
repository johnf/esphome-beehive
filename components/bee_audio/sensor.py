"""
Bee Audio Sensor Platform

Exposes frequency band power sensors, dominant frequency, sound level, spectral
centroid, and modulation spectrum metrics.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    DEVICE_CLASS_SIGNAL_STRENGTH,
    DEVICE_CLASS_FREQUENCY,
    STATE_CLASS_MEASUREMENT,
    UNIT_DECIBEL,
    UNIT_HERTZ,
    UNIT_PERCENT,
)
from . import BANDS, BeeAudioComponent

DEPENDENCIES = ["bee_audio"]

CONF_BEE_AUDIO_ID = "bee_audio_id"
CONF_DOMINANT_FREQUENCY = "dominant_frequency"
CONF_SOUND_LEVEL_RMS = "sound_level_rms"
CONF_SPECTRAL_CENTROID = "spectral_centroid"
CONF_MODULATION_INDEX = "modulation_index"
CONF_MODULATION_FREQUENCY = "modulation_frequency"

POWER_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_DECIBEL,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_SIGNAL_STRENGTH,
    state_class=STATE_CLASS_MEASUREMENT,
)

FREQUENCY_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_HERTZ,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_FREQUENCY,
    state_class=STATE_CLASS_MEASUREMENT,
)

MODULATION_INDEX_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_PERCENT,
    icon="mdi:sine-wave",
    accuracy_decimals=1,
    state_class=STATE_CLASS_MEASUREMENT,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BEE_AUDIO_ID): cv.use_id(BeeAudioComponent),
        **{cv.Optional(f"band_{key}"): POWER_SENSOR_SCHEMA for key in BANDS},
        cv.Optional(CONF_DOMINANT_FREQUENCY): FREQUENCY_SENSOR_SCHEMA,
        cv.Optional(CONF_SOUND_LEVEL_RMS): POWER_SENSOR_SCHEMA,
        cv.Optional(CONF_SPECTRAL_CENTROID): FREQUENCY_SENSOR_SCHEMA,
        cv.Optional(CONF_MODULATION_INDEX): MODULATION_INDEX_SCHEMA,
        cv.Optional(CONF_MODULATION_FREQUENCY): FREQUENCY_SENSOR_SCHEMA,
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_BEE_AUDIO_ID])

    for key, band in BANDS.items():
        if band_conf := config.get(f"band_{key}"):
            sens = await sensor.new_sensor(band_conf)
            cg.add(parent.set_band_sensor(band, sens))

    if dominant_frequency := config.get(CONF_DOMINANT_FREQUENCY):
        sens = await sensor.new_sensor(dominant_frequency)
        cg.add(parent.set_dominant_frequency_sensor(sens))

    if sound_level_rms := config.get(CONF_SOUND_LEVEL_RMS):
        sens = await sensor.new_sensor(sound_level_rms)
        cg.add(parent.set_sound_level_rms_sensor(sens))

    if spectral_centroid := config.get(CONF_SPECTRAL_CENTROID):
        sens = await sensor.new_sensor(spectral_centroid)
        cg.add(parent.set_spectral_centroid_sensor(sens))

    if modulation_index := config.get(CONF_MODULATION_INDEX):
        sens = await sensor.new_sensor(modulation_index)
        cg.add(parent.set_modulation_index_sensor(sens))

    if modulation_frequency := config.get(CONF_MODULATION_FREQUENCY):
        sens = await sensor.new_sensor(modulation_frequency)
        cg.add(parent.set_modulation_frequency_sensor(sens))
