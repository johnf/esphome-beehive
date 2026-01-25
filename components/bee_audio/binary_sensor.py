"""
Bee Audio Binary Sensor Platform

Exposes queen piping detection as a binary sensor.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID
from . import bee_audio_ns, BeeAudioComponent

DEPENDENCIES = ["bee_audio"]

CONF_BEE_AUDIO_ID = "bee_audio_id"
CONF_QUEEN_PIPING = "queen_piping"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BEE_AUDIO_ID): cv.use_id(BeeAudioComponent),
        cv.Optional(CONF_QUEEN_PIPING): binary_sensor.binary_sensor_schema(),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_BEE_AUDIO_ID])

    if queen_piping := config.get(CONF_QUEEN_PIPING):
        sens = await binary_sensor.new_binary_sensor(queen_piping)
        cg.add(parent.set_queen_piping_sensor(sens))
