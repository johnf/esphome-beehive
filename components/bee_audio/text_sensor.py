"""
Bee Audio Text Sensor Platform

Exposes hive state classification as a text sensor.
Possible states: quiet, normal, active, queenless, queen_activity, pre_swarm
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID
from . import bee_audio_ns, BeeAudioComponent

DEPENDENCIES = ["bee_audio"]

CONF_BEE_AUDIO_ID = "bee_audio_id"
CONF_HIVE_STATE = "hive_state"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BEE_AUDIO_ID): cv.use_id(BeeAudioComponent),
        cv.Optional(CONF_HIVE_STATE): text_sensor.text_sensor_schema(),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_BEE_AUDIO_ID])

    if hive_state := config.get(CONF_HIVE_STATE):
        sens = await text_sensor.new_text_sensor(hive_state)
        cg.add(parent.set_hive_state_sensor(sens))
