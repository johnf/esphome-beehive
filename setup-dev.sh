#!/bin/bash

if [ ! -d "cdeps/esphome-repo" ]; then
  git clone https://github.com/esphome/esphome.git cdeps/esphome-repo
fi

if [ ! -d "cdeps/esp-dsp" ]; then
  git clone git@github.com:espressif/esp-dsp.git cdeps/esp-dsp
fi

if [ ! -d "cdeps/esp-idf" ]; then
  git clone git@github.com:espressif/esp-idf.git cdeps/esp-idf
fi

if [ ! -d "cdeps/ArduinoJson" ]; then
  git clone git@github.com:bblanchon/ArduinoJson.git cdeps/ArduinoJson
fi
