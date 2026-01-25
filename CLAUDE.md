# ESP32 Beehive Monitor

ESPHome-based beehive monitoring system using ESP-IDF framework with deep sleep for battery-powered operation.

## Overview

This project creates a battery-powered beehive monitoring system that:
- Wakes every 5 minutes to collect sensor data
- Publishes measurements to Home Assistant via API
- Returns to deep sleep for power efficiency

## Components

### Hardware

| Component | Purpose | Interface |
|-----------|---------|-----------|
| ESP32 | Microcontroller | - |
| INMP441 | I2S MEMS microphone for audio FFT analysis | I2S (GPIO25, 26, 33) |
| NAU7802 | 24-bit ADC for 4x 50kg load cells | I2C (0x2A) |
| SHT40 | Temperature and humidity sensor | I2C (0x44) |

### Pin Assignments

| Function | GPIO |
|----------|------|
| I2C SDA | GPIO21 |
| I2C SCL | GPIO22 |
| I2S WS (LRCLK) | GPIO25 |
| I2S SCK (BCLK) | GPIO26 |
| I2S SD (DIN) | GPIO33 |

## Custom Component: bee_audio

Located in `components/bee_audio/`, this component performs FFT-based audio analysis to detect bee colony health indicators.

### Frequency Bands Monitored

| Band | Range | Purpose |
|------|-------|---------|
| Low Freq | 60-100 Hz | Low frequency content |
| Baseline | 100-200 Hz | Colony baseline hum |
| Worker | 180-260 Hz | Worker flight activity |
| Quacking | 200-350 Hz | Virgin queen quacking (in cell) |
| Tooting | 350-500 Hz | Emerged virgin queen tooting |
| Queenless Mid | 478-677 Hz | Queenless detection indicator |
| Queenless High | 876-1080 Hz | Queenless detection indicator |

### Derived Metrics

- **Dominant Frequency**: Peak frequency in 60-600 Hz range
- **Sound Level RMS**: Overall sound level in dB
- **Spectral Centroid**: Centre of mass of spectrum (indicates frequency shift)
- **Hive State**: Classification (quiet/normal/active/queen_activity/queenless/pre_swarm)
- **Queen Piping**: Binary detection of queen piping sounds

### Technical Details

- Sample rate: 8000 Hz
- FFT size: 2048 samples
- Frequency resolution: ~3.9 Hz per bin
- Audio capture duration: ~256ms
- Uses ESP-DSP library for optimised FFT

## Building

```bash
# Validate configuration
esphome config beehive-monitor.yaml

# Compile
esphome compile beehive-monitor.yaml

# Upload
esphome upload beehive-monitor.yaml

# View logs
esphome logs beehive-monitor.yaml
```

## Configuration

1. Copy `secrets.yaml.example` to `secrets.yaml`
2. Fill in your WiFi credentials and Home Assistant API key
3. Adjust load cell calibration in `beehive-monitor.yaml`

## Development

### File Structure

```
components/
  bee_audio/
    __init__.py       # Hub component configuration
    sensor.py         # Frequency band sensors
    binary_sensor.py  # Queen piping detection
    text_sensor.py    # Hive state classification
    bee_audio.h       # C++ header
    bee_audio.cpp     # ESP-IDF I2S + ESP-DSP FFT implementation
beehive-monitor.yaml  # Main ESPHome configuration
secrets.yaml          # WiFi and API credentials (not in git)
```

### Dependencies

- ESPHome with ESP-IDF framework
- ESP-DSP library (automatically added via platformio)
- NAU7802 component (built into ESPHome)
- SHT4x component (built into ESPHome)

## Calibration

### Load Cells

1. Record raw NAU7802 value with empty hive platform
2. Place known weight on platform and record raw value
3. Update `calibrate_linear` filter in YAML:
   ```yaml
   filters:
     - calibrate_linear:
         - <empty_raw> -> 0
         - <loaded_raw> -> <known_weight_kg>
   ```

### Audio Thresholds

Classification thresholds in `bee_audio.cpp` may need adjustment based on:
- Microphone placement in hive
- Hive size and colony population
- Local acoustic environment

## Power Consumption

- Deep sleep: ~10 uA
- Active (WiFi + sensors): ~150 mA for ~10 seconds
- Average current at 5 min intervals: ~5 mA
