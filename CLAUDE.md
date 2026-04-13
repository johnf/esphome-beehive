# ESP32 Beehive Monitor

ESPHome-based beehive monitoring system. See [README.md](README.md) for hardware,
wiring, installation, and usage documentation.

## Custom Component: bee_audio

Located in `components/bee_audio/`, this component performs FFT-based audio analysis
to detect bee colony health indicators.

### Technical Details

- Sample rate: 8000 Hz
- FFT size: 2048 samples
- Frequency resolution: ~3.9 Hz per bin
- Audio capture duration: ~256ms
- Uses ESP-DSP library for optimised FFT

### Configurable Parameters

| Parameter | Range | Default |
|-----------|-------|---------|
| sample_rate | 4000-48000 Hz | 8000 Hz |
| fft_size | 256, 512, 1024, 2048, 4096 | 2048 |

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
beehive-monitor.yaml  # Main ESPHome package configuration
fritzing/             # Fritzing schematic
```

### Dependencies

- ESPHome with ESP-IDF framework
- ESP-DSP library (automatically added via platformio)
- NAU7802 component (built into ESPHome)
- SHT4x component (built into ESPHome)
- INA226 component (built into ESPHome)
