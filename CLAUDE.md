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
- Frames averaged per reading: 4 (~1s of audio, Welch averaging)
- Modulation capture: separate 10s capture, feeds the task watchdog while blocking
- Band levels: mean one-sided PSD in dB re FS²/Hz, independent of fft_size
- Uses ESP-DSP library for optimised FFT

### Configurable Parameters

| Parameter | Range | Default |
|-----------|-------|---------|
| sample_rate | 4000-48000 Hz | 8000 Hz |
| fft_size | 256, 512, 1024, 2048, 4096 | 2048 |
| frames | 1-32 | 4 |
| bands.<name>.low/high | frequency | research defaults in bee_audio.h |
| queenless_threshold | dB | 6dB |
| queen_piping_threshold | dB | 10dB |
| active_threshold | dB | -95dB |
| normal_threshold | dB | -105dB |
| pre_swarm_centroid | frequency | 400Hz |
| modulation_duration | 4-60s | 10s |
| modulation_band.low/high | frequency | 150-250Hz |
| modulation_rate.low/high | frequency | 10-25Hz |

Band names: low_freq, baseline, worker, quacking, tooting, queenless_mid,
queenless_high. Sensors are `band_<name>` on the sensor platform.

Modulation spectrum (Abdollahi et al. 2026, arXiv 2607.20386): a 100 ms /
12.5 ms hop STFT of `modulation_band` gives an ~80 Hz envelope; a Welch FFT
over 256-hop segments gives modulation power 0-40 Hz. `modulation_index` is the
percentage of 1-40 Hz envelope power inside `modulation_rate`;
`modulation_frequency` is the peak rate. The capture only runs when one of
those sensors is configured.

## Building

```bash
# example.yaml wraps the package and builds bee_audio from ./components.
# It needs a secrets.yaml with wifi_ssid and wifi_password (dummy values are fine).
esphome config example.yaml
esphome compile example.yaml
esphome upload example.yaml
esphome logs example.yaml
```

CI (`.github/workflows/ci.yml`) runs `esphome config` and `esphome compile` on
`example.yaml`. There are no releases: device configurations track the default
branch and use `refresh: always` so nothing is cached.

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
example.yaml          # Device config used for local builds and CI
dashboard.yaml        # Home Assistant dashboard (paste into raw config editor)
case/                 # OpenSCAD enclosures and exported STLs
fritzing/             # Fritzing schematic
```

### Enclosure

`case/beehive-case.scad` is printed on a Bambu Lab A1 mini (180 mm cube). After
changing it, re-export every STL and image with the commands in the README's
Enclosure section. Images come only from `part` values in the model (the hive
lid uses the `hive_lid_fitted` view), never from wrapper files.

Validate printability by slicing every STL with PrusaSlicer and
`case/a1mini.ini` (README has the loop). Treat any new warning as a defect;
the only expected one is "Floating bridge anchors" on `hive_lid`, from the
drill-out layer over its counterbores. Keep `a1mini.ini` in step with the print
settings in the README.

To check fits, intersect parts in a scratch file that includes the model, and
pass `-D 'part="none"'`. A `part` set inside the scratch file is overridden by
the one in the model, so the assembly renders instead:

```bash
openscad -D 'part="none"' -o /tmp/check.stl /tmp/check.scad  # "top level object is empty" = no overlap
```

### Dependencies

- ESPHome with ESP-IDF framework
- ESP-DSP library (automatically added via platformio)
- NAU7802 component (built into ESPHome)
- SHT4x component (built into ESPHome)
- INA226 component (built into ESPHome)
- MAX17043 component (built into ESPHome, drives the onboard MAX17048)
