# ESP32 Beehive Monitor

A battery-powered beehive monitoring system built with ESPHome. Uses FFT-based
audio analysis to detect colony health indicators, along with weight,
temperature, and humidity monitoring.

## Features

- **Audio Analysis**: FFT-based frequency band monitoring to detect queen
  piping, queenless conditions, and pre-swarm behaviour
- **Weight Monitoring**: Track hive weight changes using 4x 50kg load cells via
  NAU7802 ADC
- **Environmental Sensing**: Temperature and humidity via SHT40 sensor
- **Power Monitoring**: Solar and battery current, voltage and power via two INA226s
- **Battery Optimised**: Deep sleep between 5-minute measurement cycles
- **Home Assistant Integration**: Automatic sensor discovery and state reporting

## Hardware Requirements

| Component | Description | Qty |
| ----------- | ------------- | ----- |
| [FeatherS3D](https://unexpectedmaker.com/feathers3d) (ESP32-S3) | Main microcontroller | 1 |
| INMP441 | I2S MEMS microphone (GPIO1, 3, 7) | 1 |
| [Adafruit NAU7802](https://www.adafruit.com/product/4538) | 24-bit ADC breakout | 1 |
| 50kg Load Cells | Half-bridge strain gauges | 4 |
| SHT40 | Temperature/humidity sensor | 1 |
| INA226 x2 | Current/voltage/power monitors | 2 |
| CN3065 | Solar LiPo charge controller | 1 |
| HT7833 | 3.3V LDO voltage regulator | 1 |
| [3.7V 2000mAh LiPo](https://core-electronics.com.au/polymer-lithium-ion-battery-2000mah-38459.html) | Battery (DW01+ PCM) | 1 |
| Solar panel | Input for CN3065 charger | 1 |

## Power Architecture

The battery is charged by the CN3065 solar charge controller. Two INA226 monitors
are wired inline: one on the solar input and one on the battery line before the CN3065.

The HT7833 LDO regulates battery voltage to 3.3V and feeds the FeatherS3D directly
via its **3.3V pin** (not the JST battery connector). The FeatherS3D's onboard charger
(MCP73831) and fuel gauge (MAX17048) are not used because the CN3065's charge protection
would conflict with the onboard charger.

Battery level is estimated from the INA226 battery bus voltage using a LiPo discharge
curve mapping.

The FeatherS3D's **LDO2** (GPIO39) provides a switchable 3.3V rail that is automatically
disabled during deep sleep, used to power-gate sensors that aren't needed during sleep.

### Battery Specifications

| Parameter | Value |
|-----------|-------|
| Nominal voltage | 3.7V |
| Charge voltage | 4.2V (CN3065) |
| Overcharge protection | 4.30V ±0.05V (DW01+ PCM) |
| Over-discharge protection | 2.4V ±0.1V (DW01+ PCM) |
| Capacity | 2000mAh |
| Wiring/connector rating | 1A max |

### I2C Address Map

| Address | Device |
|---------|--------|
| 0x2A | NAU7802 (load cell ADC) |
| 0x40 | INA226 (solar) |
| 0x44 | SHT40 (temperature/humidity) |
| 0x45 | INA226 (battery) |

## Wiring

### I2C Bus (SHT40, NAU7802, INA226s)

The I2C bus must be defined in your device configuration and its `id` passed to the
package via the `i2c_bus_id` substitution (defaults to `i2c_bus`).

| Signal | Default GPIO |
| -------- | ------------ |
| SDA | GPIO8 |
| SCL | GPIO9 |

### INMP441 Microphone

Pin assignments are configurable via substitutions (`i2s_lrclk_pin`, `i2s_bclk_pin`, `i2s_din_pin`).

| INMP441 Pin | Default ESP32 GPIO |
| ------------- | ------------ |
| WS (LRCLK) | GPIO1 |
| SCK (BCLK) | GPIO3 |
| SD (DOUT) | GPIO7 |
| VDD | 3.3V |
| GND | GND |
| L/R | GND (left channel) |

### Load Cells

Each 50kg half-bridge cell has three wires: red (centre tap), white and black.
Place one cell at each corner of the platform and wire them into a full
Wheatstone bridge:

1. Number the cells 1 to 4 clockwise.
2. Join the white wire of each cell to the black wire of the next cell around
   the platform (1→2, 2→3, 3→4, 4→1), forming a ring.
3. Connect the four red wires to the NAU7802 as below.

| Cell (red wire) | NAU7802 |
| --------------- | ------- |
| Cell 1 | E+ |
| Cell 3 | E- |
| Cell 2 | A+ |
| Cell 4 | A- |

If the weight reads negative when load is added, swap A+ and A-.

## Installation

### Prerequisites

- Home Assistant with the ESPHome add-on installed

### Setup

1. **Create a new device in ESPHome**

   - Open the ESPHome dashboard in Home Assistant
   - Click **+ New Device**
   - Choose **Continue** then **Skip this step** (we'll use our own config)
   - Enter a name (e.g., `beehive-monitor`) and click **Next**
   - Select **ESP32** as the device type
   - Click **Skip** to skip the installation for now

2. **Edit the configuration**

   Click **Edit** on the new device and use the following configuration:

   ```yaml
   # Include the beehive monitoring package (pinned to a release tag)
   packages:
     beehive: github://johnf/esphome-beehive/beehive-monitor.yaml@v1.0.0

   # Device configuration (required)
   esphome:
     name: beehive-monitor
     friendly_name: Beehive Monitor

   esp32:
     framework:
       type: esp-idf
     variant: esp32s3
     flash_size: 16MB  # FeatherS3D has 16MB flash

   psram:
     mode: quad
     speed: 80MHz

   # I2C bus (required - must define with id matching i2c_bus_id substitution)
   i2c:
     id: i2c_bus
     sda: GPIO8
     scl: GPIO9
     scan: true
     frequency: 400kHz

   # WiFi configuration (required)
   wifi:
     ssid: !secret wifi_ssid
     password: !secret wifi_password
     fast_connect: true

   # Home Assistant API (required)
   api:
     reboot_timeout: 0s

   # Optional: enable logging during development
   logger:
     hardware_uart: USB_CDC
     level: DEBUG

   # Optional: override default pin assignments, I2C addresses, calibration
   # or audio thresholds (see beehive-monitor.yaml for the full list)
   # substitutions:
   #   i2c_bus_id: "i2c_bus"
   #   i2s_lrclk_pin: "GPIO1"
   #   i2s_bclk_pin: "GPIO3"
   #   i2s_din_pin: "GPIO7"
   #   ina226_solar_address: "0x40"
   #   ina226_battery_address: "0x45"
   #   audio_active_threshold: "-95dB"
   ```

   A complete working example is in [example.yaml](example.yaml).

3. **Configure secrets**

   ESPHome uses Home Assistant's secrets. Add to your `secrets.yaml`:

   ```yaml
   wifi_ssid: 'YourWiFiNetwork'
   wifi_password: 'YourWiFiPassword'
   ```

   Or use the ESPHome Secrets editor (three-dot menu → **Secrets**).

4. **Install to the device**

   - Click **Install** on the device
   - For first-time installation, choose **Plug into this computer** or
     **Manual download** to get the firmware binary
   - Flash via USB using the web installer at <https://web.esphome.io/>
   - Subsequent updates can use **Wirelessly** once connected

### Power Optimisation (recommended for battery operation)

For battery-powered deployments, add these settings to reduce power consumption:

```yaml
wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
  fast_connect: true
  power_save_mode: light
  # Static IP eliminates DHCP negotiation, saving 1-3 seconds per wake
  manual_ip:
    static_ip: 192.168.1.100  # Choose an IP outside your DHCP range
    gateway: 192.168.1.1      # Your router's IP
    subnet: 255.255.255.0

api:
  reboot_timeout: 0s  # Prevent reboots when Home Assistant is unavailable

# Disable UART logging in production
logger:
  level: WARN
  baud_rate: 0
```

## Calibration

### Load Cell Calibration

The package ships with placeholder calibration values that will not match your
load cells. Calibrate before trusting any weight reading.

1. View the device logs in the ESPHome dashboard (click **Logs**). Each wake
   logs seven raw NAU7802 values; use the middle of the range
2. With no weight on the platform, note the raw value
3. Place a known weight on the platform and note the raw value. Use at least
   20 kg so the calibration spans a realistic hive weight; a second known
   weight (e.g. 40 kg) improves accuracy further
4. Let the platform settle for a minute after each change before reading
5. Click **Edit** on the device and set the calibration substitutions:

   ```yaml
   substitutions:
     weight_cal_raw_1: "<raw_empty>"
     weight_cal_kg_1: "0"
     weight_cal_raw_2: "<raw_20kg>"
     weight_cal_kg_2: "20"
     weight_cal_raw_3: "<raw_40kg>"
     weight_cal_kg_3: "40"
     weight_cal_raw_4: "<raw_40kg>"
     weight_cal_kg_4: "40"
   ```

   A least-squares line is fitted through the four points, so repeating a
   point is fine if you only have two known weights.

6. Click **Install** → **Wirelessly** to update the device

### Audio Baseline

Frequency band levels are reported as mean power spectral density in dB
relative to full scale (dB re FS²/Hz). This is independent of `fft_size` and
`frames`, so thresholds survive changes to those settings. Sound Level is
plain dBFS. The INMP441 reaches full scale at roughly 120 dB SPL, so add 120
to convert either figure to an approximate sound pressure level.

Each reading averages four consecutive FFT frames (about one second of audio).
Increase `audio_frames` for steadier band levels at the cost of awake time.

The Modulation Index and Modulation Frequency sensors come from a separate,
longer capture (`audio_modulation_duration`, default 10 s). Abdollahi et al.
(2026) found that the rate at which the buzz amplitude fluctuates predicts
colony strength far better than the average spectrum: weak colonies modulate
below 10 Hz, strong colonies spread up to about 35 Hz, and the 10-25 Hz range
is the most discriminating. The component takes a 100 ms / 12.5 ms hop STFT of
the 150-250 Hz band, then a second FFT over that envelope. Modulation Index is
the percentage of envelope power (1-40 Hz) that lies in 10-25 Hz, so it is
independent of microphone gain and colony loudness. Modulation Frequency is
the strongest modulation rate. Both are new and uncalibrated: log them for a
season alongside inspections before trusting them, and compare night-time
readings (roughly 8-11 pm, when foragers are home) rather than daytime ones.
The band and rate can be overridden with `modulation_band:` and
`modulation_rate:` blocks on the `bee_audio` component. Omit both modulation
sensors to skip the capture entirely and shorten awake time.

The classification thresholds are exposed as substitutions and will need
tuning for your microphone placement, hive size and background noise:

| Substitution | Default | Meaning |
|--------------|---------|---------|
| `audio_active_threshold` | -95dB | Worker band above this → `active` |
| `audio_normal_threshold` | -105dB | Baseline band above this → `normal`, below → `quiet` |
| `audio_queenless_threshold` | 6dB | Both queenless bands this far above baseline → `queenless` |
| `audio_queen_piping_threshold` | 10dB | Tooting or quacking this far above baseline → piping |
| `audio_pre_swarm_centroid` | 400Hz | Centroid above this while active → `pre_swarm` |
| `audio_modulation_duration` | 10s | Audio captured for the modulation spectrum (4-60 s) |

To tune, record the band sensors in Home Assistant for a few days and set
`audio_normal_threshold` between the quietest night-time baseline level and the
daytime level, and `audio_active_threshold` around the worker band level on a
busy afternoon. The band edges themselves can be overridden with a `bands:`
block on the `bee_audio` component, for example:

```yaml
bee_audio:
  bands:
    worker:
      low: 170Hz
      high: 280Hz
```

### Audio Baseline

The audio thresholds are set based on research values. You may need to adjust them in `components/bee_audio/bee_audio.cpp` based on:

- Microphone placement within the hive
- Hive size and colony population
- Background noise levels

## Sensors

### Frequency Bands

| Sensor | Frequency Range | Purpose |
|--------|-----------------|---------|
| Low Frequency | 60-100 Hz | Low frequency content |
| Baseline Hum | 100-200 Hz | Normal colony activity |
| Worker Activity | 180-260 Hz | Worker bee flight |
| Queen Quacking | 200-350 Hz | Virgin queen in cell |
| Queen Tooting | 350-500 Hz | Emerged virgin queen |
| Queenless Mid | 478-677 Hz | Queenless indicator |
| Queenless High | 876-1080 Hz | Queenless indicator |

### Derived Metrics

| Sensor | Unit | Description |
|--------|------|-------------|
| Dominant Frequency | Hz | Peak frequency in 60-600 Hz range |
| Sound Level | dB | Overall RMS sound level |
| Spectral Centroid | Hz | Centre of mass of spectrum |
| Modulation Index | % | Share of 150-250 Hz envelope power modulating at 10-25 Hz |
| Modulation Frequency | Hz | Strongest modulation rate of the 150-250 Hz envelope (1-40 Hz) |

### Power Monitoring

| Sensor | Unit | Description |
|--------|------|-------------|
| Solar Bus Voltage | V | Solar panel voltage |
| Solar Current | A | Current from solar panel |
| Solar Power | W | Solar power input |
| Battery Bus Voltage | V | Battery terminal voltage |
| Battery Current | A | Battery charge/discharge current |
| Battery Power | W | Battery power flow |
| Battery Level | % | Estimated SoC from resting voltage curve |
| Battery Charging | on/off | Solar current above `charging_current_threshold` |

Battery Level is only updated while the battery is not charging. Under solar
charge the CN3065 holds the bus near 4.2 V regardless of state of charge, so
the last resting-voltage estimate is kept until the next reading after dark.

### Hive State Classification

The system classifies the hive into one of these states:

| State | Description |
|-------|-------------|
| `quiet` | Very low activity, possibly night-time or cold |
| `normal` | Typical colony activity |
| `active` | Elevated worker activity |
| `queen_activity` | Queen piping detected (tooting or quacking) |
| `queenless` | Elevated mid/high frequency bands indicating queenless colony |
| `pre_swarm` | Elevated spectral centroid with high activity |

## Power Consumption

Indicative figures only; the battery INA226 reports the real draw of your build.

| State | Current Draw | Duration |
|-------|--------------|----------|
| Deep Sleep | ~10 µA (ESP32 only, sensor rail off) | 5 minutes |
| Active | ~150 mA | ~5-15 seconds (WiFi connect, ~1 s audio, ~1 s weight) |
| **Average** | **~5 mA** | - |

**Note**: Any sensor not powered from LDO2 draws standby current during deep
sleep. See the Hardware TODO section for power gating improvements.

With a 2000 mAh battery and solar charging, runtime depends on solar input. Without
solar, expect roughly two weeks.

## Home Assistant

Once connected, sensors will automatically appear in Home Assistant. Example automations:

```yaml
automation:
  - alias: 'Alert on Queenless Hive'
    trigger:
      - platform: state
        entity_id: text_sensor.beehive_monitor_hive_state
        to: 'queenless'
        for:
          hours: 1
    action:
      - service: notify.mobile_app
        data:
          title: 'Beehive Alert'
          message: 'Hive may be queenless - check colony!'

  # Piping is detected from a one-second sample every five minutes, so a
  # single detection may be noise. Require it to persist across two readings.
  - alias: 'Alert on Queen Piping'
    trigger:
      - platform: state
        entity_id: binary_sensor.beehive_monitor_queen_piping_detected
        to: 'on'
        for:
          minutes: 6
    action:
      - service: notify.mobile_app
        data:
          title: 'Beehive Alert'
          message: 'Queen piping detected - possible swarm preparation'
```

## Troubleshooting

### No audio data

- Check INMP441 wiring, especially L/R pin (must be grounded for left channel)
- Verify 3.3V power supply is stable
- Check I2S pin assignments in YAML

### Weight readings unstable

- Ensure load cells are properly mounted and not touching the frame
- Check for loose connections on the NAU7802
- Recalibrate with weights spanning the real hive weight range; extrapolating
  from a few kilograms magnifies noise
- Each reading is already the median of five raw samples; if the log shows
  "No measurements ready!" on every sample, check the NAU7802 power and I2C

### WiFi connection issues

- Enable `fast_connect: true` to skip scanning
- Position the ESP32 antenna away from metal components
- Consider adding an external antenna

### Deep sleep not working

- Check that `on_boot` script is executing (visible in logs)
- Verify no other components are blocking sleep
- Ensure the `run_duration` is sufficient for sensor readings

## Hardware TODO

- [ ] **Schottky diode on 3.3V input**: Add a Schottky diode (e.g. BAT43) between the
  HT7833 output and the FeatherS3D 3.3V pin (cathode toward FeatherS3D). This prevents
  the onboard LDO from back-feeding into VR1 when USB is connected for programming.
  The ~0.2V forward drop means the HT7833's 3.3V output delivers ~3.1V — this is within
  the ESP32-S3's operating range (2.3–3.6V) but verify sensors are stable at that voltage,
  or use an adjustable LDO set to ~3.5V instead.

- [ ] **Prevent-sleep jumper/switch on GPIO11**: Add a 2-pin header or switch connected
  to GPIO11 for field use. When jumpered/closed to 3.3V, this prevents deep sleep for
  debugging or OTA updates. Currently the pin is configured with an internal pull-down but
  has no physical mechanism in the schematic.

- [ ] **Decoupling capacitors on sensor modules**: Add 100nF ceramic capacitors close to
  the VCC pins of the INA226 modules and INMP441. Long wires in a beehive environment
  can pick up noise, and the INMP441 is particularly sensitive as it feeds FFT analysis.
  The Adafruit NAU7802 and SHT40 breakouts have onboard decoupling already.

## Development

Clone the repository and build the example configuration, which uses the
component from the local `components/` directory rather than the pinned
release:

```bash
cat > secrets.yaml <<'EOF'
wifi_ssid: my-ssid
wifi_password: my-password
EOF
esphome config example.yaml
esphome compile example.yaml
```

CI runs the same two commands on every push and pull request.

`setup-dev.sh` clones ESPHome, ESP-IDF and ESP-DSP into `cdeps/` so that
`clangd` can resolve headers when editing the C++ component.

### Releasing

Device configurations pin the package and component to a tag. To release:

1. Update `bee_audio_source` in `beehive-monitor.yaml` and the `packages` URL
   in this README to the new tag
2. Commit, then `git tag vX.Y.Z && git push --tags`

## Contributing

Contributions are welcome! Please open an issue or submit a pull request.

## References

- [Bee Audio Analysis Research](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7506584/) - Scientific basis for frequency bands
- [Abdollahi et al. 2026, Modulation Tensorgrams for Colony Strength](https://arxiv.org/abs/2607.20386) - Basis for the modulation spectrum sensors
- [ESPHome Documentation](https://esphome.io/)
- [ESP-DSP Library](https://github.com/espressif/esp-dsp)
- <https://how2electronics.com/how-to-use-ina226-dc-current-sensor-with-arduino/>

## Licence

ISC License - See [LICENSE](LICENSE) for details.
