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
- **Battery Optimised**: Deep sleep between 5-minute measurement cycles
- **Home Assistant Integration**: Automatic sensor discovery and state reporting

## Hardware Requirements

| Component | Description | Qty |
| ----------- | ------------- | ----- |
| ESP32 DevKit | Main microcontroller | 1 |
| INMP441 | I2S MEMS microphone | 1 |
| NAU7802 | 24-bit ADC breakout | 1 |
| 50kg Load Cells | Half-bridge strain gauges | 4 |
| SHT40 | Temperature/humidity sensor | 1 |
| 3.7V LiPo Battery | Power source | 1 |
| TP4056 | Battery charging module (optional) | 1 |

## Wiring

### I2C Bus (SHT40 & NAU7802)

| Signal | ESP32 GPIO |
| -------- | ------------ |
| SDA | GPIO21 |
| SCL | GPIO22 |

### INMP441 Microphone

| INMP441 Pin | ESP32 GPIO |
| ------------- | ------------ |
| WS (LRCLK) | GPIO25 |
| SCK (BCLK) | GPIO26 |
| SD (DOUT) | GPIO33 |
| VDD | 3.3V |
| GND | GND |
| L/R | GND (left channel) |

### Load Cells

Connect the four 50kg load cells in a Wheatstone bridge configuration to the NAU7802:

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

   - Click **Edit** on the new device
   - Replace the contents with the configuration from `beehive-monitor.yaml`

   The bee_audio component is loaded automatically from GitHub via packages:

   ```yaml
   packages:
     bee_audio: github://johnf/esp32-beehive/components/bee_audio
   ```

3. **Configure secrets**

   ESPHome uses Home Assistant's secrets. Add to your `secrets.yaml`:

   ```yaml
   wifi_ssid: 'YourWiFiNetwork'
   wifi_password: 'YourWiFiPassword'
   api_key: 'your-home-assistant-api-encryption-key'
   ```

   Or use the ESPHome Secrets editor (three-dot menu → **Secrets**).

4. **Install to the device**

   - Click **Install** on the device
   - For first-time installation, choose **Plug into this computer** or
     **Manual download** to get the firmware binary
   - Flash via USB using the web installer at <https://web.esphome.io/>
   - Subsequent updates can use **Wirelessly** once connected

## Calibration

### Load Cell Calibration

1. View the device logs in ESPHome dashboard (click **Logs**)
2. With no weight on the platform, note the raw sensor value
3. Place a known weight (e.g., 20 kg) on the platform and note the raw value
4. Click **Edit** on the device and update the calibration:

   ```yaml
   filters:
     - calibrate_linear:
         - <raw_empty> -> 0
         - <raw_loaded> -> <known_weight_kg>
   ```

5. Click **Install** → **Wirelessly** to update the device

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

| State | Current Draw | Duration |
|-------|--------------|----------|
| Deep Sleep | ~10 µA | 5 minutes |
| Active | ~150 mA | ~10 seconds |
| **Average** | **~5 mA** | - |

With a 3000 mAh battery, expect approximately 25 days of operation.

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

  - alias: 'Alert on Queen Piping'
    trigger:
      - platform: state
        entity_id: binary_sensor.beehive_monitor_queen_piping_detected
        to: 'on'
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
- Increase the `samples` value for more averaging

### WiFi connection issues

- Enable `fast_connect: true` to skip scanning
- Position the ESP32 antenna away from metal components
- Consider adding an external antenna

### Deep sleep not working

- Check that `on_boot` script is executing (visible in logs)
- Verify no other components are blocking sleep
- Ensure the `run_duration` is sufficient for sensor readings

## Contributing

Contributions are welcome! Please open an issue or submit a pull request.

## References

- [Bee Audio Analysis Research](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7506584/) - Scientific basis for frequency bands
- [ESPHome Documentation](https://esphome.io/)
- [ESP-DSP Library](https://github.com/espressif/esp-dsp)

## Licence

ISC License - See [LICENSE](LICENSE) for details.
