#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"

#include "driver/i2s_std.h"
#include "dsps_fft2r.h"
#include "dsps_wind_hann.h"
#include "esp_heap_caps.h"

namespace esphome {
namespace bee_audio {

struct FrequencyBand {
  float low_hz;
  float high_hz;
};

// Research-based frequency bands for bee monitoring
inline constexpr FrequencyBand BAND_LOW_FREQ = {60.0f, 100.0f};
inline constexpr FrequencyBand BAND_BASELINE = {100.0f, 200.0f};
inline constexpr FrequencyBand BAND_WORKER = {180.0f, 260.0f};
inline constexpr FrequencyBand BAND_QUACKING = {200.0f, 350.0f};
inline constexpr FrequencyBand BAND_TOOTING = {350.0f, 500.0f};
inline constexpr FrequencyBand BAND_QUEENLESS_MID = {478.0f, 677.0f};
inline constexpr FrequencyBand BAND_QUEENLESS_HIGH = {876.0f, 1080.0f};

enum class HiveState {
  QUIET,
  NORMAL,
  ACTIVE,
  QUEEN_ACTIVITY,
  QUEENLESS,
  PRE_SWARM
};

class BeeAudioComponent : public PollingComponent {
public:
  void setup() override;
  void update() override;
  void dump_config() override;
  ~BeeAudioComponent();
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Configuration setters
  void set_i2s_lrclk_pin(int pin) { this->i2s_lrclk_pin_ = pin; }
  void set_i2s_bclk_pin(int pin) { this->i2s_bclk_pin_ = pin; }
  void set_i2s_din_pin(int pin) { this->i2s_din_pin_ = pin; }
  void set_sample_rate(uint32_t rate) { this->sample_rate_ = rate; }
  void set_fft_size(size_t size) { this->fft_size_ = size; }

  // Sensor setters - frequency bands
  void set_band_low_freq_sensor(sensor::Sensor *sensor) {
    this->band_low_freq_sensor_ = sensor;
  }
  void set_band_baseline_sensor(sensor::Sensor *sensor) {
    this->band_baseline_sensor_ = sensor;
  }
  void set_band_worker_sensor(sensor::Sensor *sensor) {
    this->band_worker_sensor_ = sensor;
  }
  void set_band_quacking_sensor(sensor::Sensor *sensor) {
    this->band_quacking_sensor_ = sensor;
  }
  void set_band_tooting_sensor(sensor::Sensor *sensor) {
    this->band_tooting_sensor_ = sensor;
  }
  void set_band_queenless_mid_sensor(sensor::Sensor *sensor) {
    this->band_queenless_mid_sensor_ = sensor;
  }
  void set_band_queenless_high_sensor(sensor::Sensor *sensor) {
    this->band_queenless_high_sensor_ = sensor;
  }

  // Sensor setters - derived metrics
  void set_dominant_frequency_sensor(sensor::Sensor *sensor) {
    this->dominant_frequency_sensor_ = sensor;
  }
  void set_sound_level_rms_sensor(sensor::Sensor *sensor) {
    this->sound_level_rms_sensor_ = sensor;
  }
  void set_spectral_centroid_sensor(sensor::Sensor *sensor) {
    this->spectral_centroid_sensor_ = sensor;
  }

  // Binary sensor setter
  void set_queen_piping_sensor(binary_sensor::BinarySensor *sensor) {
    this->queen_piping_sensor_ = sensor;
  }

  // Text sensor setter
  void set_hive_state_sensor(text_sensor::TextSensor *sensor) {
    this->hive_state_sensor_ = sensor;
  }

protected:
  // I2S configuration
  int i2s_lrclk_pin_{0};
  int i2s_bclk_pin_{0};
  int i2s_din_pin_{0};
  uint32_t sample_rate_{8000};
  size_t fft_size_{2048};

  // I2S handle
  i2s_chan_handle_t rx_chan_{nullptr};

  // Audio buffers (allocated dynamically)
  int32_t *raw_samples_{nullptr};
  float *samples_{nullptr};
  float *fft_data_{nullptr};
  float *magnitude_{nullptr};
  float *window_{nullptr};

  // Frequency resolution (Hz per bin)
  float freq_resolution_{0.0f};

  // Frequency band sensors
  sensor::Sensor *band_low_freq_sensor_{nullptr};
  sensor::Sensor *band_baseline_sensor_{nullptr};
  sensor::Sensor *band_worker_sensor_{nullptr};
  sensor::Sensor *band_quacking_sensor_{nullptr};
  sensor::Sensor *band_tooting_sensor_{nullptr};
  sensor::Sensor *band_queenless_mid_sensor_{nullptr};
  sensor::Sensor *band_queenless_high_sensor_{nullptr};

  // Derived metric sensors
  sensor::Sensor *dominant_frequency_sensor_{nullptr};
  sensor::Sensor *sound_level_rms_sensor_{nullptr};
  sensor::Sensor *spectral_centroid_sensor_{nullptr};

  binary_sensor::BinarySensor *queen_piping_sensor_{nullptr};

  text_sensor::TextSensor *hive_state_sensor_{nullptr};

  // Cached band powers (calculated once per update)
  float cached_band_baseline_{0.0f};
  float cached_band_worker_{0.0f};
  float cached_band_tooting_{0.0f};
  float cached_band_quacking_{0.0f};
  float cached_band_queenless_mid_{0.0f};
  float cached_band_queenless_high_{0.0f};

  // Internal methods
  bool init_i2s_();
  void deinit_i2s_();
  bool allocate_buffers_();
  void free_buffers_();
  bool capture_audio_();
  void compute_fft_();
  float calculate_band_power_(const FrequencyBand &band);
  float calculate_dominant_frequency_();
  float calculate_rms_();
  float calculate_spectral_centroid_();
  bool detect_queen_piping_();
  HiveState classify_hive_state_();
  const char *hive_state_to_string_(HiveState state);
  int hz_to_bin_(float hz);
};

} // namespace bee_audio
} // namespace esphome
