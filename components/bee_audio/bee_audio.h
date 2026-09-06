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

enum Band : uint8_t {
  BAND_LOW_FREQ = 0,
  BAND_BASELINE,
  BAND_WORKER,
  BAND_QUACKING,
  BAND_TOOTING,
  BAND_QUEENLESS_MID,
  BAND_QUEENLESS_HIGH,
  BAND_COUNT
};

struct FrequencyBand {
  float low_hz;
  float high_hz;
};

// Research-based frequency bands for bee monitoring, indexed by Band
inline constexpr FrequencyBand DEFAULT_BANDS[BAND_COUNT] = {
    {60.0f, 100.0f},   // BAND_LOW_FREQ
    {100.0f, 200.0f},  // BAND_BASELINE
    {180.0f, 260.0f},  // BAND_WORKER
    {200.0f, 350.0f},  // BAND_QUACKING
    {350.0f, 500.0f},  // BAND_TOOTING
    {478.0f, 677.0f},  // BAND_QUEENLESS_MID
    {876.0f, 1080.0f}, // BAND_QUEENLESS_HIGH
};

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

  // Capture configuration
  void set_i2s_lrclk_pin(int pin) { this->i2s_lrclk_pin_ = pin; }
  void set_i2s_bclk_pin(int pin) { this->i2s_bclk_pin_ = pin; }
  void set_i2s_din_pin(int pin) { this->i2s_din_pin_ = pin; }
  void set_sample_rate(uint32_t rate) { this->sample_rate_ = rate; }
  void set_fft_size(size_t size) { this->fft_size_ = size; }
  void set_frames(uint8_t frames) { this->frames_ = frames; }

  // Analysis configuration
  void set_band(Band band, float low_hz, float high_hz) {
    this->bands_[band] = {low_hz, high_hz};
  }
  void set_queenless_threshold(float db) { this->queenless_threshold_db_ = db; }
  void set_queen_piping_threshold(float db) {
    this->queen_piping_threshold_db_ = db;
  }
  void set_active_threshold(float db) { this->active_threshold_db_ = db; }
  void set_normal_threshold(float db) { this->normal_threshold_db_ = db; }
  void set_pre_swarm_centroid(float hz) { this->pre_swarm_centroid_hz_ = hz; }

  // Modulation spectrum configuration
  void set_modulation_duration(uint32_t ms) {
    this->modulation_duration_ms_ = ms;
  }
  void set_modulation_band(float low_hz, float high_hz) {
    this->modulation_band_ = {low_hz, high_hz};
  }
  void set_modulation_rate(float low_hz, float high_hz) {
    this->modulation_rate_ = {low_hz, high_hz};
  }

  // Sensors
  void set_band_sensor(Band band, sensor::Sensor *sensor) {
    this->band_sensors_[band] = sensor;
  }
  void set_dominant_frequency_sensor(sensor::Sensor *sensor) {
    this->dominant_frequency_sensor_ = sensor;
  }
  void set_sound_level_rms_sensor(sensor::Sensor *sensor) {
    this->sound_level_rms_sensor_ = sensor;
  }
  void set_spectral_centroid_sensor(sensor::Sensor *sensor) {
    this->spectral_centroid_sensor_ = sensor;
  }
  void set_modulation_index_sensor(sensor::Sensor *sensor) {
    this->modulation_index_sensor_ = sensor;
  }
  void set_modulation_frequency_sensor(sensor::Sensor *sensor) {
    this->modulation_frequency_sensor_ = sensor;
  }
  void set_queen_piping_sensor(binary_sensor::BinarySensor *sensor) {
    this->queen_piping_sensor_ = sensor;
  }
  void set_hive_state_sensor(text_sensor::TextSensor *sensor) {
    this->hive_state_sensor_ = sensor;
  }

protected:
  int i2s_lrclk_pin_{0};
  int i2s_bclk_pin_{0};
  int i2s_din_pin_{0};
  uint32_t sample_rate_{8000};
  size_t fft_size_{2048};
  uint8_t frames_{4};

  FrequencyBand bands_[BAND_COUNT] = {
      DEFAULT_BANDS[0], DEFAULT_BANDS[1], DEFAULT_BANDS[2], DEFAULT_BANDS[3],
      DEFAULT_BANDS[4], DEFAULT_BANDS[5], DEFAULT_BANDS[6]};
  float queenless_threshold_db_{6.0f};
  float queen_piping_threshold_db_{10.0f};
  float active_threshold_db_{-95.0f};
  float normal_threshold_db_{-105.0f};
  float pre_swarm_centroid_hz_{400.0f};

  uint32_t modulation_duration_ms_{10000};
  FrequencyBand modulation_band_{150.0f, 250.0f};
  FrequencyBand modulation_rate_{10.0f, 25.0f};

  i2s_chan_handle_t rx_chan_{nullptr};
  bool fft_initialised_{false};

  int32_t *raw_samples_{nullptr}; // fft_size_ samples, DMA capable
  float *fft_data_{nullptr};      // 2 * fft_size_ interleaved re/im
  float *window_{nullptr};        // fft_size_
  float *psd_{nullptr};           // fft_size_ / 2, one-sided PSD (FS^2/Hz)

  float freq_resolution_{0.0f};
  float window_power_sum_{0.0f}; // sum of window^2, for PSD normalisation
  float rms_db_{-200.0f};

  // Modulation spectrum: a 100 ms / 12.5 ms hop STFT gives the band envelope
  // at ~80 Hz, which is then analysed with a second FFT in 256-hop segments.
  size_t stft_window_{0};
  size_t stft_hop_{0};
  size_t stft_fft_size_{0};
  size_t envelope_len_{0};
  int32_t *hop_samples_{nullptr};  // stft_hop_ samples, DMA capable
  float *stft_input_{nullptr};     // stft_window_ samples, sliding
  float *stft_window_fn_{nullptr}; // stft_window_
  float *stft_data_{nullptr};      // 2 * stft_fft_size_ interleaved re/im
  float *envelope_{nullptr};       // envelope_len_
  float *mod_window_fn_{nullptr};  // MOD_SEGMENT
  float *mod_psd_{nullptr};        // MOD_SEGMENT / 2
  float modulation_index_{0.0f};
  float modulation_frequency_{0.0f};

  sensor::Sensor *band_sensors_[BAND_COUNT] = {};
  sensor::Sensor *dominant_frequency_sensor_{nullptr};
  sensor::Sensor *sound_level_rms_sensor_{nullptr};
  sensor::Sensor *spectral_centroid_sensor_{nullptr};
  sensor::Sensor *modulation_index_sensor_{nullptr};
  sensor::Sensor *modulation_frequency_sensor_{nullptr};
  binary_sensor::BinarySensor *queen_piping_sensor_{nullptr};
  text_sensor::TextSensor *hive_state_sensor_{nullptr};

  float band_power_[BAND_COUNT] = {};

  bool init_i2s_();
  void deinit_i2s_();
  bool allocate_buffers_();
  void free_buffers_();
  bool discard_startup_samples_();
  bool capture_and_analyse_();
  bool modulation_enabled_() const {
    return this->modulation_index_sensor_ != nullptr ||
           this->modulation_frequency_sensor_ != nullptr;
  }
  bool allocate_modulation_buffers_();
  bool capture_modulation_();
  void analyse_envelope_();
  float peak_frequency_(const float *psd, int start_bin, int end_bin,
                        float resolution) const;
  float calculate_band_power_(const FrequencyBand &band) const;
  float calculate_dominant_frequency_() const;
  float calculate_spectral_centroid_() const;
  bool detect_queen_piping_() const;
  HiveState classify_hive_state_(float centroid) const;
  static const char *hive_state_to_string_(HiveState state);
  int hz_to_bin_(float hz) const;
};

} // namespace bee_audio
} // namespace esphome
