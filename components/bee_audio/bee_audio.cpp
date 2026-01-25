#include "bee_audio.h"
#include "esphome/core/log.h"
#include <cmath>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace esphome {
namespace bee_audio {

static const char *const TAG = "bee_audio";

// Thresholds for classification (in dB relative to baseline)
static const float QUEENLESS_THRESHOLD_DB = 6.0f;
static const float QUEEN_PIPING_THRESHOLD_DB = 10.0f;
static const float ACTIVE_THRESHOLD_DB = -30.0f;
static const float NORMAL_THRESHOLD_DB = -40.0f;
static const float PRE_SWARM_CENTROID_HZ = 400.0f;

void BeeAudioComponent::setup() {
  ESP_LOGD(TAG, "Setting up Bee Audio...");

  // Calculate frequency resolution
  this->freq_resolution_ = static_cast<float>(this->sample_rate_) /
                           static_cast<float>(this->fft_size_);
  ESP_LOGD(TAG, "Frequency resolution: %.2f Hz/bin", this->freq_resolution_);

  // Allocate buffers
  if (!this->allocate_buffers_()) {
    this->mark_failed();

    return;
  }

  // Generate Hanning window manually (avoids ESP-DSP alignment issues)
  // Done immediately after buffer allocation, before I2S init
  for (size_t i = 0; i < this->fft_size_; i++) {
    this->window_[i] =
        0.5f * (1.0f - cosf(2.0f * M_PI * static_cast<float>(i) /
                            static_cast<float>(this->fft_size_ - 1)));
  }

  // Initialise I2S
  if (!this->init_i2s_()) {
    this->free_buffers_();
    this->mark_failed();

    return;
  }

  ESP_LOGCONFIG(TAG, "Bee Audio initialised successfully");
}

void BeeAudioComponent::update() {
  ESP_LOGD(TAG, "Starting audio capture and analysis...");

  // Capture audio samples
  if (!this->capture_audio_()) {
    ESP_LOGW(TAG, "Audio capture failed");

    return;
  }

  // Compute FFT
  this->compute_fft_();

  // Calculate and publish frequency band powers
  if (this->band_low_freq_sensor_ != nullptr) {
    float power = this->calculate_band_power_(BAND_LOW_FREQ);
    this->band_low_freq_sensor_->publish_state(power);
  }

  if (this->band_baseline_sensor_ != nullptr) {
    float power = this->calculate_band_power_(BAND_BASELINE);
    this->band_baseline_sensor_->publish_state(power);
  }

  if (this->band_worker_sensor_ != nullptr) {
    float power = this->calculate_band_power_(BAND_WORKER);
    this->band_worker_sensor_->publish_state(power);
  }

  if (this->band_quacking_sensor_ != nullptr) {
    float power = this->calculate_band_power_(BAND_QUACKING);
    this->band_quacking_sensor_->publish_state(power);
  }

  if (this->band_tooting_sensor_ != nullptr) {
    float power = this->calculate_band_power_(BAND_TOOTING);
    this->band_tooting_sensor_->publish_state(power);
  }

  if (this->band_queenless_mid_sensor_ != nullptr) {
    float power = this->calculate_band_power_(BAND_QUEENLESS_MID);
    this->band_queenless_mid_sensor_->publish_state(power);
  }

  if (this->band_queenless_high_sensor_ != nullptr) {
    float power = this->calculate_band_power_(BAND_QUEENLESS_HIGH);
    this->band_queenless_high_sensor_->publish_state(power);
  }

  // Calculate and publish derived metrics
  if (this->dominant_frequency_sensor_ != nullptr) {
    float freq = this->calculate_dominant_frequency_();
    this->dominant_frequency_sensor_->publish_state(freq);
  }

  if (this->sound_level_rms_sensor_ != nullptr) {
    float rms = this->calculate_rms_();
    this->sound_level_rms_sensor_->publish_state(rms);
  }

  if (this->spectral_centroid_sensor_ != nullptr) {
    float centroid = this->calculate_spectral_centroid_();
    this->spectral_centroid_sensor_->publish_state(centroid);
  }

  // Detect queen piping
  if (this->queen_piping_sensor_ != nullptr) {
    bool piping = this->detect_queen_piping_();
    this->queen_piping_sensor_->publish_state(piping);
  }

  // Classify hive state
  if (this->hive_state_sensor_ != nullptr) {
    HiveState state = this->classify_hive_state_();
    this->hive_state_sensor_->publish_state(this->hive_state_to_string_(state));
  }

  ESP_LOGD(TAG, "Audio analysis complete");
}

void BeeAudioComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Bee Audio:");
  ESP_LOGCONFIG(TAG, "  I2S LRCLK Pin: GPIO%d", this->i2s_lrclk_pin_);
  ESP_LOGCONFIG(TAG, "  I2S BCLK Pin: GPIO%d", this->i2s_bclk_pin_);
  ESP_LOGCONFIG(TAG, "  I2S DIN Pin: GPIO%d", this->i2s_din_pin_);
  ESP_LOGCONFIG(TAG, "  Sample Rate: %u Hz", this->sample_rate_);
  ESP_LOGCONFIG(TAG, "  FFT Size: %u", this->fft_size_);
  ESP_LOGCONFIG(TAG, "  Frequency Resolution: %.2f Hz/bin",
                this->freq_resolution_);
}

bool BeeAudioComponent::init_i2s_() {
  ESP_LOGD(TAG, "Initialising I2S...");

  // Channel configuration - increase DMA buffers for our read size
  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num = 8;
  chan_cfg.dma_frame_num = 512; // 8 * 512 = 4096 samples of DMA buffer

  esp_err_t ret = i2s_new_channel(&chan_cfg, nullptr, &this->rx_chan_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(ret));

    return false;
  }

  // Standard mode configuration for INMP441
  // INMP441 uses MSB-aligned (left-justified) format, not Philips
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(this->sample_rate_),
      .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                  I2S_SLOT_MODE_MONO),
      .gpio_cfg =
          {
              .mclk = I2S_GPIO_UNUSED,
              .bclk = static_cast<gpio_num_t>(this->i2s_bclk_pin_),
              .ws = static_cast<gpio_num_t>(this->i2s_lrclk_pin_),
              .dout = I2S_GPIO_UNUSED,
              .din = static_cast<gpio_num_t>(this->i2s_din_pin_),
              .invert_flags =
                  {
                      .mclk_inv = false,
                      .bclk_inv = false,
                      .ws_inv = false,
                  },
          },
  };

  // INMP441 outputs on left channel when L/R is grounded
  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

  ret = i2s_channel_init_std_mode(this->rx_chan_, &std_cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to init I2S standard mode: %s", esp_err_to_name(ret));
    i2s_del_channel(this->rx_chan_);

    return false;
  }

  // Don't enable here - enable/disable around actual reads to avoid DMA issues
  ESP_LOGD(TAG, "I2S initialised successfully (not yet enabled)");

  return true;
}

void BeeAudioComponent::deinit_i2s_() {
  if (this->rx_chan_ != nullptr) {
    i2s_channel_disable(this->rx_chan_);
    i2s_del_channel(this->rx_chan_);
    this->rx_chan_ = nullptr;
  }
}

bool BeeAudioComponent::allocate_buffers_() {
  ESP_LOGD(TAG, "Allocating buffers for FFT size %zu...", this->fft_size_);

  // ESP-DSP requires 16-byte aligned memory for SIMD operations
  const size_t alignment = 16;

  // Float buffers need MALLOC_CAP_8BIT to ensure byte-addressable memory
  // (IRAM cannot be accessed by float instructions on ESP32)
  const uint32_t float_caps = MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL;

  // Raw I2S samples (32-bit signed) - needs DMA capability
  this->raw_samples_ = static_cast<int32_t *>(
      heap_caps_aligned_alloc(alignment, this->fft_size_ * sizeof(int32_t),
                              MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  if (this->raw_samples_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate raw_samples buffer");

    return false;
  }

  // Normalised float samples - 16-byte aligned for ESP-DSP
  this->samples_ = static_cast<float *>(heap_caps_aligned_alloc(
      alignment, this->fft_size_ * sizeof(float), float_caps));
  if (this->samples_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate samples buffer");

    return false;
  }

  // FFT data (interleaved real/imag, so 2x size) - 16-byte aligned for ESP-DSP
  this->fft_data_ = static_cast<float *>(heap_caps_aligned_alloc(
      alignment, this->fft_size_ * 2 * sizeof(float), float_caps));
  if (this->fft_data_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate fft_data buffer");

    return false;
  }

  // Magnitude spectrum - 16-byte aligned for ESP-DSP
  this->magnitude_ = static_cast<float *>(heap_caps_aligned_alloc(
      alignment, this->fft_size_ * sizeof(float), float_caps));
  if (this->magnitude_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate magnitude buffer");

    return false;
  }

  // Hanning window - 16-byte aligned for ESP-DSP
  this->window_ = static_cast<float *>(heap_caps_aligned_alloc(
      alignment, this->fft_size_ * sizeof(float), float_caps));
  if (this->window_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate window buffer");

    return false;
  }

  size_t total_bytes = this->fft_size_ * sizeof(int32_t) +
                       this->fft_size_ * sizeof(float) * 4 +
                       this->fft_size_ * 2 * sizeof(float);
  ESP_LOGD(TAG, "Allocated %.1f KB for audio buffers", total_bytes / 1024.0f);

  return true;
}

void BeeAudioComponent::free_buffers_() {
  if (this->raw_samples_ != nullptr) {
    heap_caps_free(this->raw_samples_);
    this->raw_samples_ = nullptr;
  }
  if (this->samples_ != nullptr) {
    heap_caps_free(this->samples_);
    this->samples_ = nullptr;
  }
  if (this->fft_data_ != nullptr) {
    heap_caps_free(this->fft_data_);
    this->fft_data_ = nullptr;
  }
  if (this->magnitude_ != nullptr) {
    heap_caps_free(this->magnitude_);
    this->magnitude_ = nullptr;
  }
  if (this->window_ != nullptr) {
    heap_caps_free(this->window_);
    this->window_ = nullptr;
  }
}

bool BeeAudioComponent::capture_audio_() {
  // Enable I2S channel for capture
  esp_err_t ret = i2s_channel_enable(this->rx_chan_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable I2S channel: %s", esp_err_to_name(ret));

    return false;
  }

  // Small delay to let DMA stabilise
  vTaskDelay(pdMS_TO_TICKS(10));

  size_t bytes_to_read = this->fft_size_ * sizeof(int32_t);
  size_t bytes_read = 0;

  ret = i2s_channel_read(this->rx_chan_, this->raw_samples_, bytes_to_read,
                         &bytes_read, pdMS_TO_TICKS(1000));

  // Disable I2S channel after capture
  i2s_channel_disable(this->rx_chan_);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2S read failed: %s", esp_err_to_name(ret));

    return false;
  }

  if (bytes_read != bytes_to_read) {
    ESP_LOGW(TAG, "I2S read incomplete: %zu/%zu bytes", bytes_read,
             bytes_to_read);

    return false;
  }

  // INMP441 outputs 24-bit data left-justified in 32-bit word
  // Convert to normalised float [-1.0, 1.0]
  const float scale = 1.0f / 8388608.0f; // 2^23
  for (size_t i = 0; i < this->fft_size_; i++) {
    // Shift right by 8 to get 24-bit value, then normalise
    int32_t sample = this->raw_samples_[i] >> 8;
    this->samples_[i] = static_cast<float>(sample) * scale;
  }

  return true;
}

void BeeAudioComponent::compute_fft_() {
  // Apply Hanning window and prepare for FFT
  // FFT input is interleaved [real0, imag0, real1, imag1, ...]
  for (size_t i = 0; i < this->fft_size_; i++) {
    this->fft_data_[i * 2] = this->samples_[i] * this->window_[i];
    this->fft_data_[i * 2 + 1] = 0.0f;
  }

  // Perform in-place FFT
  dsps_fft2r_fc32(this->fft_data_, this->fft_size_);

  // Bit-reverse the output
  dsps_bit_rev_fc32(this->fft_data_, this->fft_size_);

  // Calculate magnitude spectrum
  // Only need first half (positive frequencies)
  size_t num_bins = this->fft_size_ / 2;
  for (size_t i = 0; i < num_bins; i++) {
    float real = this->fft_data_[i * 2];
    float imag = this->fft_data_[i * 2 + 1];
    this->magnitude_[i] = sqrtf(real * real + imag * imag);
  }
}

int BeeAudioComponent::hz_to_bin_(float hz) {
  int bin = static_cast<int>(hz / this->freq_resolution_);
  int max_bin = static_cast<int>(this->fft_size_ / 2 - 1);
  if (bin < 0) {
    bin = 0;
  }
  if (bin > max_bin) {
    bin = max_bin;
  }

  return bin;
}

float BeeAudioComponent::calculate_band_power_(const FrequencyBand &band) {
  int start_bin = this->hz_to_bin_(band.low_hz);
  int end_bin = this->hz_to_bin_(band.high_hz);

  float sum_squared = 0.0f;
  int count = 0;

  for (int i = start_bin; i <= end_bin; i++) {
    sum_squared += this->magnitude_[i] * this->magnitude_[i];
    count++;
  }

  if (count == 0) {

    return -100.0f; // Very quiet
  }

  // Average power and convert to dB
  float avg_power = sum_squared / static_cast<float>(count);
  float db = 10.0f * log10f(avg_power + 1e-10f);

  return db;
}

float BeeAudioComponent::calculate_dominant_frequency_() {
  // Find peak in 60-600 Hz range
  int start_bin = this->hz_to_bin_(60.0f);
  int end_bin = this->hz_to_bin_(600.0f);

  float max_mag = 0.0f;
  int max_bin = start_bin;

  for (int i = start_bin; i <= end_bin; i++) {
    if (this->magnitude_[i] > max_mag) {
      max_mag = this->magnitude_[i];
      max_bin = i;
    }
  }

  // Parabolic interpolation for better frequency resolution
  if (max_bin > start_bin && max_bin < end_bin) {
    float y0 = this->magnitude_[max_bin - 1];
    float y1 = this->magnitude_[max_bin];
    float y2 = this->magnitude_[max_bin + 1];

    float delta = 0.5f * (y0 - y2) / (y0 - 2.0f * y1 + y2 + 1e-10f);
    float refined_bin = static_cast<float>(max_bin) + delta;

    return refined_bin * this->freq_resolution_;
  }

  return static_cast<float>(max_bin) * this->freq_resolution_;
}

float BeeAudioComponent::calculate_rms_() {
  float sum_squared = 0.0f;

  for (size_t i = 0; i < this->fft_size_; i++) {
    sum_squared += this->samples_[i] * this->samples_[i];
  }

  float rms = sqrtf(sum_squared / static_cast<float>(this->fft_size_));

  // Convert to dB (relative to full scale)
  float db = 20.0f * log10f(rms + 1e-10f);

  return db;
}

float BeeAudioComponent::calculate_spectral_centroid_() {
  // Spectral centroid = weighted mean of frequencies
  // Only consider 60-1200 Hz range for bee-relevant signals
  int start_bin = this->hz_to_bin_(60.0f);
  int end_bin = this->hz_to_bin_(1200.0f);

  float weighted_sum = 0.0f;
  float magnitude_sum = 0.0f;

  for (int i = start_bin; i <= end_bin; i++) {
    float freq = static_cast<float>(i) * this->freq_resolution_;
    weighted_sum += freq * this->magnitude_[i];
    magnitude_sum += this->magnitude_[i];
  }

  if (magnitude_sum < 1e-10f) {

    return 0.0f;
  }

  return weighted_sum / magnitude_sum;
}

bool BeeAudioComponent::detect_queen_piping_() {
  // Simple detection: check if tooting or quacking bands are significantly
  // elevated
  float baseline_power = this->calculate_band_power_(BAND_BASELINE);
  float tooting_power = this->calculate_band_power_(BAND_TOOTING);
  float quacking_power = this->calculate_band_power_(BAND_QUACKING);

  bool tooting_detected =
      (tooting_power - baseline_power) > QUEEN_PIPING_THRESHOLD_DB;
  bool quacking_detected =
      (quacking_power - baseline_power) > QUEEN_PIPING_THRESHOLD_DB;

  if (tooting_detected || quacking_detected) {
    ESP_LOGD(TAG,
             "Queen piping detected! Tooting: %.1f dB, Quacking: %.1f dB above "
             "baseline",
             tooting_power - baseline_power, quacking_power - baseline_power);
  }

  return tooting_detected || quacking_detected;
}

HiveState BeeAudioComponent::classify_hive_state_() {
  float baseline_power = this->calculate_band_power_(BAND_BASELINE);
  float worker_power = this->calculate_band_power_(BAND_WORKER);
  float tooting_power = this->calculate_band_power_(BAND_TOOTING);
  float quacking_power = this->calculate_band_power_(BAND_QUACKING);
  float queenless_mid_power = this->calculate_band_power_(BAND_QUEENLESS_MID);
  float queenless_high_power = this->calculate_band_power_(BAND_QUEENLESS_HIGH);
  float centroid = this->calculate_spectral_centroid_();

  ESP_LOGD(TAG,
           "Classification - Baseline: %.1f, Worker: %.1f, Tooting: %.1f, "
           "Quacking: %.1f",
           baseline_power, worker_power, tooting_power, quacking_power);
  ESP_LOGD(TAG,
           "Classification - QueenlessMid: %.1f, QueenlessHigh: %.1f, "
           "Centroid: %.1f Hz",
           queenless_mid_power, queenless_high_power, centroid);

  // Check for queenless condition (elevated mid and high frequency bands)
  if ((queenless_mid_power - baseline_power) > QUEENLESS_THRESHOLD_DB &&
      (queenless_high_power - baseline_power) > QUEENLESS_THRESHOLD_DB) {

    return HiveState::QUEENLESS;
  }

  // Check for queen piping activity
  if ((tooting_power - baseline_power) > QUEEN_PIPING_THRESHOLD_DB ||
      (quacking_power - baseline_power) > QUEEN_PIPING_THRESHOLD_DB) {

    return HiveState::QUEEN_ACTIVITY;
  }

  // Check for pre-swarm (elevated centroid and activity)
  if (centroid > PRE_SWARM_CENTROID_HZ && worker_power > ACTIVE_THRESHOLD_DB) {

    return HiveState::PRE_SWARM;
  }

  // Check for active state
  if (worker_power > ACTIVE_THRESHOLD_DB) {

    return HiveState::ACTIVE;
  }

  // Check for normal state
  if (baseline_power > NORMAL_THRESHOLD_DB) {

    return HiveState::NORMAL;
  }

  return HiveState::QUIET;
}

const char *BeeAudioComponent::hive_state_to_string_(HiveState state) {
  switch (state) {
  case HiveState::QUIET:

    return "quiet";
  case HiveState::NORMAL:

    return "normal";
  case HiveState::ACTIVE:

    return "active";
  case HiveState::QUEEN_ACTIVITY:

    return "queen_activity";
  case HiveState::QUEENLESS:

    return "queenless";
  case HiveState::PRE_SWARM:

    return "pre_swarm";
  default:

    return "unknown";
  }
}

} // namespace bee_audio
} // namespace esphome
