#include "bee_audio.h"
#include "esphome/core/log.h"
#include <cinttypes>
#include <cmath>
#include <cstring>

namespace esphome {
namespace bee_audio {

static const char *const TAG = "bee_audio";

static const char *const BAND_NAMES[BAND_COUNT] = {
    "low_freq", "baseline",      "worker",        "quacking",
    "tooting",  "queenless_mid", "queenless_high"};

// Floor for log10 of power values, keeps silence finite (-200 dB)
static const float POWER_FLOOR = 1e-20f;

BeeAudioComponent::~BeeAudioComponent() {
  this->deinit_i2s_();
  this->free_buffers_();
}

void BeeAudioComponent::setup() {
  ESP_LOGD(TAG, "Setting up Bee Audio...");

  if (this->fft_size_ == 0 || (this->fft_size_ & (this->fft_size_ - 1)) != 0) {
    ESP_LOGE(TAG, "FFT size must be power of 2, got %zu", this->fft_size_);
    this->mark_failed();

    return;
  }

  this->freq_resolution_ = static_cast<float>(this->sample_rate_) /
                           static_cast<float>(this->fft_size_);
  ESP_LOGD(TAG, "Frequency resolution: %.2f Hz/bin", this->freq_resolution_);

  if (!this->allocate_buffers_()) {
    this->mark_failed();

    return;
  }

  dsps_wind_hann_f32(this->window_, this->fft_size_);
  this->window_power_sum_ = 0.0f;
  for (size_t i = 0; i < this->fft_size_; i++) {
    this->window_power_sum_ += this->window_[i] * this->window_[i];
  }

  esp_err_t ret = dsps_fft2r_init_fc32(nullptr, this->fft_size_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "FFT init failed: %s", esp_err_to_name(ret));
    this->free_buffers_();
    this->mark_failed();

    return;
  }
  this->fft_initialised_ = true;

  if (!this->init_i2s_()) {
    this->free_buffers_();
    this->mark_failed();

    return;
  }

  ESP_LOGCONFIG(TAG, "Bee Audio initialised successfully");
}

void BeeAudioComponent::update() {
  ESP_LOGD(TAG, "Starting audio capture and analysis...");

  if (!this->capture_and_analyse_()) {
    ESP_LOGW(TAG, "Audio capture failed");
    this->status_set_warning();

    return;
  }
  this->status_clear_warning();

  for (uint8_t b = 0; b < BAND_COUNT; b++) {
    this->band_power_[b] = this->calculate_band_power_(this->bands_[b]);
    if (this->band_sensors_[b] != nullptr) {
      this->band_sensors_[b]->publish_state(this->band_power_[b]);
    }
  }

  float centroid = this->calculate_spectral_centroid_();

  if (this->dominant_frequency_sensor_ != nullptr) {
    this->dominant_frequency_sensor_->publish_state(
        this->calculate_dominant_frequency_());
  }

  if (this->sound_level_rms_sensor_ != nullptr) {
    this->sound_level_rms_sensor_->publish_state(this->rms_db_);
  }

  if (this->spectral_centroid_sensor_ != nullptr) {
    this->spectral_centroid_sensor_->publish_state(centroid);
  }

  bool piping = this->detect_queen_piping_();
  if (piping) {
    ESP_LOGD(TAG, "Queen piping detected! Tooting: %.1f dB, Quacking: %.1f dB "
                  "above baseline",
             this->band_power_[BAND_TOOTING] - this->band_power_[BAND_BASELINE],
             this->band_power_[BAND_QUACKING] - this->band_power_[BAND_BASELINE]);
  }
  if (this->queen_piping_sensor_ != nullptr) {
    this->queen_piping_sensor_->publish_state(piping);
  }

  if (this->hive_state_sensor_ != nullptr) {
    HiveState state = this->classify_hive_state_(centroid);
    this->hive_state_sensor_->publish_state(hive_state_to_string_(state));
  }

  ESP_LOGD(TAG, "Audio analysis complete");
}

void BeeAudioComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Bee Audio:");
  ESP_LOGCONFIG(TAG, "  I2S LRCLK Pin: GPIO%d", this->i2s_lrclk_pin_);
  ESP_LOGCONFIG(TAG, "  I2S BCLK Pin: GPIO%d", this->i2s_bclk_pin_);
  ESP_LOGCONFIG(TAG, "  I2S DIN Pin: GPIO%d", this->i2s_din_pin_);
  ESP_LOGCONFIG(TAG, "  Sample Rate: %" PRIu32 " Hz", this->sample_rate_);
  ESP_LOGCONFIG(TAG, "  FFT Size: %zu", this->fft_size_);
  ESP_LOGCONFIG(TAG, "  Frames Averaged: %u", this->frames_);
  ESP_LOGCONFIG(TAG, "  Frequency Resolution: %.2f Hz/bin",
                this->freq_resolution_);
  for (uint8_t b = 0; b < BAND_COUNT; b++) {
    ESP_LOGCONFIG(TAG, "  Band %s: %.0f-%.0f Hz", BAND_NAMES[b],
                  this->bands_[b].low_hz, this->bands_[b].high_hz);
  }
  ESP_LOGCONFIG(TAG, "  Queenless Threshold: %.1f dB",
                this->queenless_threshold_db_);
  ESP_LOGCONFIG(TAG, "  Queen Piping Threshold: %.1f dB",
                this->queen_piping_threshold_db_);
  ESP_LOGCONFIG(TAG, "  Active Threshold: %.1f dB", this->active_threshold_db_);
  ESP_LOGCONFIG(TAG, "  Normal Threshold: %.1f dB", this->normal_threshold_db_);
  ESP_LOGCONFIG(TAG, "  Pre-swarm Centroid: %.0f Hz",
                this->pre_swarm_centroid_hz_);
}

bool BeeAudioComponent::init_i2s_() {
  ESP_LOGD(TAG, "Initialising I2S...");

  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num = 8;
  chan_cfg.dma_frame_num = 512;

  esp_err_t ret = i2s_new_channel(&chan_cfg, nullptr, &this->rx_chan_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(ret));

    return false;
  }

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
    this->rx_chan_ = nullptr;

    return false;
  }

  // Channel is enabled only around reads so DMA is idle between updates
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

  // Float buffers need byte-addressable memory (IRAM cannot be accessed by
  // float instructions)
  const uint32_t float_caps = MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL;

  const size_t raw_bytes = this->fft_size_ * sizeof(int32_t);
  const size_t fft_bytes = this->fft_size_ * 2 * sizeof(float);
  const size_t window_bytes = this->fft_size_ * sizeof(float);
  const size_t psd_bytes = (this->fft_size_ / 2) * sizeof(float);

  this->raw_samples_ = static_cast<int32_t *>(heap_caps_aligned_alloc(
      alignment, raw_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  this->fft_data_ = static_cast<float *>(
      heap_caps_aligned_alloc(alignment, fft_bytes, float_caps));
  this->window_ = static_cast<float *>(
      heap_caps_aligned_alloc(alignment, window_bytes, float_caps));
  this->psd_ = static_cast<float *>(
      heap_caps_aligned_alloc(alignment, psd_bytes, float_caps));

  if (this->raw_samples_ == nullptr || this->fft_data_ == nullptr ||
      this->window_ == nullptr || this->psd_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate audio buffers (%zu bytes)",
             raw_bytes + fft_bytes + window_bytes + psd_bytes);
    this->free_buffers_();

    return false;
  }

  ESP_LOGD(TAG, "Allocated %.1f KB for audio buffers",
           (raw_bytes + fft_bytes + window_bytes + psd_bytes) / 1024.0f);

  return true;
}

void BeeAudioComponent::free_buffers_() {
  heap_caps_free(this->raw_samples_);
  heap_caps_free(this->fft_data_);
  heap_caps_free(this->window_);
  heap_caps_free(this->psd_);
  this->raw_samples_ = nullptr;
  this->fft_data_ = nullptr;
  this->window_ = nullptr;
  this->psd_ = nullptr;

  if (this->fft_initialised_) {
    dsps_fft2r_deinit_fc32();
    this->fft_initialised_ = false;
  }
}

// Captures frames_ consecutive frames and accumulates a Welch-averaged
// one-sided power spectral density in psd_ (units: FS^2/Hz), plus the RMS
// level of all samples in rms_db_ (dBFS).
bool BeeAudioComponent::capture_and_analyse_() {
  esp_err_t ret = i2s_channel_enable(this->rx_chan_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable I2S channel: %s", esp_err_to_name(ret));

    return false;
  }

  const size_t num_bins = this->fft_size_ / 2;
  const size_t bytes_to_read = this->fft_size_ * sizeof(int32_t);
  const uint32_t frame_ms =
      (static_cast<uint32_t>(this->fft_size_) * 1000) / this->sample_rate_;
  const uint32_t read_timeout_ms = 2 * frame_ms + 100;
  size_t bytes_read = 0;

  // Discard the first samples: the INMP441 needs time to settle after clocks
  // start and the first DMA buffers can contain stale data
  int32_t dummy_buffer[256];
  for (int i = 0; i < 3; i++) {
    i2s_channel_read(this->rx_chan_, dummy_buffer, sizeof(dummy_buffer),
                     &bytes_read, 100);
  }

  std::memset(this->psd_, 0, num_bins * sizeof(float));
  double sum_squared = 0.0;

  // INMP441 outputs 24-bit data left-justified in a 32-bit word
  const float scale = 1.0f / 8388608.0f; // 2^23

  for (uint8_t frame = 0; frame < this->frames_; frame++) {
    ret = i2s_channel_read(this->rx_chan_, this->raw_samples_, bytes_to_read,
                           &bytes_read, read_timeout_ms);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "I2S read failed: %s", esp_err_to_name(ret));
      i2s_channel_disable(this->rx_chan_);

      return false;
    }
    if (bytes_read != bytes_to_read) {
      ESP_LOGW(TAG, "I2S read incomplete: %zu/%zu bytes", bytes_read,
               bytes_to_read);
      i2s_channel_disable(this->rx_chan_);

      return false;
    }

    // Normalise to [-1, 1], apply Hann window, interleave as complex input
    for (size_t i = 0; i < this->fft_size_; i++) {
      float sample = static_cast<float>(this->raw_samples_[i] >> 8) * scale;
      sum_squared += static_cast<double>(sample) * sample;
      this->fft_data_[i * 2] = sample * this->window_[i];
      this->fft_data_[i * 2 + 1] = 0.0f;
    }

    dsps_fft2r_fc32(this->fft_data_, this->fft_size_);
    dsps_bit_rev_fc32(this->fft_data_, this->fft_size_);

    for (size_t k = 0; k < num_bins; k++) {
      float real = this->fft_data_[k * 2];
      float imag = this->fft_data_[k * 2 + 1];
      this->psd_[k] += real * real + imag * imag;
    }
  }

  i2s_channel_disable(this->rx_chan_);

  // One-sided PSD: 2|X|^2 / (fs * sum(w^2)), averaged over frames. This is
  // independent of fft_size, so thresholds survive resolution changes.
  const float norm = 2.0f / (static_cast<float>(this->sample_rate_) *
                             this->window_power_sum_ *
                             static_cast<float>(this->frames_));
  for (size_t k = 0; k < num_bins; k++) {
    this->psd_[k] *= norm;
  }

  const double total_samples =
      static_cast<double>(this->fft_size_) * this->frames_;
  this->rms_db_ = 10.0f * log10f(static_cast<float>(sum_squared / total_samples) +
                                 POWER_FLOOR);

  return true;
}

int BeeAudioComponent::hz_to_bin_(float hz) const {
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

// Mean PSD across the band in dB (re FS^2/Hz). Using density rather than the
// band sum keeps bands of different widths directly comparable.
float BeeAudioComponent::calculate_band_power_(const FrequencyBand &band) const {
  int start_bin = this->hz_to_bin_(band.low_hz);
  int end_bin = this->hz_to_bin_(band.high_hz);

  float sum = 0.0f;
  for (int i = start_bin; i <= end_bin; i++) {
    sum += this->psd_[i];
  }
  float mean = sum / static_cast<float>(end_bin - start_bin + 1);

  return 10.0f * log10f(mean + POWER_FLOOR);
}

float BeeAudioComponent::calculate_dominant_frequency_() const {
  int start_bin = this->hz_to_bin_(60.0f);
  int end_bin = this->hz_to_bin_(600.0f);

  float max_power = 0.0f;
  int max_bin = start_bin;

  for (int i = start_bin; i <= end_bin; i++) {
    if (this->psd_[i] > max_power) {
      max_power = this->psd_[i];
      max_bin = i;
    }
  }

  // Parabolic interpolation for sub-bin frequency resolution
  if (max_bin > start_bin && max_bin < end_bin) {
    float y0 = this->psd_[max_bin - 1];
    float y1 = this->psd_[max_bin];
    float y2 = this->psd_[max_bin + 1];

    float delta = 0.5f * (y0 - y2) / (y0 - 2.0f * y1 + y2 + POWER_FLOOR);

    return (static_cast<float>(max_bin) + delta) * this->freq_resolution_;
  }

  return static_cast<float>(max_bin) * this->freq_resolution_;
}

// Magnitude-weighted mean frequency over the bee-relevant 60-1200 Hz range
float BeeAudioComponent::calculate_spectral_centroid_() const {
  int start_bin = this->hz_to_bin_(60.0f);
  int end_bin = this->hz_to_bin_(1200.0f);

  float weighted_sum = 0.0f;
  float magnitude_sum = 0.0f;

  for (int i = start_bin; i <= end_bin; i++) {
    float magnitude = sqrtf(this->psd_[i]);
    weighted_sum += static_cast<float>(i) * this->freq_resolution_ * magnitude;
    magnitude_sum += magnitude;
  }

  if (magnitude_sum < 1e-10f) {

    return 0.0f;
  }

  return weighted_sum / magnitude_sum;
}

bool BeeAudioComponent::detect_queen_piping_() const {
  float baseline = this->band_power_[BAND_BASELINE];
  float tooting = this->band_power_[BAND_TOOTING] - baseline;
  float quacking = this->band_power_[BAND_QUACKING] - baseline;

  return tooting > this->queen_piping_threshold_db_ ||
         quacking > this->queen_piping_threshold_db_;
}

HiveState BeeAudioComponent::classify_hive_state_(float centroid) const {
  float baseline = this->band_power_[BAND_BASELINE];
  float worker = this->band_power_[BAND_WORKER];

  ESP_LOGD(TAG,
           "Classification - Baseline: %.1f, Worker: %.1f, Tooting: %.1f, "
           "Quacking: %.1f",
           baseline, worker, this->band_power_[BAND_TOOTING],
           this->band_power_[BAND_QUACKING]);
  ESP_LOGD(TAG,
           "Classification - QueenlessMid: %.1f, QueenlessHigh: %.1f, "
           "Centroid: %.1f Hz",
           this->band_power_[BAND_QUEENLESS_MID],
           this->band_power_[BAND_QUEENLESS_HIGH], centroid);

  if ((this->band_power_[BAND_QUEENLESS_MID] - baseline) >
          this->queenless_threshold_db_ &&
      (this->band_power_[BAND_QUEENLESS_HIGH] - baseline) >
          this->queenless_threshold_db_) {

    return HiveState::QUEENLESS;
  }

  if (this->detect_queen_piping_()) {

    return HiveState::QUEEN_ACTIVITY;
  }

  if (centroid > this->pre_swarm_centroid_hz_ &&
      worker > this->active_threshold_db_) {

    return HiveState::PRE_SWARM;
  }

  if (worker > this->active_threshold_db_) {

    return HiveState::ACTIVE;
  }

  if (baseline > this->normal_threshold_db_) {

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
