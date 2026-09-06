#include "bee_audio.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include <algorithm>
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

// INMP441 outputs 24-bit data left-justified in a 32-bit word
static const float SAMPLE_SCALE = 1.0f / 8388608.0f; // 2^23

// Modulation spectrum STFT: 100 ms window, 12.5 ms hop (Abdollahi et al.)
static const uint32_t STFT_WINDOWS_PER_SECOND = 10;
static const uint32_t STFT_HOPS_PER_SECOND = 80;
// Second-stage FFT over the band envelope: 256 hops (3.2 s), 50% overlap
static const size_t MOD_SEGMENT = 256;
static const size_t MOD_HOP = MOD_SEGMENT / 2;
// Modulation rates below this are treated as drift rather than buzz dynamics
static const float MOD_MIN_HZ = 1.0f;

static size_t next_power_of_two(size_t n) {
  size_t p = 1;
  while (p < n) {
    p <<= 1;
  }

  return p;
}

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

  size_t fft_table_size = this->fft_size_;
  if (this->modulation_enabled_()) {
    this->stft_window_ = this->sample_rate_ / STFT_WINDOWS_PER_SECOND;
    this->stft_hop_ = this->sample_rate_ / STFT_HOPS_PER_SECOND;
    this->stft_fft_size_ = next_power_of_two(this->stft_window_);
    this->envelope_len_ =
        (this->modulation_duration_ms_ * STFT_HOPS_PER_SECOND) / 1000;
    if (this->envelope_len_ < MOD_SEGMENT) {
      ESP_LOGE(TAG, "Modulation duration too short: need at least %u ms",
               static_cast<unsigned>(MOD_SEGMENT * 1000 / STFT_HOPS_PER_SECOND));
      this->free_buffers_();
      this->mark_failed();

      return;
    }
    if (!this->allocate_modulation_buffers_()) {
      this->free_buffers_();
      this->mark_failed();

      return;
    }
    dsps_wind_hann_f32(this->stft_window_fn_, this->stft_window_);
    dsps_wind_hann_f32(this->mod_window_fn_, MOD_SEGMENT);
    fft_table_size = std::max(fft_table_size, this->stft_fft_size_);
  }

  // The twiddle table is bit-reversed, so one table sized for the largest
  // FFT also serves every smaller power-of-two length
  esp_err_t ret = dsps_fft2r_init_fc32(nullptr, fft_table_size);
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

  if (this->modulation_enabled_()) {
    if (this->capture_modulation_()) {
      this->analyse_envelope_();
      ESP_LOGD(TAG, "Modulation index: %.1f%%, peak modulation: %.1f Hz",
               this->modulation_index_, this->modulation_frequency_);
      if (this->modulation_index_sensor_ != nullptr) {
        this->modulation_index_sensor_->publish_state(this->modulation_index_);
      }
      if (this->modulation_frequency_sensor_ != nullptr) {
        this->modulation_frequency_sensor_->publish_state(
            this->modulation_frequency_);
      }
    } else {
      ESP_LOGW(TAG, "Modulation capture failed");
      this->status_set_warning();
    }
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
  if (this->modulation_enabled_()) {
    ESP_LOGCONFIG(TAG, "  Modulation Duration: %" PRIu32 " ms (%zu hops)",
                  this->modulation_duration_ms_, this->envelope_len_);
    ESP_LOGCONFIG(TAG, "  Modulation Band: %.0f-%.0f Hz",
                  this->modulation_band_.low_hz, this->modulation_band_.high_hz);
    ESP_LOGCONFIG(TAG, "  Modulation Rate: %.1f-%.1f Hz",
                  this->modulation_rate_.low_hz, this->modulation_rate_.high_hz);
  }
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

bool BeeAudioComponent::allocate_modulation_buffers_() {
  const size_t alignment = 16;
  const uint32_t float_caps = MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL;

  const size_t hop_bytes = this->stft_hop_ * sizeof(int32_t);
  const size_t input_bytes = this->stft_window_ * sizeof(float);
  const size_t stft_bytes = this->stft_fft_size_ * 2 * sizeof(float);
  const size_t envelope_bytes = this->envelope_len_ * sizeof(float);
  const size_t mod_window_bytes = MOD_SEGMENT * sizeof(float);
  const size_t mod_psd_bytes = (MOD_SEGMENT / 2) * sizeof(float);
  const size_t total = hop_bytes + 2 * input_bytes + stft_bytes +
                       envelope_bytes + mod_window_bytes + mod_psd_bytes;

  this->hop_samples_ = static_cast<int32_t *>(heap_caps_aligned_alloc(
      alignment, hop_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  this->stft_input_ = static_cast<float *>(
      heap_caps_aligned_alloc(alignment, input_bytes, float_caps));
  this->stft_window_fn_ = static_cast<float *>(
      heap_caps_aligned_alloc(alignment, input_bytes, float_caps));
  this->stft_data_ = static_cast<float *>(
      heap_caps_aligned_alloc(alignment, stft_bytes, float_caps));
  this->envelope_ = static_cast<float *>(
      heap_caps_aligned_alloc(alignment, envelope_bytes, float_caps));
  this->mod_window_fn_ = static_cast<float *>(
      heap_caps_aligned_alloc(alignment, mod_window_bytes, float_caps));
  this->mod_psd_ = static_cast<float *>(
      heap_caps_aligned_alloc(alignment, mod_psd_bytes, float_caps));

  if (this->hop_samples_ == nullptr || this->stft_input_ == nullptr ||
      this->stft_window_fn_ == nullptr || this->stft_data_ == nullptr ||
      this->envelope_ == nullptr || this->mod_window_fn_ == nullptr ||
      this->mod_psd_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate modulation buffers (%zu bytes)", total);

    return false;
  }

  ESP_LOGD(TAG, "Allocated %.1f KB for modulation buffers", total / 1024.0f);

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

  heap_caps_free(this->hop_samples_);
  heap_caps_free(this->stft_input_);
  heap_caps_free(this->stft_window_fn_);
  heap_caps_free(this->stft_data_);
  heap_caps_free(this->envelope_);
  heap_caps_free(this->mod_window_fn_);
  heap_caps_free(this->mod_psd_);
  this->hop_samples_ = nullptr;
  this->stft_input_ = nullptr;
  this->stft_window_fn_ = nullptr;
  this->stft_data_ = nullptr;
  this->envelope_ = nullptr;
  this->mod_window_fn_ = nullptr;
  this->mod_psd_ = nullptr;

  if (this->fft_initialised_) {
    dsps_fft2r_deinit_fc32();
    this->fft_initialised_ = false;
  }
}

// Enables the channel and discards the first samples: the INMP441 needs time
// to settle after clocks start and the first DMA buffers can contain stale data
bool BeeAudioComponent::discard_startup_samples_() {
  esp_err_t ret = i2s_channel_enable(this->rx_chan_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable I2S channel: %s", esp_err_to_name(ret));

    return false;
  }

  int32_t dummy_buffer[256];
  size_t bytes_read = 0;
  for (int i = 0; i < 3; i++) {
    i2s_channel_read(this->rx_chan_, dummy_buffer, sizeof(dummy_buffer),
                     &bytes_read, 100);
  }

  return true;
}

// Captures frames_ consecutive frames and accumulates a Welch-averaged
// one-sided power spectral density in psd_ (units: FS^2/Hz), plus the RMS
// level of all samples in rms_db_ (dBFS).
bool BeeAudioComponent::capture_and_analyse_() {
  if (!this->discard_startup_samples_()) {

    return false;
  }

  const size_t num_bins = this->fft_size_ / 2;
  const size_t bytes_to_read = this->fft_size_ * sizeof(int32_t);
  const uint32_t frame_ms =
      (static_cast<uint32_t>(this->fft_size_) * 1000) / this->sample_rate_;
  const uint32_t read_timeout_ms = 2 * frame_ms + 100;
  size_t bytes_read = 0;
  esp_err_t ret;

  std::memset(this->psd_, 0, num_bins * sizeof(float));
  double sum_squared = 0.0;

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
      float sample =
          static_cast<float>(this->raw_samples_[i] >> 8) * SAMPLE_SCALE;
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

// Records envelope_len_ hops of the modulation band's STFT magnitude into
// envelope_. The window is primed with a full 100 ms before recording starts
// so every envelope point comes from a fully populated window.
bool BeeAudioComponent::capture_modulation_() {
  if (!this->discard_startup_samples_()) {

    return false;
  }

  const size_t hop_bytes = this->stft_hop_ * sizeof(int32_t);
  const size_t keep = this->stft_window_ - this->stft_hop_;
  const uint32_t read_timeout_ms =
      2 * (1000 * this->stft_hop_ / this->sample_rate_) + 100;
  const float stft_resolution = static_cast<float>(this->sample_rate_) /
                                static_cast<float>(this->stft_fft_size_);
  const int max_bin = static_cast<int>(this->stft_fft_size_ / 2 - 1);
  const int start_bin = std::min(
      static_cast<int>(this->modulation_band_.low_hz / stft_resolution),
      max_bin);
  const int end_bin = std::min(
      static_cast<int>(this->modulation_band_.high_hz / stft_resolution),
      max_bin);
  const size_t prime_hops = this->stft_window_ / this->stft_hop_;
  size_t bytes_read = 0;

  std::memset(this->stft_input_, 0, this->stft_window_ * sizeof(float));

  for (size_t hop = 0; hop < prime_hops + this->envelope_len_; hop++) {
    esp_err_t ret = i2s_channel_read(this->rx_chan_, this->hop_samples_,
                                     hop_bytes, &bytes_read, read_timeout_ms);
    if (ret != ESP_OK || bytes_read != hop_bytes) {
      ESP_LOGE(TAG, "I2S hop read failed: %s (%zu/%zu bytes)",
               esp_err_to_name(ret), bytes_read, hop_bytes);
      i2s_channel_disable(this->rx_chan_);

      return false;
    }

    std::memmove(this->stft_input_, this->stft_input_ + this->stft_hop_,
                 keep * sizeof(float));
    for (size_t i = 0; i < this->stft_hop_; i++) {
      this->stft_input_[keep + i] =
          static_cast<float>(this->hop_samples_[i] >> 8) * SAMPLE_SCALE;
    }

    // Blocking here for the whole capture; keep the task watchdog happy
    App.feed_wdt();

    if (hop < prime_hops) {
      continue;
    }

    for (size_t i = 0; i < this->stft_window_; i++) {
      this->stft_data_[i * 2] = this->stft_input_[i] * this->stft_window_fn_[i];
      this->stft_data_[i * 2 + 1] = 0.0f;
    }
    std::memset(this->stft_data_ + this->stft_window_ * 2, 0,
                (this->stft_fft_size_ - this->stft_window_) * 2 * sizeof(float));

    dsps_fft2r_fc32(this->stft_data_, this->stft_fft_size_);
    dsps_bit_rev_fc32(this->stft_data_, this->stft_fft_size_);

    float magnitude = 0.0f;
    for (int k = start_bin; k <= end_bin; k++) {
      float real = this->stft_data_[k * 2];
      float imag = this->stft_data_[k * 2 + 1];
      magnitude += sqrtf(real * real + imag * imag);
    }
    this->envelope_[hop - prime_hops] = magnitude;
  }

  i2s_channel_disable(this->rx_chan_);

  return true;
}

// Welch-averaged power spectrum of the mean-removed band envelope. The
// modulation index is the share of envelope power (above MOD_MIN_HZ) that
// falls in the configured modulation rate band, so it is independent of
// microphone gain and colony loudness.
void BeeAudioComponent::analyse_envelope_() {
  const size_t num_bins = MOD_SEGMENT / 2;
  const float envelope_rate = static_cast<float>(this->sample_rate_) /
                              static_cast<float>(this->stft_hop_);
  const float resolution = envelope_rate / static_cast<float>(MOD_SEGMENT);

  std::memset(this->mod_psd_, 0, num_bins * sizeof(float));

  for (size_t start = 0; start + MOD_SEGMENT <= this->envelope_len_;
       start += MOD_HOP) {
    float mean = 0.0f;
    for (size_t i = 0; i < MOD_SEGMENT; i++) {
      mean += this->envelope_[start + i];
    }
    mean /= static_cast<float>(MOD_SEGMENT);

    for (size_t i = 0; i < MOD_SEGMENT; i++) {
      this->stft_data_[i * 2] =
          (this->envelope_[start + i] - mean) * this->mod_window_fn_[i];
      this->stft_data_[i * 2 + 1] = 0.0f;
    }

    dsps_fft2r_fc32(this->stft_data_, MOD_SEGMENT);
    dsps_bit_rev_fc32(this->stft_data_, MOD_SEGMENT);

    for (size_t k = 0; k < num_bins; k++) {
      float real = this->stft_data_[k * 2];
      float imag = this->stft_data_[k * 2 + 1];
      this->mod_psd_[k] += real * real + imag * imag;
    }
  }

  const int max_bin = static_cast<int>(num_bins - 1);
  const int total_start =
      std::min(static_cast<int>(ceilf(MOD_MIN_HZ / resolution)), max_bin);
  const int band_start = std::max(
      std::min(static_cast<int>(this->modulation_rate_.low_hz / resolution),
               max_bin),
      total_start);
  const int band_end = std::min(
      static_cast<int>(this->modulation_rate_.high_hz / resolution), max_bin);

  float total = 0.0f;
  for (int k = total_start; k <= max_bin; k++) {
    total += this->mod_psd_[k];
  }
  float band = 0.0f;
  for (int k = band_start; k <= band_end; k++) {
    band += this->mod_psd_[k];
  }

  this->modulation_index_ = total > 0.0f ? 100.0f * band / total : 0.0f;
  this->modulation_frequency_ =
      this->peak_frequency_(this->mod_psd_, total_start, max_bin, resolution);
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

// Frequency of the strongest bin in [start_bin, end_bin], refined with
// parabolic interpolation for sub-bin resolution
float BeeAudioComponent::peak_frequency_(const float *psd, int start_bin,
                                         int end_bin, float resolution) const {
  float max_power = 0.0f;
  int max_bin = start_bin;

  for (int i = start_bin; i <= end_bin; i++) {
    if (psd[i] > max_power) {
      max_power = psd[i];
      max_bin = i;
    }
  }

  if (max_bin > start_bin && max_bin < end_bin) {
    float y0 = psd[max_bin - 1];
    float y1 = psd[max_bin];
    float y2 = psd[max_bin + 1];

    float delta = 0.5f * (y0 - y2) / (y0 - 2.0f * y1 + y2 + POWER_FLOOR);

    return (static_cast<float>(max_bin) + delta) * resolution;
  }

  return static_cast<float>(max_bin) * resolution;
}

float BeeAudioComponent::calculate_dominant_frequency_() const {
  return this->peak_frequency_(this->psd_, this->hz_to_bin_(60.0f),
                               this->hz_to_bin_(600.0f), this->freq_resolution_);
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
