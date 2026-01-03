#pragma once

#include <atomic>
#include <cstdint>
#include <utility>

class WheelFrequencyReader
{
public:
  // gpio_left=23, gpio_right=27 par défaut (vos pins)
  explicit WheelFrequencyReader(int gpio_left = 23, int gpio_right = 27);
  ~WheelFrequencyReader();

  WheelFrequencyReader(const WheelFrequencyReader&) = delete;
  WheelFrequencyReader& operator=(const WheelFrequencyReader&) = delete;

  // Initialise pigpio + callbacks
  bool init();

  // Retourne {freq_left_hz, freq_right_hz}
  std::pair<double, double> getFrequenciesHz() const;

  // Optionnel : renvoie {rad/s gauche, rad/s droite} si vous connaissez pulses_per_rev
  std::pair<double, double> getAngularVelRadS(double pulses_per_rev) const;

  // Optionnel : si vous voulez considérer 0 Hz après un silence de X ms
  void setTimeoutMs(uint32_t timeout_ms);

private:
  static void alertTrampoline_(int gpio, int level, uint32_t tick);
  void handleEdge_(int gpio, int level, uint32_t tick);

  // Attention: pigpio utilise des callbacks C => on passe par instance unique (simple et robuste).
  static WheelFrequencyReader* instance_;

  int gpio_left_;
  int gpio_right_;

  std::atomic<uint32_t> last_tick_left_{0};
  std::atomic<uint32_t> last_tick_right_{0};

  std::atomic<uint32_t> last_seen_tick_left_{0};
  std::atomic<uint32_t> last_seen_tick_right_{0};

  std::atomic<double> freq_left_hz_{0.0};
  std::atomic<double> freq_right_hz_{0.0};

  // Filtrage simple (EMA) optionnel
  std::atomic<bool> use_ema_{true};
  std::atomic<double> alpha_{0.2}; // 0.2 = assez réactif

  // Timeout (en ms) : si aucun front depuis timeout_ms => freq=0
  std::atomic<uint32_t> timeout_ms_{200};

  bool callbacks_attached_ = false;
};
