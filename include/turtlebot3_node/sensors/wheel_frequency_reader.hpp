#pragma once

#include <atomic>
#include <cstdint>
#include <utility>

class WheelFrequencyReader
{
public:
  // pi = handle retourné par pigpio_start() dans TurtleBot3
  explicit WheelFrequencyReader(int pi, int gpio_left = 23, int gpio_right = 27);
  ~WheelFrequencyReader();

  WheelFrequencyReader(const WheelFrequencyReader&) = delete;
  WheelFrequencyReader& operator=(const WheelFrequencyReader&) = delete;

  bool init();

  std::pair<double, double> getFrequenciesHz() const;
  std::pair<double, double> getAngularVelRadS(double pulses_per_rev) const;

  void setTimeoutMs(uint32_t timeout_ms);

private:
  static void alertCb_(int pi, unsigned user_gpio, unsigned level, uint32_t tick, void* user);
  void handleEdge_(unsigned gpio, unsigned level, uint32_t tick);

  int pi_{-1};
  int gpio_left_;
  int gpio_right_;

  std::atomic<uint32_t> last_tick_left_{0};
  std::atomic<uint32_t> last_tick_right_{0};

  std::atomic<uint32_t> last_seen_tick_left_{0};
  std::atomic<uint32_t> last_seen_tick_right_{0};

  std::atomic<double> freq_left_hz_{0.0};
  std::atomic<double> freq_right_hz_{0.0};

  std::atomic<bool> use_ema_{true};
  std::atomic<double> alpha_{0.2};

  std::atomic<uint32_t> timeout_ms_{200};

  int cb_left_id_{-1};
  int cb_right_id_{-1};
  bool callbacks_attached_{false};
};
