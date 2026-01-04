#include "turtlebot3_node/sensors/wheel_frequency_reader.hpp"

#include <pigpiod_if2.h>

#include <cstdio>
#include <cmath>

WheelFrequencyReader::WheelFrequencyReader(int pi, int gpio_left, int gpio_right)
: pi_(pi), gpio_left_(gpio_left), gpio_right_(gpio_right)
{
}

WheelFrequencyReader::~WheelFrequencyReader()
{
  if (callbacks_attached_ && pi_ >= 0) {
    if (cb_left_id_ >= 0)  callback_cancel(cb_left_id_);
    if (cb_right_id_ >= 0) callback_cancel(cb_right_id_);
    callbacks_attached_ = false;
  }
}

bool WheelFrequencyReader::init()
{
  if (pi_ < 0) {
    std::fprintf(stderr, "WheelFrequencyReader::init(): invalid pi handle (pi=%d)\n", pi_);
    return false;
  }

  if (set_mode(pi_, gpio_left_, PI_INPUT) != 0 ||
      set_mode(pi_, gpio_right_, PI_INPUT) != 0) {
    std::fprintf(stderr, "set_mode failed\n");
    return false;
  }

  // Adaptez si votre capteur a besoin de PULL_UP au lieu de PULL_DOWN
  set_pull_up_down(pi_, gpio_left_, PI_PUD_DOWN);
  set_pull_up_down(pi_, gpio_right_, PI_PUD_DOWN);

  // Callback EITHER_EDGE, on filtrera sur front montant
  cb_left_id_  = callback_ex(pi_, gpio_left_,  EITHER_EDGE, &WheelFrequencyReader::alertCb_, this);
  cb_right_id_ = callback_ex(pi_, gpio_right_, EITHER_EDGE, &WheelFrequencyReader::alertCb_, this);

  if (cb_left_id_ < 0 || cb_right_id_ < 0) {
    std::fprintf(stderr, "callback_ex failed: left=%d right=%d\n", cb_left_id_, cb_right_id_);
    if (cb_left_id_ >= 0) callback_cancel(cb_left_id_);
    if (cb_right_id_ >= 0) callback_cancel(cb_right_id_);
    cb_left_id_ = cb_right_id_ = -1;
    return false;
  }

  callbacks_attached_ = true;
  return true;
}

void WheelFrequencyReader::setTimeoutMs(uint32_t timeout_ms)
{
  timeout_ms_.store(timeout_ms);
}

std::pair<double, double> WheelFrequencyReader::getFrequenciesHz() const
{
  if (pi_ < 0) return {0.0, 0.0};

  // Tick courant en microsecondes (wrap uint32 OK)
  const uint32_t now = get_current_tick(pi_);
  const uint32_t timeout_us = timeout_ms_.load() * 1000u;

  const auto left_last  = last_seen_tick_left_.load();
  const auto right_last = last_seen_tick_right_.load();

  double fl = freq_left_hz_.load();
  double fr = freq_right_hz_.load();

  if (left_last != 0) {
    const uint32_t dt = now - left_last;
    if (dt > timeout_us) fl = 0.0;
  } else {
    fl = 0.0;
  }

  if (right_last != 0) {
    const uint32_t dt = now - right_last;
    if (dt > timeout_us) fr = 0.0;
  } else {
    fr = 0.0;
  }

  return {fl, fr};
}

std::pair<double, double> WheelFrequencyReader::getAngularVelRadS(double pulses_per_rev) const
{
  if (pulses_per_rev <= 0.0) return {0.0, 0.0};
  const auto [fl, fr] = getFrequenciesHz();
  const double k = 2.0 * M_PI / pulses_per_rev;
  return {fl * k, fr * k};
}

void WheelFrequencyReader::alertCb_(int /*pi*/, unsigned user_gpio, unsigned level, uint32_t tick, void* user)
{
  auto* self = static_cast<WheelFrequencyReader*>(user);
  if (self) self->handleEdge_(user_gpio, level, tick);
}

void WheelFrequencyReader::handleEdge_(unsigned gpio, unsigned level, uint32_t tick)
{
  // En pigpio: level 0=low, 1=high, 2=timeout, 3=watchdog (selon config)
  if (level != PI_HIGH) return; // front montant uniquement

  if (static_cast<int>(gpio) == gpio_left_) {
    const uint32_t prev = last_tick_left_.exchange(tick);
    last_seen_tick_left_.store(tick);

    if (prev != 0) {
      const uint32_t delta = tick - prev;
      if (delta > 0) {
        const double f = 1e6 / static_cast<double>(delta);
        if (use_ema_.load()) {
          const double a = alpha_.load();
          const double old = freq_left_hz_.load();
          freq_left_hz_.store(a * f + (1.0 - a) * old);
        } else {
          freq_left_hz_.store(f);
        }
      }
    }
  } else if (static_cast<int>(gpio) == gpio_right_) {
    const uint32_t prev = last_tick_right_.exchange(tick);
    last_seen_tick_right_.store(tick);

    if (prev != 0) {
      const uint32_t delta = tick - prev;
      if (delta > 0) {
        const double f = 1e6 / static_cast<double>(delta);
        if (use_ema_.load()) {
          const double a = alpha_.load();
          const double old = freq_right_hz_.load();
          freq_right_hz_.store(a * f + (1.0 - a) * old);
        } else {
          freq_right_hz_.store(f);
        }
      }
    }
  }
}
