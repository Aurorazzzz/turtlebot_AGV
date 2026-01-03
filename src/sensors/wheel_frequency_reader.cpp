#include "turtlebot3_node/sensors/wheel_frequency_reader.hpp"

#include <pigpio.h>
#include <cstdio>
#include <cmath>

WheelFrequencyReader* WheelFrequencyReader::instance_ = nullptr;

WheelFrequencyReader::WheelFrequencyReader(int gpio_left, int gpio_right)
: gpio_left_(gpio_left), gpio_right_(gpio_right)
{
}

WheelFrequencyReader::~WheelFrequencyReader()
{
  // Détache callbacks
  if (callbacks_attached_) {
    if (gpioHardwareRevision() != 0) {
    gpioSetAlertFunc(gpio_left_, nullptr);
    gpioSetAlertFunc(gpio_right_, nullptr);
    callbacks_attached_ = false;
    }
  }

  // IMPORTANT :
  // Si vous avez d'autres composants pigpio, ne terminez pas ici.
  // Dans une archi plus grande, on centralise gpioInitialise()/gpioTerminate().
//   if (pigpio_inited_) {
//     gpioTerminate();
//     pigpio_inited_ = false;
//   }

  if (instance_ == this) instance_ = nullptr;
}

bool WheelFrequencyReader::init()
{
  if (instance_ != nullptr && instance_ != this) {
    std::fprintf(stderr, "WheelFrequencyReader: only one instance supported\n");
    return false;
  }

  // Vérifie pigpio global
  if (gpioHardwareRevision() == 0) {
    std::fprintf(stderr, "WheelFrequencyReader::init(): pigpio not initialized\n");
    return false;
  }

  instance_ = this;

  if (gpioSetMode(gpio_left_, PI_INPUT) != 0 ||
      gpioSetMode(gpio_right_, PI_INPUT) != 0) {
    std::fprintf(stderr, "gpioSetMode failed\n");
    return false;
  }

  gpioSetPullUpDown(gpio_left_, PI_PUD_DOWN);
  gpioSetPullUpDown(gpio_right_, PI_PUD_DOWN);

  if (gpioSetAlertFunc(gpio_left_, &WheelFrequencyReader::alertTrampoline_) != 0 ||
      gpioSetAlertFunc(gpio_right_, &WheelFrequencyReader::alertTrampoline_) != 0) {
    std::fprintf(stderr, "gpioSetAlertFunc failed\n");
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
  // Gestion timeout : si pas de front depuis X ms => 0
  const uint32_t now = gpioTick(); // microsecondes (uint32 wrap OK)
  const uint32_t timeout_us = timeout_ms_.load() * 1000u;

  auto left_last = last_seen_tick_left_.load();
  auto right_last = last_seen_tick_right_.load();

  double fl = freq_left_hz_.load();
  double fr = freq_right_hz_.load();

  if (left_last != 0) {
    uint32_t dt = now - left_last;
    if (dt > timeout_us) fl = 0.0;
  } else {
    fl = 0.0;
  }

  if (right_last != 0) {
    uint32_t dt = now - right_last;
    if (dt > timeout_us) fr = 0.0;
  } else {
    fr = 0.0;
  }

  return {fl, fr};
}

std::pair<double, double> WheelFrequencyReader::getAngularVelRadS(double pulses_per_rev) const
{
  // f [Hz] = pulses/s ; rev/s = f / pulses_per_rev ; rad/s = 2pi * rev/s
  if (pulses_per_rev <= 0.0) return {0.0, 0.0};
  const auto [fl, fr] = getFrequenciesHz();
  const double k = 2.0 * M_PI / pulses_per_rev;
  return {fl * k, fr * k};
}

void WheelFrequencyReader::alertTrampoline_(int gpio, int level, uint32_t tick)
{
  if (instance_) instance_->handleEdge_(gpio, level, tick);
}

void WheelFrequencyReader::handleEdge_(int gpio, int level, uint32_t tick)
{
  if (level != PI_HIGH) return; // front montant

  if (gpio == gpio_left_) {
    const uint32_t prev = last_tick_left_.exchange(tick);
    last_seen_tick_left_.store(tick);

    if (prev != 0) {
      const uint32_t delta = tick - prev; // us
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
  } else if (gpio == gpio_right_) {
    const uint32_t prev = last_tick_right_.exchange(tick);
    last_seen_tick_right_.store(tick);

    if (prev != 0) {
      const uint32_t delta = tick - prev; // us
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
