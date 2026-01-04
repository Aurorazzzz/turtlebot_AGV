#include "turtlebot3_node/sensors/AdcReader.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <pigpiod_if2.h>   // <-- au lieu de pigpio.h

#include <cerrno>
#include <cstdio>
#include <cstring>

namespace {
constexpr double VREF = 2.048; // MCP3428
}

AdcReader::AdcReader(std::string i2c_dev, int addr_7b, int enable_gpio)
: AdcReader(-1, std::move(i2c_dev), addr_7b, enable_gpio) // compat si vous aviez des appels anciens
{
}

// Nouveau constructeur recommandé
AdcReader::AdcReader(int pi, std::string i2c_dev, int addr_7b, int enable_gpio)
: pi_(pi), i2c_dev_(std::move(i2c_dev)), addr_7b_(addr_7b), enable_gpio_(enable_gpio)
{
}

AdcReader::~AdcReader()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  // NE PAS arrêter pigpiod ici : c'est géré au niveau TurtleBot3 (pigpio_stop(pi_))
}

bool AdcReader::init()
{
  if (!open_i2c_()) {
    return false;
  }

  // GPIO enable via pigpiod (si demandé)
  if (enable_gpio_ >= 0) {
    if (pi_ < 0) {
      std::fprintf(stderr, "AdcReader::init(): enable_gpio set but pi handle is invalid (pi=%d)\n", pi_);
      return false;
    }
    set_mode(pi_, enable_gpio_, PI_OUTPUT);
    gpio_write(pi_, enable_gpio_, 1);
  }

  return true;
}

void AdcReader::setEnabled(bool on)
{
  if (enable_gpio_ < 0) return;
  if (pi_ < 0) return;
  gpio_write(pi_, enable_gpio_, on ? 1 : 0);
}

bool AdcReader::update()
{
  int16_t code1 = 0, code4 = 0;
  uint8_t cfg1 = 0, cfg4 = 0;

  if (fd_ < 0) {
    latest_.valid = false;
    return false;
  }

  if (!read_channel_once_(1, code1, cfg1)) {
    latest_.valid = false;
    return false;
  }
  if (!read_channel_once_(4, code4, cfg4)) {
    latest_.valid = false;
    return false;
  }

  const double v1 = code_to_volts_(code1, 1);
  const double v4 = code_to_volts_(code4, 1);

  latest_.ch1_code = code1;
  latest_.ch4_code = code4;
  latest_.ch1_cfg = cfg1;
  latest_.ch4_cfg = cfg4;
  latest_.ch1_V = v1;
  latest_.ch4_V = v4;
  latest_.valid = true;

  return true;
}

AdcReader::Sample AdcReader::getLatest() const
{
  return latest_;
}

uint8_t AdcReader::make_cfg_(int channel_1_to_4)
{
  const uint8_t RDY = 1u << 7;
  const uint8_t OC  = 1u << 4;       // continuous
  const uint8_t S16 = 0b10u << 2;    // 16-bit
  const uint8_t G1  = 0b00u;         // gain x1

  uint8_t C = 0;
  switch (channel_1_to_4) {
    case 1: C = 0b00u << 5; break;
    case 2: C = 0b01u << 5; break;
    case 3: C = 0b10u << 5; break;
    case 4: C = 0b11u << 5; break;
    default: C = 0b00u << 5; break;
  }

  return static_cast<uint8_t>(RDY | C | OC | S16 | G1);
}

bool AdcReader::open_i2c_()
{
  fd_ = ::open(i2c_dev_.c_str(), O_RDWR);
  if (fd_ < 0) {
    std::fprintf(stderr, "open(%s) failed: %s\n", i2c_dev_.c_str(), std::strerror(errno));
    return false;
  }

  if (ioctl(fd_, I2C_SLAVE, addr_7b_) < 0) {
    std::fprintf(stderr, "ioctl(I2C_SLAVE, 0x%02X) failed: %s\n", addr_7b_, std::strerror(errno));
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  return true;
}

bool AdcReader::read_channel_once_(int channel, int16_t& out_code, uint8_t& out_cfg)
{
  const uint8_t cfg = make_cfg_(channel);

  if (::write(fd_, &cfg, 1) != 1) {
    std::fprintf(stderr, "write(cfg) failed: %s\n", std::strerror(errno));
    return false;
  }

  for (int tries = 0; tries < 200; ++tries) {
    uint8_t buf[3] = {0, 0, 0};
    const ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n != 3) {
      std::fprintf(stderr, "read(data) failed: %s\n", std::strerror(errno));
      return false;
    }

    const uint8_t cfg_read = buf[2];
    const bool ready = ((cfg_read & 0x80u) == 0u);
    if (ready) {
      out_code = static_cast<int16_t>((buf[0] << 8) | buf[1]);
      out_cfg = cfg_read;
      return true;
    }

    usleep(1000);
  }

  std::fprintf(stderr, "Timeout: conversion not ready (channel %d)\n", channel);
  return false;
}

double AdcReader::code_to_volts_(int16_t code, int gain)
{
  return (static_cast<double>(code) * (VREF / 32768.0)) / static_cast<double>(gain);
}

double AdcReader::applyVoltageDivider(double adc_input_V, double r_top_ohm, double r_bottom_ohm)
{
  if (r_bottom_ohm <= 0.0) return 0.0;
  return adc_input_V * (r_top_ohm + r_bottom_ohm) / r_bottom_ohm;
}

double AdcReader::shuntToCurrent(double sense_V, double shunt_ohm, double gain)
{
  if (shunt_ohm <= 0.0 || gain <= 0.0) return 0.0;
  return sense_V / (shunt_ohm * gain);
}
