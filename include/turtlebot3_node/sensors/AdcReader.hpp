#pragma once

#include <cstdint>
#include <string>

class AdcReader
{
public:
  struct Sample
  {
    double ch1_V = 0.0;
    double ch4_V = 0.0;
    int16_t ch1_code = 0;
    int16_t ch4_code = 0;
    uint8_t ch1_cfg = 0;
    uint8_t ch4_cfg = 0;
    bool valid = false;
  };

  // IMPORTANT:
  // - pi : handle pigpiod_if2 (retourné par pigpio_start), -1 si vous n'utilisez pas de GPIO enable
  // - enable_gpio: GPIO optionnel qui alimente/active votre front-end analogique (-1 si pas utilisé)
  explicit AdcReader(
    int pi,
    std::string i2c_dev = "/dev/i2c-1",
    int addr_7b = 0x68,
    int enable_gpio = -1);

  ~AdcReader();

  AdcReader(const AdcReader&) = delete;
  AdcReader& operator=(const AdcReader&) = delete;

  bool init();
  void setEnabled(bool on);

  bool update();
  Sample getLatest() const;

  static double applyVoltageDivider(double adc_input_V, double r_top_ohm, double r_bottom_ohm);
  static double shuntToCurrent(double sense_V, double shunt_ohm, double gain);

private:
  static uint8_t make_cfg_(int channel_1_to_4);
  bool open_i2c_();
  bool read_channel_once_(int channel, int16_t& out_code, uint8_t& out_cfg);
  static double code_to_volts_(int16_t code, int gain);

  int pi_{-1};              // <-- pigpiod_if2 handle
  std::string i2c_dev_;
  int addr_7b_;
  int enable_gpio_;

  int fd_ = -1;
  Sample latest_;
};
