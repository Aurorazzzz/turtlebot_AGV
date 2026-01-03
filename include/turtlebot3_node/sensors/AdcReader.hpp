#pragma once

#include <cstdint>
#include <string>

class AdcReader
{
public:
  struct Sample
  {
    double ch1_V = 0.0;      // Volts (ex: tension batterie après votre pont diviseur)
    double ch4_V = 0.0;      // Volts (ex: tension shunt / sortie INA / etc.)
    int16_t ch1_code = 0;
    int16_t ch4_code = 0;
    uint8_t ch1_cfg = 0;
    uint8_t ch4_cfg = 0;
    bool valid = false;
  };

  // i2c_dev: "/dev/i2c-1"
  // addr_7b: 0x68 pour MCP3428 avec A2A1A0=000
  // enable_gpio: GPIO qui alimente/active votre front-end analogique (optionnel : -1 si pas utilisé)
  explicit AdcReader(
    std::string i2c_dev = "/dev/i2c-1",
    int addr_7b = 0x68,
    int enable_gpio = -1);

  ~AdcReader();

  AdcReader(const AdcReader&) = delete;
  AdcReader& operator=(const AdcReader&) = delete;

  // Initialise I2C + pigpio (si enable_gpio >= 0)
  bool init();

  // Active/désactive l’alim/enable (si enable_gpio >= 0)
  void setEnabled(bool on);

  // Lit CH1 et CH4, met à jour la dernière mesure
  bool update();

  // Renvoie la dernière mesure
  Sample getLatest() const;

  // Helpers conversion "mesure" (à adapter selon votre électronique)
  // Exemple : conversion tension batterie via pont diviseur
  static double applyVoltageDivider(double adc_input_V, double r_top_ohm, double r_bottom_ohm);

  // Exemple : conversion courant via shunt + ampli (ou INA)
  // current_A = (sense_V / (shunt_ohm * gain))
  static double shuntToCurrent(double sense_V, double shunt_ohm, double gain);

private:
  static uint8_t make_cfg_(int channel_1_to_4);
  bool open_i2c_();
  bool read_channel_once_(int channel, int16_t& out_code, uint8_t& out_cfg);
  static double code_to_volts_(int16_t code, int gain);

  std::string i2c_dev_;
  int addr_7b_;
  int enable_gpio_;

  int fd_ = -1;
  bool pigpio_inited_ = false;

  Sample latest_;
};
