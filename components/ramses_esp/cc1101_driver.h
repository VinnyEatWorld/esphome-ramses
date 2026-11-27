#pragma once

#include "cc_const.h"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace esphome {
namespace ramses_esp {

struct CustomTxConfig {
  uint32_t frequency{868300000}; // Hz
  float symbol_rate{64000.0f};   // Baud
  uint8_t sync0{0x91};
  uint8_t sync1{0xD3};
  bool crc_enable{true};
  uint8_t packet_length{0}; // 0 = variable length
};

class CC1101SPITransport {
public:
  virtual ~CC1101SPITransport() = default;
  virtual void cc1101_select() = 0;
  virtual void cc1101_deselect() = 0;
  virtual void cc1101_transfer(const uint8_t *tx, uint8_t *rx, size_t len) = 0;
};

class CC1101Driver {
public:
  CC1101Driver() = default;

  bool init(CC1101SPITransport *transport);

  uint8_t read_reg(uint8_t addr);
  uint8_t write_reg(uint8_t addr, uint8_t val);
  uint8_t strobe(uint8_t cmd);
  uint8_t write_fifo(uint8_t b);
  void write_fifo_burst(const uint8_t *data, size_t len);

  bool enter_idle_mode();
  bool enter_rx_mode();
  bool prepare_tx_mode();
  bool start_tx_mode();
  bool enter_tx_mode();
  void fifo_end();
  int8_t read_rssi();

  bool apply_ramses_config();
  void apply_custom_tx_config(const CustomTxConfig &cfg);

protected:
  void spi_reset();
  bool configure_ramses_profile();
  bool spi_write_bytes(uint8_t *rx_data, const uint8_t *data, size_t len);
  bool spi_read_bytes(uint8_t *rx_data, const uint8_t *tx_data, size_t len);

  CC1101SPITransport *transport_{nullptr};
};

} // namespace ramses_esp
} // namespace esphome
