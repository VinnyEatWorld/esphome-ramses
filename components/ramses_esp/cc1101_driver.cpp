#include "cc1101_driver.h"
#include "esp_rom_sys.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <cmath>
#include <cstring>

static const char *TAG = "ramses_esp.cc1101";
static constexpr uint32_t CC1101_STATE_TIMEOUT_MS = 100;
static constexpr uint8_t CC1101_MARCSTATE_IDLE = 0x01;
static constexpr uint8_t CC1101_MARCSTATE_MASK = 0x1F;

#define delayMicroseconds(us) esp_rom_delay_us(us)

namespace esphome {
namespace ramses_esp {

static const uint8_t CC_RAMSES_CFG[CC_PARAM_MAX] = {
    0x0D, // CC_IOCFG2   GDO2- RX data
    0x2E, // CC_IOCFG1   GDO1- not used
    0x2E, // CC_IOCFG0   GDO0- TX data
    0x07, // CC_FIFOTHR  default
    0xD3, // CC_SYNC1    default
    0x91, // CC_SYNC0    default
    0x3D, // CC_PKTLEN   default
    0x04, // CC_PKTCTRL1 default
    0x31, // CC_PKTCTRL0 Asynchronous Serial, TX on GDO0, RX on GDOx
    0x00, // CC_ADDR     default
    0x00, // CC_CHANNR   default
    0x0F, // CC_FSCTRL1  default
    0x00, // CC_FSCTRL0  default
    0x21, // CC_FREQ2    868.3 MHz
    0x65, // CC_FREQ1
    0x6A, // CC_FREQ0
    0x6A, // CC_MDMCFG4
    0x83, // CC_MDMCFG3  DRATE_M=131 data rate=38,383.48Hz
    0x10, // CC_MDMCFG2  GFSK, No Sync Word
    0x22, // CC_MDMCFG1  FEC_EN=0, NUM_PREAMBLE=4, CHANSPC_E=2
    0xF8, // CC_MDMCFG0  Channel spacing 199.951 KHz
    0x50, // CC_DEVIATN
    0x07, // CC_MCSM2    default
    0x30, // CC_MCSM1    default
    0x18, // CC_MCSM0    Auto-calibrate on Idle to RX+TX
    0x16, // CC_FOCCFG   default
    0x6C, // CC_BSCFG    default
    0x43, // CC_AGCCTRL2
    0x40, // CC_AGCCTRL1 default
    0x91, // CC_AGCCTRL0 default
    0x87, // CC_WOREVT1  default
    0x6B, // CC_WOREVT0  default
    0xF8, // CC_WORCTRL  default
    0x56, // CC_FREND1   default
    0x10, // CC_FREND0   default
    0xE9, // CC_FSCAL3
    0x21, // CC_FSCAL2
    0x00, // CC_FSCAL1
    0x1F, // CC_FSCAL0
    0x41, // CC_RCCTRL1  default
    0x00, // CC_RCCTRL0  default
    0x59, // CC_FSTEST   default
    0x7F, // CC_PTEST    default
    0x3F, // CC_AGCTEST  default
    0x81, // CC_TEST2
    0x35, // CC_TEST1
    0x09  // CC_TEST0
};

static const uint8_t CC_DEFAULT_PA[CC_PA_MAX] = {0xC3, 0, 0, 0, 0, 0, 0, 0};

bool CC1101Driver::init(CC1101SPITransport *transport) {
  if (transport == nullptr) {
    ESP_LOGE(TAG, "ESPHome SPI transport is not available");
    return false;
  }
  this->transport_ = transport;

  // Reset through the ESPHome-owned SPI device. The shared SPI bus and CS
  // routing stay under ESPHome; this driver never initializes SPI2 itself.
  this->spi_reset();
  this->strobe(CC_SRES);

  const uint32_t reset_started = millis();
  uint8_t status = this->strobe(CC_SNOP);
  while ((status & CC_CHIP_RDY) != 0 &&
         (millis() - reset_started) < CC1101_STATE_TIMEOUT_MS) {
    delay(1);
    status = this->strobe(CC_SNOP);
  }
  if ((status & CC_CHIP_RDY) != 0) {
    ESP_LOGE(TAG, "CC1101 CHIP_RDYn timeout after %u ms (status=0x%02X)",
             static_cast<unsigned>(CC1101_STATE_TIMEOUT_MS), status);
    return false;
  }

  uint8_t partnum = this->read_reg(CC_PARTNUM);
  uint8_t version = this->read_reg(CC_VERSION);
  ESP_LOGI(TAG, "CC1101 Hardware Check: PartNum=0x%02X, Version=0x%02X",
           partnum, version);
  if (partnum == 0xFF && version == 0xFF) {
    ESP_LOGE(TAG, "No CC1101 response (PARTNUM=0x%02X, VERSION=0x%02X)",
             partnum, version);
    return false;
  }

  // Configure and verify while idle. The component starts RX only after its
  // UART/GDO path has been initialized.
  return this->configure_ramses_profile();
}

void CC1101Driver::spi_reset() {
  // SPIDevice::disable() releases a transaction lock, so do not deselect
  // before this component has acquired the SPI bus with select().
  delayMicroseconds(1);
  this->transport_->cc1101_select();
  delayMicroseconds(10);
  this->transport_->cc1101_deselect();
  delayMicroseconds(41);
}

bool CC1101Driver::spi_write_bytes(uint8_t *rx_data, const uint8_t *data,
                                   size_t len) {
  if (len == 0 || this->transport_ == nullptr || rx_data == nullptr)
    return false;
  this->transport_->cc1101_select();
  this->transport_->cc1101_transfer(data, rx_data, len);
  this->transport_->cc1101_deselect();
  return true;
}

bool CC1101Driver::spi_read_bytes(uint8_t *rx_data, const uint8_t *tx_data,
                                  size_t len) {
  if (len == 0 || this->transport_ == nullptr || rx_data == nullptr)
    return false;
  this->transport_->cc1101_select();
  this->transport_->cc1101_transfer(tx_data, rx_data, len);
  this->transport_->cc1101_deselect();
  return true;
}

uint8_t CC1101Driver::read_reg(uint8_t addr) {
  uint8_t tx[2] = {static_cast<uint8_t>(addr | CC_READ), 0};
  uint8_t rx[2] = {0, 0};
  this->spi_read_bytes(rx, tx, 2);
  return rx[1];
}

uint8_t CC1101Driver::write_reg(uint8_t addr, uint8_t val) {
  uint8_t tx[2] = {addr, val};
  uint8_t rx[2] = {0, 0};
  this->spi_write_bytes(rx, tx, sizeof(tx));
  return rx[0];
}

uint8_t CC1101Driver::strobe(uint8_t cmd) {
  uint8_t status = 0;
  this->spi_write_bytes(&status, &cmd, sizeof(cmd));
  return status;
}

uint8_t CC1101Driver::write_fifo(uint8_t b) {
  uint8_t res = this->write_reg(CC_FIFO, b);
  return res & 0x0F;
}

void CC1101Driver::write_fifo_burst(const uint8_t *data, size_t len) {
  if (len == 0)
    return;
  std::vector<uint8_t> buf(len + 1);
  buf[0] = CC_FIFO | CC_BURST;
  memcpy(buf.data() + 1, data, len);
  std::vector<uint8_t> rx(buf.size(), 0);
  this->spi_write_bytes(rx.data(), buf.data(), buf.size());
}

bool CC1101Driver::enter_idle_mode() {
  this->strobe(CC_SIDLE);
  const uint32_t started = millis();
  while ((millis() - started) < CC1101_STATE_TIMEOUT_MS) {
    if (CC_STATE(this->strobe(CC_SNOP)) == CC_STATE_IDLE)
      return true;
    delay(1);
  }
  ESP_LOGE(TAG, "CC1101 did not enter IDLE within %u ms",
           static_cast<unsigned>(CC1101_STATE_TIMEOUT_MS));
  return false;
}

bool CC1101Driver::enter_rx_mode() {
  if (!this->enter_idle_mode())
    return false;
  this->write_reg(CC_IOCFG0, 0x2E);   // GDO0 not needed / async
  this->write_reg(CC_PKTCTRL0, 0x32); // Asynchronous, infinite packet
  this->strobe(CC_SFRX);
  this->strobe(CC_SRX);

  const uint32_t started = millis();
  while ((millis() - started) < CC1101_STATE_TIMEOUT_MS) {
    if (CC_STATE(this->strobe(CC_SNOP)) == CC_STATE_RX)
      return true;
    delay(1);
  }
  ESP_LOGE(TAG, "CC1101 did not enter RX within %u ms",
           static_cast<unsigned>(CC1101_STATE_TIMEOUT_MS));
  return false;
}

bool CC1101Driver::enter_tx_mode() {
  if (!this->enter_idle_mode())
    return false;
  this->write_reg(CC_PKTCTRL0, 0x02); // Fifo mode, infinite packet
  this->write_reg(CC_IOCFG0, 0x03);   // Falling edge, TX Fifo low
  this->strobe(CC_SFTX);
  this->strobe(CC_STX);

  const uint32_t started = millis();
  while ((millis() - started) < CC1101_STATE_TIMEOUT_MS) {
    if (CC_STATE(this->strobe(CC_SNOP)) == CC_STATE_TX)
      return true;
    delay(1);
  }
  ESP_LOGE(TAG, "CC1101 did not enter TX within %u ms",
           static_cast<unsigned>(CC1101_STATE_TIMEOUT_MS));
  return false;
}

void CC1101Driver::fifo_end() {
  this->write_reg(CC_IOCFG0, 0x05); // Rising edge, TX Fifo empty
}

uint8_t CC1101Driver::read_rssi() {
  int8_t rssi = static_cast<int8_t>(this->read_reg(CC_RSSI));
  rssi = rssi / 2 - 74;
  return static_cast<uint8_t>(-rssi);
}

bool CC1101Driver::apply_ramses_config() {
  if (!this->configure_ramses_profile())
    return false;
  if (!this->enter_rx_mode())
    return false;
  ESP_LOGI(TAG, "CC1101 configured for RAMSES II RX (868.3 MHz)");
  return true;
}

bool CC1101Driver::configure_ramses_profile() {
  if (!this->enter_idle_mode())
    return false;
  for (uint8_t i = 0; i < CC_PARAM_MAX; i++) {
    this->write_reg(i, CC_RAMSES_CFG[i]);
  }
  // TX Fifo Threshold 17
  this->write_reg(CC_FIFOTHR, (CC_RAMSES_CFG[CC_FIFOTHR] & 0xF0) + 11);
  for (uint8_t i = 0; i < CC_PA_MAX; i++) {
    this->write_reg(CC_PATABLE, CC_DEFAULT_PA[i]);
  }

  struct RegisterCheck {
    uint8_t address;
    const char *name;
  };
  static const RegisterCheck checks[] = {
      {CC_IOCFG2, "IOCFG2"},     {CC_IOCFG0, "IOCFG0"},
      {CC_FIFOTHR, "FIFOTHR"},   {CC_SYNC1, "SYNC1"},
      {CC_SYNC0, "SYNC0"},       {CC_PKTLEN, "PKTLEN"},
      {CC_PKTCTRL1, "PKTCTRL1"}, {CC_PKTCTRL0, "PKTCTRL0"},
      {CC_CHANNR, "CHANNR"},     {CC_FSCTRL1, "FSCTRL1"},
      {CC_FSCTRL0, "FSCTRL0"},   {CC_FREQ2, "FREQ2"},
      {CC_FREQ1, "FREQ1"},       {CC_FREQ0, "FREQ0"},
      {CC_MDMCFG4, "MDMCFG4"},   {CC_MDMCFG3, "MDMCFG3"},
      {CC_MDMCFG2, "MDMCFG2"},   {CC_MDMCFG1, "MDMCFG1"},
      {CC_MDMCFG0, "MDMCFG0"},   {CC_DEVIATN, "DEVIATN"},
      {CC_MCSM1, "MCSM1"},       {CC_MCSM0, "MCSM0"},
      {CC_FOCCFG, "FOCCFG"},     {CC_BSCFG, "BSCFG"},
      {CC_AGCCTRL2, "AGCCTRL2"}, {CC_AGCCTRL1, "AGCCTRL1"},
      {CC_AGCCTRL0, "AGCCTRL0"},
  };

  for (const auto &check : checks) {
    uint8_t expected = CC_RAMSES_CFG[check.address];
    if (check.address == CC_FIFOTHR)
      expected = (expected & 0xF0) | 11;

    const uint8_t actual = this->read_reg(check.address);
    if (actual != expected) {
      ESP_LOGE(TAG,
               "CC1101 register %s (0x%02X) mismatch: expected 0x%02X, "
               "read 0x%02X",
               check.name, check.address, expected, actual);
      return false;
    }
  }

  this->strobe(CC_SIDLE);
  const uint8_t marcstate = this->read_reg(CC_MARCSTATE) & CC1101_MARCSTATE_MASK;
  if (marcstate != CC1101_MARCSTATE_IDLE) {
    ESP_LOGE(TAG,
             "CC1101 MARCSTATE is not IDLE after initialization: expected "
             "0x%02X, read 0x%02X",
             CC1101_MARCSTATE_IDLE, marcstate);
    return false;
  }
  ESP_LOGI(TAG, "CC1101 MARCSTATE=0x%02X (IDLE)", marcstate);
  return true;
}

void CC1101Driver::apply_custom_tx_config(const CustomTxConfig &cfg) {
  if (!this->enter_idle_mode())
    return;

  // 1. Calculate frequency registers: F_carrier = (F_xosc / 2^16) * FREQ
  // FREQ = freq_hz * 65536 / 26000000
  uint64_t freq_reg = ((uint64_t)cfg.frequency * 65536ULL) / 26000000ULL;
  this->write_reg(CC_FREQ2, (freq_reg >> 16) & 0xFF);
  this->write_reg(CC_FREQ1, (freq_reg >> 8) & 0xFF);
  this->write_reg(CC_FREQ0, freq_reg & 0xFF);

  // 2. Calculate symbol rate registers: DRATE = ((256 + DRATE_M) * 2^DRATE_E /
  // 2^28) * 26 MHz
  int drate_e = static_cast<int>(
      std::floor(std::log2(cfg.symbol_rate * 1048576.0f / 26000000.0f)));
  if (drate_e < 0)
    drate_e = 0;
  if (drate_e > 15)
    drate_e = 15;
  int drate_m =
      static_cast<int>(std::round((cfg.symbol_rate * 268435456.0f) /
                                      (26000000.0f * std::pow(2.0f, drate_e)) -
                                  256.0f));
  if (drate_m < 0)
    drate_m = 0;
  if (drate_m > 255)
    drate_m = 255;

  uint8_t mdmcfg4 = (CC_RAMSES_CFG[CC_MDMCFG4] & 0xF0) | (drate_e & 0x0F);
  this->write_reg(CC_MDMCFG4, mdmcfg4);
  this->write_reg(CC_MDMCFG3, drate_m & 0xFF);

  // 3. Sync word & modulation (2-FSK, 16/16 sync word detection/insertion)
  this->write_reg(CC_SYNC1, cfg.sync1);
  this->write_reg(CC_SYNC0, cfg.sync0);
  this->write_reg(CC_MDMCFG2, 0x03); // 2-FSK, 16/16 sync bits, no Manchester

  // 4. Packet Control
  uint8_t pktctrl0 =
      (cfg.crc_enable ? 0x04 : 0x00) | (cfg.packet_length == 0 ? 0x01 : 0x00);
  this->write_reg(CC_PKTCTRL0, pktctrl0);
  this->write_reg(CC_PKTCTRL1, 0x04); // Append status (RSSI/LQI)
  this->write_reg(CC_PKTLEN, cfg.packet_length == 0 ? 0xFF : cfg.packet_length);

  // GDO0 asserts on sync, deasserts on end of packet
  this->write_reg(CC_IOCFG0, 0x06);

  this->strobe(CC_SFTX);
  ESP_LOGV(TAG,
           "CC1101 custom TX profile applied (Freq: %lu Hz, Rate: %.1f Baud, "
           "Sync: %02X%02X)",
           (unsigned long)cfg.frequency, cfg.symbol_rate, cfg.sync1, cfg.sync0);
}

} // namespace ramses_esp
} // namespace esphome
