#include "ramses_esp.h"
#include "esphome/core/log.h"
#if __has_include("esp_timer.h")
#include "esp_timer.h"
#endif
#include <algorithm>
#include <arpa/inet.h>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

static const char *const TAG = "ramses_esp";
static constexpr uint8_t RAMSES_MARCSTATE_IDLE = 0x01;
static constexpr uint8_t RAMSES_MARCSTATE_MASK = 0x1F;
static constexpr uint8_t RAMSES_MARCSTATE_TXFIFO_UNDERFLOW = 0x16;

namespace esphome {
namespace ramses_esp {

void RamsesESPComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up RAMSES ESP component...");

  this->radio_mutex_ = xSemaphoreCreateMutex();
  this->rx_msg_queue_ = xQueueCreate(16, sizeof(RamsesMessage));
  this->tx_msg_queue_ = xQueueCreate(8, sizeof(RamsesMessage));

  // Set up the ESPHome SPI device before touching the CC1101. This reuses the
  // configured bus and lets other devices (such as the M5Stack display) share it.
  this->spi_setup();
  if (!this->cc1101_.init(this)) {
    ESP_LOGE(TAG, "Failed to initialize CC1101 transceiver!");
    this->mark_failed();
    return;
  }

  if (!this->frame_handler_.init(this->uart_num_, this->gdo0_pin_,
                                 this->gdo2_pin_, &this->cc1101_)) {
    ESP_LOGE(TAG, "Failed to initialize RAMSES frame handler!");
    this->mark_failed();
    return;
  }

  // Start RF reception only after the UART connected to GDO2 is ready.
  if (!this->cc1101_.enter_rx_mode()) {
    ESP_LOGE(TAG, "Failed to start CC1101 RX after UART initialization!");
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "CC1101 RX started after UART initialization");

  this->frame_handler_.set_on_message_callback(
      [this](const RamsesMessage &msg) {
        if (this->rx_msg_queue_ != nullptr) {
          xQueueSend(this->rx_msg_queue_, &msg, 0);
        }
      });

  xTaskCreatePinnedToCore(RamsesESPComponent::radio_task_trampoline,
                          "ramses_radio", 4096, this, 10,
                          &this->radio_task_handle_, 0);

  this->start_tcp_server();
}

void RamsesESPComponent::radio_task_trampoline(void *arg) {
  reinterpret_cast<RamsesESPComponent *>(arg)->radio_task();
}

void RamsesESPComponent::radio_task() {
  ESP_LOGI(TAG, "RAMSES Radio task started");
  while (true) {
    if (!this->paused_) {
      if (xSemaphoreTake(this->radio_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (!this->paused_) {
          this->frame_handler_.work();
        }
        xSemaphoreGive(this->radio_mutex_);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void RamsesESPComponent::pause() {
  if (this->paused_)
    return;
  ESP_LOGD(TAG, "Pausing RAMSES Radio for external operation...");
  if (xSemaphoreTake(this->radio_mutex_, pdMS_TO_TICKS(200)) == pdTRUE) {
    this->paused_ = true;
    this->frame_handler_.rx_disable();
    this->cc1101_.enter_idle_mode();
  }
}

void RamsesESPComponent::resume() {
  if (!this->paused_)
    return;
  ESP_LOGD(TAG, "Resuming RAMSES Radio reception...");
  if (!this->cc1101_.apply_ramses_config()) {
    ESP_LOGE(TAG, "Unable to resume CC1101 RX; radio remains paused");
    this->mark_failed();
    xSemaphoreGive(this->radio_mutex_);
    return;
  }
  this->frame_handler_.rx_enable();
  this->paused_ = false;
  xSemaphoreGive(this->radio_mutex_);
}

void RamsesESPComponent::start_tcp_server() {
  this->server_fd_ = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (this->server_fd_ < 0) {
    ESP_LOGE(TAG, "Unable to create TCP socket: errno %d", errno);
    return;
  }

  int opt = 1;
  setsockopt(this->server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  fcntl(this->server_fd_, F_SETFL, O_NONBLOCK);

  struct sockaddr_in dest_addr = {
      .sin_family = AF_INET,
      .sin_port = htons(this->port_),
      .sin_addr =
          {
              .s_addr = htonl(INADDR_ANY),
          },
  };

  int err =
      bind(this->server_fd_, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  if (err != 0) {
    ESP_LOGE(TAG, "Socket unable to bind on port %u: errno %d", this->port_,
             errno);
    close(this->server_fd_);
    this->server_fd_ = -1;
    return;
  }

  err = listen(this->server_fd_, 4);
  if (err != 0) {
    ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
    close(this->server_fd_);
    this->server_fd_ = -1;
    return;
  }

  ESP_LOGI(TAG, "RAMSES HGI80 TCP Server listening on port %u", this->port_);
}

void RamsesESPComponent::loop() {
  // 1. Drain decoded incoming RAMSES messages from Radio queue
  RamsesMessage rx_msg;
  while (this->rx_msg_queue_ != nullptr &&
         xQueueReceive(this->rx_msg_queue_, &rx_msg, 0) == pdTRUE) {
    std::string hgi80 = rx_msg.to_hgi80();
    this->broadcast_hgi80(hgi80);
    for (auto &cb : this->on_message_callbacks_) {
      cb(hgi80);
    }
    for (auto &cb : this->raw_message_callbacks_) {
      cb(rx_msg);
    }
  }

  // 2. Accept and manage TCP clients
  this->handle_tcp_clients();

  // 3. Process outbound RAMSES messages
  if (!this->paused_) {
    this->process_tx_queue();
  }
}

bool RamsesESPComponent::send_message(const RamsesMessage &msg) {
  if (this->tx_msg_queue_ == nullptr)
    return false;
  return xQueueSend(this->tx_msg_queue_, &msg, 0) == pdTRUE;
}

void RamsesESPComponent::handle_tcp_clients() {
  if (this->server_fd_ < 0)
    return;

  // Accept new clients
  struct sockaddr_in source_addr;
  socklen_t addr_len = sizeof(source_addr);
  int client_fd =
      accept(this->server_fd_, (struct sockaddr *)&source_addr, &addr_len);
  if (client_fd >= 0) {
    fcntl(client_fd, F_SETFL, O_NONBLOCK);
    this->client_fds_.push_back(client_fd);
    ESP_LOGI(TAG, "TCP Client connected from %s (Total clients: %d)",
             inet_ntoa(source_addr.sin_addr), (int)this->client_fds_.size());
  }

  // Read data from existing clients
  for (auto it = this->client_fds_.begin(); it != this->client_fds_.end();) {
    int fd = *it;
    char rx_buffer[256];
    int len = recv(fd, rx_buffer, sizeof(rx_buffer) - 1, 0);
    if (len > 0) {
      rx_buffer[len] = '\0';
      std::string line(rx_buffer);
      // Remove trailing CR/LF
      line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
      line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
      if (!line.empty()) {
        ESP_LOGI(TAG, "TCP Rx Command: %s", line.c_str());
        this->send_hgi80_command(line);
      }
      ++it;
    } else if (len == 0 ||
               (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
      ESP_LOGI(TAG, "TCP Client disconnected");
      close(fd);
      it = this->client_fds_.erase(it);
    } else {
      ++it;
    }
  }
}

void RamsesESPComponent::broadcast_hgi80(const std::string &hgi80) {
  std::string line = hgi80 + "\r\n";
  for (auto it = this->client_fds_.begin(); it != this->client_fds_.end();) {
    int fd = *it;
    int sent = send(fd, line.c_str(), line.length(), 0);
    if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      close(fd);
      it = this->client_fds_.erase(it);
    } else {
      ++it;
    }
  }
}

bool RamsesESPComponent::send_hgi80_command(const std::string &cmd) {
  RamsesMessage msg;
  if (!msg.from_hgi80(cmd)) {
    ESP_LOGW(TAG, "Invalid HGI80 command format: %s", cmd.c_str());
    return false;
  }

  if (this->tx_msg_queue_ != nullptr) {
    return xQueueSend(this->tx_msg_queue_, &msg, pdMS_TO_TICKS(100)) == pdTRUE;
  }
  return false;
}

void RamsesESPComponent::process_tx_queue() {
  if (this->tx_msg_queue_ == nullptr)
    return;

  // Enforce CSMA/CD quiet window: wait at least 250ms after receiving an RF
  // packet before transmitting to avoid stepping on reply frames from
  // destination devices
  uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
  if (now_ms - this->frame_handler_.get_last_rx_ms() < 250)
    return;

  RamsesMessage tx_msg;

  if (xQueueReceive(this->tx_msg_queue_, &tx_msg, 0) == pdTRUE) {
    if (xSemaphoreTake(this->radio_mutex_, pdMS_TO_TICKS(200)) == pdTRUE) {
      ESP_LOGI(TAG, "Transmitting RAMSES packet: %s",
               tx_msg.to_hgi80().c_str());

      this->frame_handler_.rx_disable();
      if (!this->cc1101_.enter_idle_mode()) {
        this->frame_handler_.rx_enable();
        xSemaphoreGive(this->radio_mutex_);
        return;
      }

      std::vector<uint8_t> raw_frame = tx_msg.to_raw_frame();
      if (!this->cc1101_.prepare_tx_mode()) {
        ESP_LOGE(TAG, "Unable to prepare CC1101 TX mode");
        if (!this->cc1101_.apply_ramses_config()) {
          ESP_LOGE(TAG, "Unable to restore CC1101 RX after TX preparation failure");
          this->mark_failed();
        }
        this->frame_handler_.rx_enable();
        xSemaphoreGive(this->radio_mutex_);
        return;
      }

      // Preload most of the 64-byte TX FIFO before starting the carrier. This
      // prevents an immediate FIFO underflow between STX and the first SPI
      // write; longer frames are streamed while the radio drains the FIFO.
      constexpr size_t TX_FIFO_PRELOAD_SIZE = 60;
      const size_t preload = std::min(raw_frame.size(), TX_FIFO_PRELOAD_SIZE);
      if (preload > 0)
        this->cc1101_.write_fifo_burst(raw_frame.data(), preload);

      if (!this->cc1101_.start_tx_mode()) {
        if (!this->cc1101_.apply_ramses_config()) {
          ESP_LOGE(TAG, "Unable to restore CC1101 RX after TX start failure");
          this->mark_failed();
        }
        this->frame_handler_.rx_enable();
        xSemaphoreGive(this->radio_mutex_);
        return;
      }

      constexpr uint32_t RAMSES_BIT_RATE_BPS = 38383;
      constexpr uint32_t TX_AIRTIME_GUARD_MS = 5;
      const uint32_t tx_start_ms = millis();
      const uint32_t tx_airtime_ms = static_cast<uint32_t>(
          (raw_frame.size() * 8ULL * 1000ULL + RAMSES_BIT_RATE_BPS - 1) /
          RAMSES_BIT_RATE_BPS);
      const uint32_t tx_timeout_ms = tx_airtime_ms + TX_AIRTIME_GUARD_MS;
      size_t sent = preload;
      bool tx_fifo_underflow = false;
      while (sent < raw_frame.size() &&
             (millis() - tx_start_ms < tx_timeout_ms)) {
        const uint8_t status = this->cc1101_.strobe(CC_SNOP);
        const uint8_t state = CC_STATE(status);
        if (state == CC_STATE_TX_UNDERFLOW) {
          tx_fifo_underflow = true;
          break;
        }
        if ((status & CC_FIFO_MASK) == 0) {
          vTaskDelay(pdMS_TO_TICKS(1));
          continue;
        }
        this->cc1101_.write_fifo(raw_frame[sent++]);
      }

      if (sent != raw_frame.size()) {
        ESP_LOGE(TAG,
                 "CC1101 TX FIFO underflow/incomplete frame: queued %u of %u bytes%s",
                 static_cast<unsigned>(sent),
                 static_cast<unsigned>(raw_frame.size()),
                 tx_fifo_underflow ? " (radio entered underflow)" : "");
      }

      this->cc1101_.fifo_end();
      // Infinite-length FIFO mode has no hardware packet-end event. Wait for
      // the calculated on-air duration (including a guard) before resetting
      // the radio, then inspect the final state and FIFO count for diagnostics.
      while (millis() - tx_start_ms < tx_timeout_ms)
        vTaskDelay(pdMS_TO_TICKS(1));

      const uint8_t final_txbytes = this->cc1101_.read_reg(CC_TXBYTES);
      const uint8_t final_marcstate = this->cc1101_.read_reg(CC_MARCSTATE);
      const uint8_t remaining = final_txbytes & 0x7F;
      const uint8_t final_state = final_marcstate & RAMSES_MARCSTATE_MASK;
      if (sent != raw_frame.size()) {
        ESP_LOGE(TAG,
                 "CC1101 TX incomplete: queued=%u/%u bytes, TXBYTES=%u, MARCSTATE=0x%02X",
                 static_cast<unsigned>(sent),
                 static_cast<unsigned>(raw_frame.size()), remaining,
                 final_state);
      } else if (remaining != 0 ||
                 (final_state != RAMSES_MARCSTATE_IDLE &&
                  final_state != RAMSES_MARCSTATE_TXFIFO_UNDERFLOW)) {
        ESP_LOGW(TAG,
                 "CC1101 TX completion not confirmed: TXBYTES=%u MARCSTATE=0x%02X",
                 remaining, final_state);
      } else {
        ESP_LOGI(TAG,
                 "CC1101 TX drain observed: queued=%u bytes, remaining=%u, MARCSTATE=0x%02X, underflow=%u",
                 static_cast<unsigned>(sent), remaining, final_state,
                 (final_txbytes & 0x80) != 0);
      }

      if (!this->cc1101_.apply_ramses_config()) {
        ESP_LOGE(TAG, "Unable to restore CC1101 RX after TX");
        this->mark_failed();
      }
      this->frame_handler_.rx_enable();

      xSemaphoreGive(this->radio_mutex_);
    }
  }
}

void RamsesESPComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "RAMSES ESP Transceiver & Gateway:");
  ESP_LOGCONFIG(TAG, "  CC1101 SPI: ESPHome shared bus, mode 0, MSB first, 1 MHz");
  ESP_LOGCONFIG(TAG, "  GDO0 Pin (CC1101 TX): GPIO%d", this->gdo0_pin_);
  if (this->gdo2_pin_ != GPIO_NUM_NC) {
    ESP_LOGCONFIG(TAG, "  GDO2 Pin (CC1101 RX): GPIO%d", this->gdo2_pin_);
  }
  ESP_LOGCONFIG(TAG, "  UART Port: UART%d", this->uart_num_);
  ESP_LOGCONFIG(TAG, "  TCP Server Port: %u", this->port_);
}

} // namespace ramses_esp
} // namespace esphome
