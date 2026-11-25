#include "ramses_fan.h"
#include "esphome/core/defines.h"

#ifdef USE_FAN

#include "esphome/core/log.h"
#include <algorithm>
#include <array>
#include <span>

#ifdef USE_ESP_IDF
#include "components/ramses_esp/ramses_esp.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include <esp_efuse.h>
#include <esp_mac.h>
#endif

namespace esphome {
namespace ramses_devices {

static const char *const TAG = "ramses_fan";

// ----------------------------------------------------------------
// Chip-ID derived address
// ----------------------------------------------------------------
ramses_esp::RamsesAddress RamsesFan::derive_remote_from_chip_id() const {
#ifdef USE_ESP_IDF
  uint8_t mac[6] = {};
  esp_efuse_mac_get_default(mac);
  // RAMSES device IDs are 18 bits; use the low 18 MAC bits for a stable ID
  uint32_t chip_id =
      ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
  chip_id &= 0x03FFFF;
  // Device class 37 (HVAC display switch / DIS) is the canonical virtual remote
  // class
  return ramses_esp::RamsesAddress{
      .dev_class = 37,
      .id = chip_id,
      .is_valid = true,
  };
#else
  // Native test builds: fall back to a deterministic fixture address
  return ramses_esp::RamsesAddress{
      .dev_class = 37,
      .id = 0x022034, // 37:140340
      .is_valid = true,
  };
#endif
}

// ----------------------------------------------------------------
// NVS Persistence
// ----------------------------------------------------------------
void RamsesFan::load_preferences() {
#ifdef USE_ESP_IDF
  // Hash is derived from the entity name so each fan instance has its own slot
  std::array<char, esphome::ENTITY_NAME_BUF_SIZE> name_buffer{};
  const std::string entity_name =
      this->get_name_to(std::span<char, esphome::ENTITY_NAME_BUF_SIZE>(
                            name_buffer))
          .str();
  uint32_t hash_dev = fnv1_hash(std::string("ramses_fan_dev_") + entity_name);
  uint32_t hash_rem = fnv1_hash(std::string("ramses_fan_rem_") + entity_name);
  this->pref_device_addr_ =
      global_preferences->make_preference<uint32_t>(hash_dev, true);
  this->pref_remote_addr_ =
      global_preferences->make_preference<uint32_t>(hash_rem, true);

  // Load device address from NVS (only when not overridden by YAML)
  uint32_t stored_dev = 0;
  if (this->pref_device_addr_.load(&stored_dev) && stored_dev != 0) {
    ramses_esp::RamsesAddress nvs_dev = u32_to_addr(stored_dev);
    if (!this->device_address_from_yaml_.is_valid) {
      this->device_address_ = nvs_dev;
      ESP_LOGI(TAG, "Loaded device address from NVS: %s",
               nvs_dev.to_string().c_str());
    } else if (!(this->device_address_from_yaml_ == nvs_dev)) {
      ESP_LOGW(TAG,
               "NVS device address (%s) differs from YAML (%s); using YAML",
               nvs_dev.to_string().c_str(),
               this->device_address_from_yaml_.to_string().c_str());
    }
  }

  // Load remote address from NVS (only when not overridden by YAML)
  uint32_t stored_rem = 0;
  if (this->pref_remote_addr_.load(&stored_rem) && stored_rem != 0) {
    ramses_esp::RamsesAddress nvs_rem = u32_to_addr(stored_rem);
    if (!this->remote_address_from_yaml_.is_valid) {
      this->remote_address_ = nvs_rem;
      ESP_LOGI(TAG, "Loaded remote address from NVS: %s",
               nvs_rem.to_string().c_str());
    } else if (!(this->remote_address_from_yaml_ == nvs_rem)) {
      ESP_LOGW(TAG,
               "NVS remote address (%s) differs from YAML (%s); using YAML",
               nvs_rem.to_string().c_str(),
               this->remote_address_from_yaml_.to_string().c_str());
    }
  }
#endif
}

void RamsesFan::save_device_address() {
#ifdef USE_ESP_IDF
  uint32_t v = addr_to_u32(this->device_address_);
  this->pref_device_addr_.save(&v);
  global_preferences->sync();
  ESP_LOGI(TAG, "Saved device address to NVS: %s",
           this->device_address_.to_string().c_str());
#endif
}

void RamsesFan::save_remote_address() {
#ifdef USE_ESP_IDF
  uint32_t v = addr_to_u32(this->remote_address_);
  this->pref_remote_addr_.save(&v);
  global_preferences->sync();
  ESP_LOGI(TAG, "Saved remote address to NVS: %s",
           this->remote_address_.to_string().c_str());
#endif
}

// ----------------------------------------------------------------
// Component lifecycle
// ----------------------------------------------------------------
void RamsesFan::setup() {
  this->set_supported_preset_modes(
      {"auto", "low", "medium", "high", "boost", "away"});

  // Ensure a stable remote address exists before we ever transmit
  if (!this->remote_address_.is_valid) {
    this->remote_address_ = this->derive_remote_from_chip_id();
    ESP_LOGI(TAG, "Derived virtual remote address from chip ID: %s",
             this->remote_address_.to_string().c_str());
  }

  // Load NVS state (may override the chip-ID address if a pairing was
  // previously completed)
  this->load_preferences();

  // Publish initial state so entity is immediately available in HA
  this->state = true;
  this->speed = 33;
  this->set_preset_mode_("low");
  this->publish_state();

  this->setup_base();
}

void RamsesFan::loop() {
  if (!this->pairing_active_)
    return;

  uint32_t now = millis();
  if (now - this->pairing_start_time_ >= this->pairing_timeout_ms_) {
    this->stop_pairing();
    ESP_LOGW(TAG, "Ventilation pairing timed out after %u ms",
             (unsigned int)this->pairing_timeout_ms_);
    return;
  }

  // Broadcast 1FC9 Offer every 1000 ms during pairing mode
  if (now - this->last_offer_time_ > 1000) {
    this->last_offer_time_ = now;
#ifdef USE_ESP_IDF
    if (this->parent_ != nullptr) {
      ramses_esp::RamsesAddress sender = this->get_effective_sender_address();
      ramses_esp::RamsesMessage offer =
          ramses_esp::BindingPayload::encode_offer(sender, this->scheme_);
      ESP_LOGI(TAG, "Broadcasting 1FC9 Pairing Offer from Remote %s...",
               sender.to_string().c_str());
      this->parent_->send_message(offer);
    }
#endif
  }
}

// ----------------------------------------------------------------
// Pairing control
// ----------------------------------------------------------------
void RamsesFan::start_pairing(uint32_t timeout_ms) {
  this->pairing_active_ = true;
  this->pairing_start_time_ = millis();
  this->pairing_timeout_ms_ = timeout_ms;
  this->last_offer_time_ = 0;

  ESP_LOGI(TAG,
           "Virtual remote pairing active for %u s; offers are sent every "
           "second.",
           (unsigned int)(timeout_ms / 1000));
  if (this->scheme_ == ramses_esp::HvacScheme::ORCON) {
    ESP_LOGI(TAG,
             "For an MVS-15RH, enter learn mode first: disconnect mains for "
             "5 s, reconnect, then start pairing within its 3-minute window.");
  }
}

void RamsesFan::stop_pairing() {
  this->pairing_active_ = false;
  ESP_LOGI(TAG, "Ventilation pairing mode ended.");
}

ramses_esp::RamsesAddress RamsesFan::get_effective_sender_address() const {
  if (this->remote_address_.is_valid) {
    return this->remote_address_;
  }
  return this->derive_remote_from_chip_id();
}

// ----------------------------------------------------------------
// Fan entity
// ----------------------------------------------------------------
fan::FanTraits RamsesFan::get_traits() {
  fan::FanTraits traits;
  traits.set_speed(true);
  traits.set_supported_speed_count(100);
  return traits;
}

void RamsesFan::handle_message(const ramses_esp::RamsesMessage &msg) {
  uint16_t opcode = ((uint16_t)msg.opcode[0] << 8) | msg.opcode[1];

  // Handle the respondent's 1FC9 Accept only when it targets this remote,
  // comes from the configured MVS (if supplied), and grants the fan status
  // bindings observed in real Orcon handshakes.
  if (this->pairing_active_ && opcode == 0x1FC9 &&
      msg.type == ramses_esp::RAMSES_MSG_W) {
    auto binding = ramses_esp::BindingPayload::decode(msg);
    if (!binding.has_value() || !binding->is_accept) {
      ESP_LOGW(TAG, "Ignoring malformed 1FC9 Accept during pairing");
      return;
    }

    ramses_esp::RamsesAddress target =
        ramses_esp::RamsesAddress::from_bytes(msg.addr[1]);
    ramses_esp::RamsesAddress sender = this->get_effective_sender_address();
    ramses_esp::RamsesAddress fan_addr =
        ramses_esp::RamsesAddress::from_bytes(msg.addr[0]);
    if (target != sender) {
      ESP_LOGD(TAG, "Ignoring 1FC9 Accept addressed to %s (our remote is %s)",
               target.to_string().c_str(), sender.to_string().c_str());
      return;
    }
    if (!fan_addr.is_valid ||
        (this->device_address_.is_valid &&
         fan_addr != this->device_address_)) {
      ESP_LOGW(TAG, "Ignoring 1FC9 Accept from unexpected fan %s",
               fan_addr.to_string().c_str());
      return;
    }

    bool accepts_31d9 = false;
    bool accepts_31da = false;
    for (const auto &item : binding->bindings) {
      accepts_31d9 |= item.opcode == 0x31D9;
      accepts_31da |= item.opcode == 0x31DA;
    }
    if (!accepts_31d9 || !accepts_31da) {
      ESP_LOGW(TAG,
               "Ignoring 1FC9 Accept from %s: expected 31D9 and 31DA bindings",
               fan_addr.to_string().c_str());
      return;
    }

#ifdef USE_ESP_IDF
    if (this->parent_ == nullptr) {
      ESP_LOGE(TAG, "Cannot confirm pairing: RAMSES gateway is unavailable");
      return;
    }
    ramses_esp::RamsesMessage confirm =
        ramses_esp::BindingPayload::encode_confirm(sender, fan_addr);
    if (!this->parent_->send_message(confirm)) {
      ESP_LOGW(TAG, "Could not queue 1FC9 Confirm; keeping pairing active");
      return;
    }

    // Persist addresses only after the confirm was accepted by the TX queue.
    if (!this->device_address_from_yaml_.is_valid) {
      this->device_address_ = fan_addr;
      this->save_device_address();
    }
    this->save_remote_address();

    // Orcon DIS remotes announce their 10E0 identity after binding. The same
    // identity is also returned if the FAN explicitly queries the remote.
    if (this->scheme_ == ramses_esp::HvacScheme::ORCON) {
      const auto all_devices = ramses_esp::RamsesAddress{
          .dev_class = 63, .id = 0x03FFFE, .is_valid = true};
      auto identity = ramses_esp::DeviceInfoPayload::encode_orcon_dis_info(
          sender, all_devices);
      if (!this->parent_->send_message(identity)) {
        ESP_LOGW(TAG, "Could not queue Orcon DIS 10E0 identity announcement");
      }
    }
#endif
    this->stop_pairing();
    ESP_LOGI(TAG,
             "Pairing confirmed with Fan Unit %s; remote %s is now bound.",
             fan_addr.to_string().c_str(), sender.to_string().c_str());
    return;
  }

  // Respond to the FAN's 10E0 query as an Orcon DIS virtual remote.
  if (this->scheme_ == ramses_esp::HvacScheme::ORCON && opcode == 0x10E0 &&
      msg.type == ramses_esp::RAMSES_MSG_RQ &&
      (msg.fields & RAMSES_F_ADDR1)) {
    auto target = ramses_esp::RamsesAddress::from_bytes(msg.addr[1]);
    auto sender = this->get_effective_sender_address();
    if (target == sender && this->parent_ != nullptr) {
      auto fan_addr = ramses_esp::RamsesAddress::from_bytes(msg.addr[0]);
      auto identity = ramses_esp::DeviceInfoPayload::encode_orcon_dis_info(
          sender, fan_addr, true);
      if (!this->parent_->send_message(identity)) {
        ESP_LOGW(TAG, "Could not queue response to FAN 10E0 query");
      } else {
        ESP_LOGD(TAG, "Answered 10E0 query from %s", fan_addr.to_string().c_str());
      }
      return;
    }
  }

  // 22F1 commands are sent by the *remote* to the fan unit, so we accept them
  // from either the configured device address or the remote address.
  bool from_device = address_matches(this->device_address_, msg);
  bool from_remote = address_matches(this->remote_address_, msg);

  if (opcode == 0x22F1 && (from_device || from_remote)) {
    auto dec = ramses_esp::FanStatePayload::decode(msg.payload, msg.n_payload,
                                                   this->scheme_);
    if (dec.has_value()) {
      this->set_preset_mode_(
          ramses_esp::fan_preset_to_string(dec->preset_mode));
      this->state = dec->preset_mode != ramses_esp::FanPresetMode::OFF;
      this->speed = dec->speed_percent;
      this->publish_state();
    }
  } else if (!from_device) {
    return;
  } else if (opcode == 0x31D9) {
    // The MVS reports its confirmed mode in 31D9. Use the configured vendor
    // mapping (Orcon for MVS-15RH) so Home Assistant reflects actual feedback.
    auto dec = ramses_esp::HvacFanInfoPayload::decode(
        msg.payload, msg.n_payload, this->scheme_);
    if (dec.has_value() &&
        dec->preset_mode != ramses_esp::FanPresetMode::UNKNOWN) {
      this->set_preset_mode_(
          ramses_esp::fan_preset_to_string(dec->preset_mode));
      this->state = dec->preset_mode != ramses_esp::FanPresetMode::OFF;
      if (dec->fan_speed_percent.has_value())
        this->speed = static_cast<int>(*dec->fan_speed_percent);
      this->publish_state();
    }
  } else if (opcode == 0x31DA) {
    // 31DA reports motor feedback / actual airflow (not commanded preset).
    // Keep the selected preset from the 31D9 status frame; detailed telemetry
    // is published by RamsesSensor entities sharing this subscription.
  }
}

void RamsesFan::control(const fan::FanCall &call) {
  ramses_esp::FanPresetMode target_mode = ramses_esp::FanPresetMode::AUTO;

  if (call.has_preset_mode()) {
    std::string pm = call.get_preset_mode();
    std::transform(pm.begin(), pm.end(), pm.begin(), ::tolower);
    if (pm == "low")
      target_mode = ramses_esp::FanPresetMode::LOW;
    else if (pm == "medium" || pm == "med")
      target_mode = ramses_esp::FanPresetMode::MEDIUM;
    else if (pm == "high")
      target_mode = ramses_esp::FanPresetMode::HIGH;
    else if (pm == "boost")
      target_mode = ramses_esp::FanPresetMode::BOOST;
    else if (pm == "away")
      target_mode = ramses_esp::FanPresetMode::AWAY;
    else if (pm == "auto")
      target_mode = ramses_esp::FanPresetMode::AUTO;
    else if (pm == "off")
      target_mode = ramses_esp::FanPresetMode::OFF;
  } else if (call.get_speed().has_value()) {
    int sp = *call.get_speed();
    if (sp == 0)
      target_mode = ramses_esp::FanPresetMode::OFF;
    else if (sp <= 33)
      target_mode = ramses_esp::FanPresetMode::LOW;
    else if (sp <= 66)
      target_mode = ramses_esp::FanPresetMode::MEDIUM;
    else
      target_mode = ramses_esp::FanPresetMode::HIGH;
  } else if (call.get_state().has_value()) {
    if (!*call.get_state()) {
      target_mode = ramses_esp::FanPresetMode::OFF;
    } else {
      target_mode = ramses_esp::FanPresetMode::LOW;
    }
  }

  this->set_preset_mode_(ramses_esp::fan_preset_to_string(target_mode));
  this->state = (target_mode != ramses_esp::FanPresetMode::OFF);
  if (target_mode == ramses_esp::FanPresetMode::LOW)
    this->speed = 33;
  else if (target_mode == ramses_esp::FanPresetMode::MEDIUM)
    this->speed = 66;
  else if (target_mode == ramses_esp::FanPresetMode::HIGH ||
           target_mode == ramses_esp::FanPresetMode::BOOST)
    this->speed = 100;
  else if (target_mode == ramses_esp::FanPresetMode::OFF)
    this->speed = 0;

  ramses_esp::RamsesAddress sender = this->get_effective_sender_address();
  ramses_esp::RamsesMessage msg = ramses_esp::FanStatePayload::encode_write(
      sender, this->device_address_, target_mode, this->scheme_);

#ifdef USE_ESP_IDF
  if (this->parent_ != nullptr) {
    this->parent_->send_message(msg);
  }
#endif
  this->publish_state();
}

} // namespace ramses_devices
} // namespace esphome

#endif // USE_FAN
