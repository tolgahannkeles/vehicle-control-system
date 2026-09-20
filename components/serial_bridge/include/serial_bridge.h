#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// Callback tipi: Bir paket çözüldüğünde üst katmana haber vermek için
typedef void (*serial_rx_callback_t)(uint8_t pkt_id, const uint8_t *payload, uint8_t len);

// Başlatma ve callback kaydı
esp_err_t serial_bridge_init(serial_rx_callback_t rx_cb);

// Genel amaçlı paket gönderme (Payload'un ne olduğunu bilmez, sadece taşır)
bool serial_bridge_send_packet(uint8_t pkt_id, const uint8_t *payload, uint8_t payload_len);