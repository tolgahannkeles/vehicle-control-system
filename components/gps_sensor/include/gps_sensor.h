#pragma once

#include "esp_err.h"
#include "protocol.h"
#include <stdbool.h>
#include "soc/gpio_num.h"

// GPS UART donanım pinleri (NEO-7M için)
#define GPS_UART_NUM        UART_NUM_1
#define GPS_TX_PIN          GPIO_NUM_5 // ESP32 TX -> NEO-7M RX
#define GPS_RX_PIN          GPIO_NUM_4 // ESP32 RX <- NEO-7M TX
#define GPS_BAUD_RATE       9600

/**
 * @brief GPS modülünü (UART2) ilklendirir.
 */
esp_err_t gps_sensor_init(void);

/**
 * @brief UART hattından NMEA cümlelerini okur, ayrıştırır ve veriyi doldurur.
 * 
 * @param out Doldurulacak GPS verisi yapısı
 * @return true Yeni ve geçerli bir paket başarıyla ayrıştırıldıysa
 * @return false Henüz tam veya yeni bir paket oluşmadıysa
 */
bool gps_sensor_read(gps_payload_t *out);