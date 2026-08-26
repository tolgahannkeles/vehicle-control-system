#pragma once

#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief OTA güncelleme oturumunu başlatır, hedef bölümü ayarlar.
 */
esp_err_t ota_manager_begin(void);

/**
 * @brief Gelen ham ikili (binary) parçayı flash'a yazar.
 * 
 * @param data Byte dizisi
 * @param length Yazılacak byte uzunluğu
 */
esp_err_t ota_manager_write(const void *data, size_t length);

/**
 * @brief OTA yazma işlemini doğrular, tamamlar ve boot bölümünü günceller.
 * 
 * @param auto_restart true ise cihazı hemen yeniden başlatır
 */
esp_err_t ota_manager_finish(bool auto_restart);

/**
 * @brief Hata durumunda OTA oturumunu iptal eder ve temizler.
 */
void ota_manager_abort(void);