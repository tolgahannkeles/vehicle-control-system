#include "ota_manager.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "OTA_MANAGER";

static esp_ota_handle_t s_ota_handle = 0;
static const esp_partition_t *s_update_partition = NULL;
static bool s_is_ongoing = false;

// HTTP yanıtı istemciye ulaştıktan sonra güvenli yeniden başlatma görevi
static void delayed_restart_task(void *pvParameter) {
    vTaskDelay(pdMS_TO_TICKS(1500)); // Yanıtın tarayıcıya gitmesi için 1.5 sn bekle
    ESP_LOGI(TAG, "Yeniden baslatiliyor...");
    esp_restart();
}

esp_err_t ota_manager_begin(void) {
    s_update_partition = esp_ota_get_next_update_partition(NULL);
    if (!s_update_partition) {
        ESP_LOGE(TAG, "Gecerli bir OTA hedef bolumu bulunamadi!");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "OTA Baslatiliyor: %s (Adres: 0x%lx)", 
             s_update_partition->label, s_update_partition->address);

    esp_err_t err = esp_ota_begin(s_update_partition, OTA_WITH_SEQUENTIAL_WRITES, &s_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin basarisiz: 0x%x", err);
        s_is_ongoing = false;
        return err;
    }

    s_is_ongoing = true;
    return ESP_OK;
}

esp_err_t ota_manager_write(const void *data, size_t length) {
    if (!s_is_ongoing) {
        ESP_LOGE(TAG, "OTA oturumu acilmadan yazma yapilamaz!");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = esp_ota_write(s_ota_handle, data, length);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA yazma hatasi: 0x%x", err);
        ota_manager_abort();
        return err;
    }

    return ESP_OK;
}

esp_err_t ota_manager_finish(bool auto_restart) {
    if (!s_is_ongoing) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = esp_ota_end(s_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end dogrulama hatasi: 0x%x", err);
        s_is_ongoing = false;
        return err;
    }

    err = esp_ota_set_boot_partition(s_update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Yeni boot bolumu secilemedi: 0x%x", err);
        s_is_ongoing = false;
        return err;
    }

    s_is_ongoing = false;
    ESP_LOGI(TAG, "OTA basariyla tamamlandi ve dogrulandi!");

    if (auto_restart) {
        xTaskCreate(delayed_restart_task, "delayed_restart", 2048, NULL, 5, NULL);
    }

    return ESP_OK;
}

void ota_manager_abort(void) {
    if (s_is_ongoing) {
        esp_ota_abort(s_ota_handle);
        s_is_ongoing = false;
        ESP_LOGW(TAG, "OTA oturumu iptal edildi.");
    }
}