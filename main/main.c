#include "nvs_flash.h"
#include "motor_driver.h"
#include "wifi_ap.h"
#include "web_ui.h"
#include "uart_receiver.h"

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    motor_init();
    wifi_init_sta();       // Web Arayüzü ve OTA yine çalışmaya devam eder
    start_webserver();
    uart_receiver_init();  // Raspberry Pi'dan gelen komutları dinler
}