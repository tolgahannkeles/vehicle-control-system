#include "nvs_flash.h"
#include "motor_driver.h"
#include "wifi_ap.h"
#include "web_ui.h"

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    motor_init();
    wifi_init_sta();       // Modeme bağlanana kadar bekler ve IP alır
    start_webserver();     // Web sunucusunu başlatır
}