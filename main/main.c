#include "nvs_flash.h"
#include "esp_log.h"
#include <string.h>

#include "motor_driver.h"
#include "serial_bridge.h"
#include "telemetry.h"
#include "imu_sensor.h"
#include "wifi_ap.h"   // wifi_init_sta() burada tanımlı
#include "web_ui.h"
#include "protocol.h"

static const char *TAG = "MAIN";


static void on_serial_packet_received(uint8_t pkt_id, const uint8_t *payload, uint8_t len) {
    if (pkt_id == PKT_ID_CMD_VEL && len == sizeof(cmd_vel_payload_t)) {
        cmd_vel_payload_t cmd;
        memcpy(&cmd, payload, sizeof(cmd));
        motor_set_targets(cmd.v_left, cmd.v_right);
    }
}


void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    motor_init();
    serial_bridge_init(on_serial_packet_received);

    imu_sensor_init();
    telemetry_init();

    wifi_init_sta();
    start_webserver();

    ESP_LOGI(TAG, "Tum sistem basariyla ayaga kaldirildi.");
}