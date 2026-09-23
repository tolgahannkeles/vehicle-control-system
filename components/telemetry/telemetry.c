#include "telemetry.h"
#include "serial_bridge.h"
#include "protocol.h"
#include "imu_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "TELEMETRY";

void telemetry_task(void *pvParameters) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period_20ms = pdMS_TO_TICKS(20); // 50 Hz
    uint32_t count = 0;

    double dummy_lat = 39.92077;
    double dummy_lon = 32.85411;

    while (1) {
        // 50 Hz IMU Telemetrisi
        imu_payload_t imu_data;
        if (imu_sensor_read(&imu_data) == ESP_OK) {
            serial_bridge_send_packet(PKT_ID_IMU, (const uint8_t *)&imu_data, sizeof(imu_data));
        }

        // 1 Hz GPS Telemetrisi
        if (++count >= 50) {
            count = 0;
            dummy_lat += 0.00002;
            dummy_lon += 0.00002;

            gps_payload_t gps_data = {
                .lat = dummy_lat,
                .lon = dummy_lon,
                .fix = 1
            };
            serial_bridge_send_packet(PKT_ID_GPS, (const uint8_t *)&gps_data, sizeof(gps_data));
        }

        vTaskDelayUntil(&last_wake, period_20ms);
    }
}

esp_err_t telemetry_init(void) {
    BaseType_t ret = xTaskCreatePinnedToCore(
        telemetry_task,
        "telemetry_task",
        3072,
        NULL,
        5,
        NULL,
        1 // Core 1
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Telemetry gorevi olusturulamadi!");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Telemetry modulu baslatildi.");
    return ESP_OK;
}