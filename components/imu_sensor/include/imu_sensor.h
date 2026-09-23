#pragma once

#include "esp_err.h"
#include "protocol.h"

// I2C veri yolunu baslatir ve MPU6050'yi uyandirip yapilandirir.
esp_err_t imu_sensor_init(void);

// MPU6050'den anlik ivme (m/s^2) ve aci hizi (rad/s) degerlerini okur.
esp_err_t imu_sensor_read(imu_payload_t *out);
