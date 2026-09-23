#include "imu_sensor.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "IMU_SENSOR";

#define I2C_PORT            I2C_NUM_0
#define I2C_SDA_PIN         GPIO_NUM_32
#define I2C_SCL_PIN         GPIO_NUM_33
#define I2C_FREQ_HZ         100000

#define MPU6050_ADDR        0x68

#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75
#define REG_ACCEL_XOUT_H    0x3B

// Varsayilan hassasiyet: ivme +/-2g, jiroskop +/-250 dps
#define ACCEL_SCALE         (9.80665f / 16384.0f)
#define GYRO_SCALE          ((float)(M_PI / 180.0) / 131.0f)

// MPU6050 datasheet: Temp_degC = raw / 340 + 36.53
#define TEMP_SCALE          (1.0f / 340.0f)
#define TEMP_OFFSET         36.53f

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev = NULL;

#define I2C_XFER_TIMEOUT_MS 100

static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, sizeof(buf), I2C_XFER_TIMEOUT_MS);
}

static esp_err_t mpu6050_read_regs(uint8_t reg, uint8_t *out, size_t len) {
    uint8_t reg_addr = reg;
    return i2c_master_transmit_receive(s_dev, &reg_addr, 1, out, len, I2C_XFER_TIMEOUT_MS);
}

esp_err_t imu_sensor_init(void) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_config, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C veri yolu olusturulamadi: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "MPU6050 baslatilamadi, IMU telemetrisi devre disi kalacak.");
        return err;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    err = i2c_master_bus_add_device(s_bus, &dev_config, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 veri yoluna eklenemedi: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "MPU6050 baslatilamadi, IMU telemetrisi devre disi kalacak.");
        return err;
    }

    uint8_t who_am_i = 0;
    err = mpu6050_read_regs(REG_WHO_AM_I, &who_am_i, 1);
    if (err != ESP_OK || who_am_i != MPU6050_ADDR) {
        ESP_LOGE(TAG, "MPU6050 bulunamadi (WHO_AM_I=0x%02X, err=%s)", who_am_i, esp_err_to_name(err));
        ESP_LOGE(TAG, "MPU6050 baslatilamadi, IMU telemetrisi devre disi kalacak.");
        return ESP_FAIL;
    }

    // 1. Sensore donanimsal reset atip register'lari fabrika ayarina cek
    ESP_ERROR_CHECK(mpu6050_write_reg(REG_PWR_MGMT_1, 0x80));
    vTaskDelay(pdMS_TO_TICKS(100));

    // 2. Uyku modundan cikar ve Gyro PLL saat kaynagina baglan (0x01)
    ESP_ERROR_CHECK(mpu6050_write_reg(REG_PWR_MGMT_1, 0x01));
    vTaskDelay(pdMS_TO_TICKS(50));

    // 3. Titresimleri engellemek icin DLPF: ~21Hz bant genisligi (0x04)
    ESP_ERROR_CHECK(mpu6050_write_reg(REG_CONFIG, 0x04));

    // 4. Ornekleme hizi: 1kHz / (1 + 9) = 100Hz
    ESP_ERROR_CHECK(mpu6050_write_reg(REG_SMPLRT_DIV, 0x09));

    // 5. Jiroskop: +/-250 dps, Self-Test kapali (0x00)
    ESP_ERROR_CHECK(mpu6050_write_reg(REG_GYRO_CONFIG, 0x00));

    // 6. Ivmeolcer: +/-2g, Self-Test zorla kapali (0x00)
    ESP_ERROR_CHECK(mpu6050_write_reg(REG_ACCEL_CONFIG, 0x00));

    ESP_LOGI(TAG, "MPU6050 basariyla sifirlandi ve hazirlandi.");
    return ESP_OK;
}

esp_err_t imu_sensor_read(imu_payload_t *out) {
    uint8_t raw[14] = {0};
    esp_err_t err = mpu6050_read_regs(REG_ACCEL_XOUT_H, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    int16_t ax_raw   = (int16_t)((raw[0] << 8)  | raw[1]);
    int16_t ay_raw   = (int16_t)((raw[2] << 8)  | raw[3]);
    int16_t az_raw   = (int16_t)((raw[4] << 8)  | raw[5]);
    int16_t temp_raw = (int16_t)((raw[6] << 8)  | raw[7]);
    int16_t gx_raw   = (int16_t)((raw[8] << 8)  | raw[9]);
    int16_t gy_raw   = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz_raw   = (int16_t)((raw[12] << 8) | raw[13]);

    out->ax   = ax_raw   * ACCEL_SCALE;
    out->ay   = ay_raw   * ACCEL_SCALE;
    out->az   = az_raw   * ACCEL_SCALE;
    out->gx   = gx_raw   * GYRO_SCALE;
    out->gy   = gy_raw   * GYRO_SCALE;
    out->gz   = gz_raw   * GYRO_SCALE;
    out->temp = temp_raw * TEMP_SCALE + TEMP_OFFSET;

    return ESP_OK;
}