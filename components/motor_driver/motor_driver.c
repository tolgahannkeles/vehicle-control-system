#include "motor_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <math.h>
#include <stdlib.h>

static const char *TAG = "MOTOR_DRIVER";

#define MOTOR_L_ENA     18
#define MOTOR_L_IN1     19
#define MOTOR_L_IN2     21

#define MOTOR_R_ENB     22
#define MOTOR_R_IN3     23
#define MOTOR_R_IN4     25

#define PWM_TIMER       LEDC_TIMER_0
#define PWM_MODE        LEDC_LOW_SPEED_MODE
#define PWM_CHANNEL_L   LEDC_CHANNEL_0
#define PWM_CHANNEL_R   LEDC_CHANNEL_1
#define PWM_DUTY_RES    LEDC_TIMER_10_BIT // 0 - 1023
#define PWM_FREQ        5000
#define MAX_DUTY        1023

#define SMOOTHING_ALPHA 0.08f
#define ROBOT_MAX_MPS   0.60f // Maksimum çizgisel palet hızı (m/s)

static volatile float s_target_left = 0.0f;
static volatile float s_target_right = 0.0f;
static float s_filtered_left = 0.0f;
static float s_filtered_right = 0.0f;

static void apply_hardware_pwm(int left_speed, int right_speed) {
    // Ölü bölge (Deadband)
    if (abs(left_speed) < 25) left_speed = 0;
    if (abs(right_speed) < 25) right_speed = 0;

    // --- Sol Palet ---
    uint32_t left_mag = (uint32_t)abs(left_speed);
    if (left_mag > 1000) left_mag = 1000;
    uint32_t left_duty = (left_mag * MAX_DUTY) / 1000;

    if (left_speed > 0) {
        gpio_set_level(MOTOR_L_IN1, 1);
        gpio_set_level(MOTOR_L_IN2, 0);
    } else if (left_speed < 0) {
        gpio_set_level(MOTOR_L_IN1, 0);
        gpio_set_level(MOTOR_L_IN2, 1);
    } else {
        gpio_set_level(MOTOR_L_IN1, 0);
        gpio_set_level(MOTOR_L_IN2, 0);
    }
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_L, left_duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_L);

    // --- Sağ Palet ---
    uint32_t right_mag = (uint32_t)abs(right_speed);
    if (right_mag > 1000) right_mag = 1000;
    uint32_t right_duty = (right_mag * MAX_DUTY) / 1000;

    if (right_speed > 0) {
        gpio_set_level(MOTOR_R_IN3, 1);
        gpio_set_level(MOTOR_R_IN4, 0);
    } else if (right_speed < 0) {
        gpio_set_level(MOTOR_R_IN3, 0);
        gpio_set_level(MOTOR_R_IN4, 1);
    } else {
        gpio_set_level(MOTOR_R_IN3, 0);
        gpio_set_level(MOTOR_R_IN4, 0);
    }
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_R, right_duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_R);
}

static void motor_filter_task(void *pvParameters) {
    while (1) {
        s_filtered_left  += SMOOTHING_ALPHA * (s_target_left - s_filtered_left);
        s_filtered_right += SMOOTHING_ALPHA * (s_target_right - s_filtered_right);

        apply_hardware_pwm((int)roundf(s_filtered_left), (int)roundf(s_filtered_right));
        vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz döngü
    }
}

void motor_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_L_IN1) | (1ULL << MOTOR_L_IN2) |
                        (1ULL << MOTOR_R_IN3) | (1ULL << MOTOR_R_IN4),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = PWM_MODE,
        .timer_num        = PWM_TIMER,
        .duty_resolution  = PWM_DUTY_RES,
        .freq_hz          = PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ch_l = {
        .speed_mode = PWM_MODE,
        .channel    = PWM_CHANNEL_L,
        .timer_sel  = PWM_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = MOTOR_L_ENA,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&ch_l);

    ledc_channel_config_t ch_r = {
        .speed_mode = PWM_MODE,
        .channel    = PWM_CHANNEL_R,
        .timer_sel  = PWM_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = MOTOR_R_ENB,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&ch_r);

    motor_stop();
    xTaskCreatePinnedToCore(motor_filter_task, "motor_filter", 2048, NULL, 5, NULL, 1);
    ESP_LOGI(TAG, "Motor donanimi ve yumusatma filtresi hazir.");
}

void motor_set_targets(float left_mps, float right_mps) {
    // EĞER SAĞ VE SOL TERSSE DOĞRUDAN BURADA DÜZELTİYORUZ:
    float norm_l = (left_mps / ROBOT_MAX_MPS) * 1000.0f;
    float norm_r = (right_mps / ROBOT_MAX_MPS) * 1000.0f;

    // Sınırlandırmalar
    if (norm_l > 1000.0f) norm_l = 1000.0f;
    if (norm_l < -1000.0f) norm_l = -1000.0f;
    if (norm_r > 1000.0f) norm_r = 1000.0f;
    if (norm_r < -1000.0f) norm_r = -1000.0f;

    // BURAYI DÜZELTTİK: left komutu sağ değişkene gidiyordu, tersini yapıyoruz:
    s_target_left = norm_r;  // ya da tam tersi: hangisi yanlışsa çaprazla
    s_target_right = norm_l;
}

void motor_stop(void) {
    s_target_left = 0.0f;
    s_target_right = 0.0f;
}

// Web UI'dan gelen normalize throttle ve steer değerlerini doğrudan hedefe yazar
void motor_update(int throttle, int steer) {
    int left = throttle - steer;
    int right = throttle + steer;

    if (left > 1000) left = 1000;
    if (left < -1000) left = -1000;
    if (right > 1000) right = 1000;
    if (right < -1000) right = -1000;

    s_target_left = (float)left;
    s_target_right = (float)right;
}
