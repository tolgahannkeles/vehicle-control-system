#include "motor_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <math.h>

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
#define PWM_DUTY_RES    LEDC_TIMER_10_BIT
#define PWM_FREQ        5000
#define MAX_DUTY        1023

#define SMOOTHING_ALPHA 0.08f

static volatile float target_left = 0.0f;
static volatile float target_right = 0.0f;
static float filtered_left = 0.0f;
static float filtered_right = 0.0f;

static void apply_hardware_pwm(int left_speed, int right_speed) {
    if (abs(left_speed) < 20) left_speed = 0;
    if (abs(right_speed) < 20) right_speed = 0;

    // Sol Motor Sürüşü
    if (left_speed >= 0) {
        gpio_set_level(MOTOR_L_IN1, 1);
        gpio_set_level(MOTOR_L_IN2, 0);
    } else {
        gpio_set_level(MOTOR_L_IN1, 0);
        gpio_set_level(MOTOR_L_IN2, 1);
        left_speed = -left_speed;
    }
    uint32_t left_duty = (uint32_t)((left_speed * MAX_DUTY) / 1000);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_L, left_duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_L);

    // Sağ Motor Sürüşü
    if (right_speed >= 0) {
        gpio_set_level(MOTOR_R_IN3, 1);
        gpio_set_level(MOTOR_R_IN4, 0);
    } else {
        gpio_set_level(MOTOR_R_IN3, 0);
        gpio_set_level(MOTOR_R_IN4, 1);
        right_speed = -right_speed;
    }
    uint32_t right_duty = (uint32_t)((right_speed * MAX_DUTY) / 1000);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_R, right_duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_R);
}

static void motor_filter_task(void *pvParameters) {
    while (1) {
        filtered_left  += SMOOTHING_ALPHA * (target_left - filtered_left);
        filtered_right += SMOOTHING_ALPHA * (target_right - filtered_right);

        apply_hardware_pwm((int)roundf(filtered_left), (int)roundf(filtered_right));

        vTaskDelay(pdMS_TO_TICKS(20));
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

    xTaskCreate(motor_filter_task, "motor_filter_task", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "Motor surucu hazir.");
}

void motor_update(int throttle, int steer) {
    // Sağ ve Sol palet atamaları ters çevrildi (Yer değişimi sağlandı)
    int left = throttle - steer;
    int right = throttle + steer;

    if (left > 1000) left = 1000;
    if (left < -1000) left = -1000;
    if (right > 1000) right = 1000;
    if (right < -1000) right = -1000;

    target_left = (float)left;
    target_right = (float)right;
}

void motor_stop(void) {
    target_left = 0.0f;
    target_right = 0.0f;
}