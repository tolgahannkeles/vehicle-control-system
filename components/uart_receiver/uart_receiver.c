#include "uart_receiver.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "motor_driver.h"

static const char *TAG = "UART_RX";

#define UART_PORT_NUM      UART_NUM_2
#define UART_TX_PIN        17
#define UART_RX_PIN        16
#define UART_BAUD_RATE     115200
#define BUF_SIZE           512

static void uart_rx_task(void *pvParameters) {
    uint8_t data[BUF_SIZE];
    char line_buf[64];
    int line_idx = 0;

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE - 1, pdMS_TO_TICKS(20));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = (char)data[i];
                if (c == '\n' || c == '\r') {
                    if (line_idx > 0) {
                        line_buf[line_idx] = '\0';
                        int throttle = 0, steer = 0;
                        // Format: "<throttle,steer>" Örn: "<400,-150>"
                        if (sscanf(line_buf, "<%d,%d>", &throttle, &steer) == 2) {
                            motor_update(throttle, steer);
                        }
                        line_idx = 0;
                    }
                } else {
                    if (line_idx < sizeof(line_buf) - 1) {
                        line_buf[line_idx++] = c;
                    } else {
                        line_idx = 0;
                    }
                }
            }
        }
    }
}

void uart_receiver_init(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreate(uart_rx_task, "uart_rx_task", 3072, NULL, 5, NULL);
    ESP_LOGI(TAG, "UART2 dinleme baslatildi (RX: GPIO 16, TX: GPIO 17, %d Baud).", UART_BAUD_RATE);
}