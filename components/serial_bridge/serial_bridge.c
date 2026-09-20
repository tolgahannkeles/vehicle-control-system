#include "serial_bridge.h"
#include "protocol.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "SERIAL_BRIDGE";

#define UART_PORT         UART_NUM_2
#define UART_BAUD_RATE    115200
#define UART_RX_BUF_SIZE  1024
#define UART_TX_PIN       GPIO_NUM_17
#define UART_RX_PIN       GPIO_NUM_16

typedef struct {
    uint8_t data[64];
    uint8_t len;
} tx_packet_t;

static QueueHandle_t s_tx_queue = NULL;
static serial_rx_callback_t s_rx_callback = NULL;

bool serial_bridge_send_packet(uint8_t pkt_id, const uint8_t *payload, uint8_t payload_len) {
    if (s_tx_queue == NULL || (payload_len + 5) > sizeof(((tx_packet_t *)0)->data)) {
        return false;
    }

    tx_packet_t pkt;
    pkt.data[0] = FRAME_HEADER1;
    pkt.data[1] = FRAME_HEADER2;
    pkt.data[2] = pkt_id;
    pkt.data[3] = payload_len;
    if (payload && payload_len > 0) {
        memcpy(&pkt.data[4], payload, payload_len);
    }
    pkt.data[4 + payload_len] = protocol_calc_crc(pkt_id, payload_len, payload);
    pkt.len = payload_len + 5;

    return (xQueueSend(s_tx_queue, &pkt, 0) == pdTRUE);
}

static void uart_tx_task(void *pvParameters) {
    tx_packet_t pkt;
    while (1) {
        if (xQueueReceive(s_tx_queue, &pkt, portMAX_DELAY) == pdTRUE) {
            uart_write_bytes(UART_PORT, (const char *)pkt.data, pkt.len);
        }
    }
}

static void uart_rx_task(void *pvParameters) {
    protocol_parser_t parser;
    protocol_parser_init(&parser);

    uint8_t byte, id, len;
    uint8_t payload[64];

    while (1) {
        if (uart_read_bytes(UART_PORT, &byte, 1, portMAX_DELAY) > 0) {
            if (protocol_parse_byte(&parser, byte, &id, payload, &len)) {
                // Paket çözüldü, ne olduğuna bakmaksızın üst katmana pasla
                if (s_rx_callback) {
                    s_rx_callback(id, payload, len);
                }
            }
        }
    }
}

esp_err_t serial_bridge_init(serial_rx_callback_t rx_cb) {
    s_rx_callback = rx_cb;

    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_RX_BUF_SIZE, 0, 0, NULL, 0));

    s_tx_queue = xQueueCreate(16, sizeof(tx_packet_t));

    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx", 3072, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(uart_tx_task, "uart_tx", 3072, NULL, 9, NULL, 0);

    ESP_LOGI(TAG, "Serial Bridge hazır (Soyut taşıma katmanı).");
    return ESP_OK;
}