#include "gps_sensor.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "GPS_SENSOR";

#define GPS_BUF_SIZE        1024
#define NMEA_LINE_MAX       128

static gps_payload_t s_last_gps_data = {0};
static char s_line_buf[NMEA_LINE_MAX];
static size_t s_line_idx = 0;

// NMEA formatını (DDMM.MMMM) Ondalık Dereceye (DD.DDDDDD) çevirir
static double nmea_to_decimal(const char *nmea_coord, char hemisphere) {
    if (!nmea_coord || strlen(nmea_coord) < 4) return 0.0;

    double raw = atof(nmea_coord);
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100);
    double dec = degrees + (minutes / 60.0);

    if (hemisphere == 'S' || hemisphere == 'W') {
        dec = -dec;
    }
    return dec;
}

// NMEA satırını tokenlara ayırır (boş alanları atlamadan korur)
static char *nmea_next_token(char **cursor) {
    if (*cursor == NULL) return NULL;
    char *token = *cursor;
    char *comma = strchr(*cursor, ',');
    if (comma) {
        *comma = '\0';
        *cursor = comma + 1;
    } else {
        *cursor = NULL;
    }
    return token;
}

// GGA Cümlesini Ayrıştırır ($--GGA: Fix, Enlem, Boylam, İrtifa, HDOP, Uydu Sayısı)
static bool parse_gga(char *line) {
    char *cursor = line;
    char *fields[15] = {0};
    int i = 0;

    while (cursor && i < 15) {
        fields[i++] = nmea_next_token(&cursor);
    }

    if (i < 10) return false;

    // fields[6]: Fix kalitesi (0=Yok, 1=GPS, 2=DGPS)
    uint8_t fix = fields[6] ? (uint8_t)atoi(fields[6]) : 0;
    s_last_gps_data.fix = fix;

    if (fix > 0) {
        // Enlem
        if (fields[2] && fields[3]) {
            s_last_gps_data.lat = nmea_to_decimal(fields[2], fields[3][0]);
        }
        // Boylam
        if (fields[4] && fields[5]) {
            s_last_gps_data.lon = nmea_to_decimal(fields[4], fields[5][0]);
        }
        // Uydu Sayısı
        if (fields[7]) {
            s_last_gps_data.sats = (uint8_t)atoi(fields[7]);
        }
        // HDOP
        if (fields[8]) {
            s_last_gps_data.hdop = (float)atof(fields[8]);
        }
        // İrtifa (Metre)
        if (fields[9]) {
            s_last_gps_data.alt = (float)atof(fields[9]);
        }
        return true;
    }
    return false;
}

// RMC Cümlesini Ayrıştırır ($--RMC: Hız ve Rota)
static void parse_rmc(char *line) {
    char *cursor = line;
    char *fields[13] = {0};
    int i = 0;

    while (cursor && i < 13) {
        fields[i++] = nmea_next_token(&cursor);
    }

    if (i < 9) return;

    // fields[2]: 'A' = Valid, 'V' = Warning
    if (fields[2] && fields[2][0] == 'A') {
        // Hız (Knot -> m/s çevrimi: 1 knot = 0.514444 m/s)
        if (fields[7] && strlen(fields[7]) > 0) {
            s_last_gps_data.speed = (float)(atof(fields[7]) * 0.514444);
        }
        // Rota / Course (Derece)
        if (fields[8] && strlen(fields[8]) > 0) {
            s_last_gps_data.course = (float)atof(fields[8]);
        }
    }
}

esp_err_t gps_sensor_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(GPS_UART_NUM, GPS_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART2 surucusu kurulamadi: %s", esp_err_to_name(err));
        return err;
    }

    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "GPS UART2 baslatildi (RX: %d, TX: %d, Baud: %d)", GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD_RATE);
    return ESP_OK;
}

bool gps_sensor_read(gps_payload_t *out) {
    uint8_t ch;
    bool has_new_fix_packet = false;

    // UART kuyruğundaki baytları satır satır tüket
    while (uart_read_bytes(GPS_UART_NUM, &ch, 1, 0) > 0) {
        if (ch == '\n' || ch == '\r') {
            if (s_line_idx > 6) { // En azından bir $GPxxx başlığı olmalı
                s_line_buf[s_line_idx] = '\0';

                // NMEA Checksum kontrolü istenirse eklenebilir, genelde $GP veya $GN ile başlar
                if (strstr(s_line_buf, "GGA") != NULL) {
                    if (parse_gga(s_line_buf)) {
                        has_new_fix_packet = true;
                    }
                } else if (strstr(s_line_buf, "RMC") != NULL) {
                    parse_rmc(s_line_buf);
                }
            }
            s_line_idx = 0; // Satırı sıfırla
        } else if (s_line_idx < NMEA_LINE_MAX - 1) {
            s_line_buf[s_line_idx++] = (char)ch;
        }
    }

    if (out != NULL) {
        *out = s_last_gps_data;
    }

    return has_new_fix_packet;
}