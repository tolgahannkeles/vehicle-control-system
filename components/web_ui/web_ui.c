#include "web_ui.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "motor_driver.h"
#include "ota_manager.h"

static const char *TAG = "WEB_UI";

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

static char active_token[32] = {0};
static int64_t last_heartbeat_time = 0;
#define SESSION_TIMEOUT_US 1500000 

static esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    const size_t html_len = index_html_end - index_html_start;
    return httpd_resp_send(req, (const char *)index_html_start, html_len);
}

// HTTP soketinden veriyi alıp ham byte olarak OTA yöneticisine aktaran handler
static esp_err_t ota_post_handler(httpd_req_t *req) {
    motor_stop(); // Güvenlik önlemi

    esp_err_t err = ota_manager_begin();
    if (err != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char buffer[1024];
    int remaining = req->content_len;

    while (remaining > 0) {
        size_t to_read = sizeof(buffer) < (size_t)remaining ? sizeof(buffer) : (size_t)remaining;
        int recv_len = httpd_req_recv(req, buffer, to_read);
        
        if (recv_len <= 0) {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) continue;
            ESP_LOGE(TAG, "Istemciden veri akisi kesildi!");
            ota_manager_abort();
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }

        // Ham veriyi OTA modülüne aktar
        err = ota_manager_write(buffer, recv_len);
        if (err != ESP_OK) {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }

        remaining -= recv_len;
    }

    // Doğrula, tamamla ve cihazı yeniden başlat
    err = ota_manager_finish(true);
    if (err != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "SUCCESS");
    return ESP_OK;
}

static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) return ESP_OK;

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) return ret;

    if (ws_pkt.len > 0) {
        uint8_t *buf = calloc(ws_pkt.len + 1, sizeof(uint8_t));
        if (!buf) return ESP_ERR_NO_MEM;

        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret == ESP_OK) {
            char token[32] = {0};
            int t = 0, s = 0;
            int64_t now = esp_timer_get_time();

            if (sscanf((char*)buf, "%31[^;];%d,%d", token, &t, &s) == 3) {
                if (active_token[0] != '\0' && (now - last_heartbeat_time > SESSION_TIMEOUT_US)) {
                    active_token[0] = '\0';
                    motor_stop();
                }

                if (active_token[0] == '\0') {
                    strncpy(active_token, token, sizeof(active_token) - 1);
                }

                if (strcmp(active_token, token) == 0) {
                    last_heartbeat_time = now;
                    motor_update(t, s);

                    httpd_ws_frame_t ack = { .final = true, .type = HTTPD_WS_TYPE_TEXT, .payload = (uint8_t*)"GRANTED", .len = 7 };
                    httpd_ws_send_frame(req, &ack);
                } else {
                    httpd_ws_frame_t nack = { .final = true, .type = HTTPD_WS_TYPE_TEXT, .payload = (uint8_t*)"LOCKED", .len = 6 };
                    httpd_ws_send_frame(req, &nack);
                }
            }
        }
        free(buf);
    }
    return ESP_OK;
}

httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 10240;
    config.max_open_sockets = 7;
    config.lru_purge_enable = true;
    httpd_handle_t server = NULL;

    httpd_uri_t root_uri = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
    httpd_uri_t ws_uri   = { .uri = "/ws", .method = HTTP_GET, .handler = ws_handler, .is_websocket = true };
    httpd_uri_t ota_uri  = { .uri = "/update", .method = HTTP_POST, .handler = ota_post_handler };

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &ws_uri);
        httpd_register_uri_handler(server, &ota_uri);
        ESP_LOGI(TAG, "Web Sunucusu baslatildi.");
    }
    return server;
}

void stop_webserver(httpd_handle_t server) {
    if (server) httpd_stop(server);
}