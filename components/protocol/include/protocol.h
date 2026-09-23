#pragma once

#include <stdint.h>
#include <stdbool.h>

#define FRAME_HEADER1    0xAA
#define FRAME_HEADER2    0x55

#define PKT_ID_CMD_VEL   0x01
#define PKT_ID_IMU       0x10
#define PKT_ID_GPS       0x11

#pragma pack(push, 1)
typedef struct {
    float v_left;   // m/s
    float v_right;  // m/s
} cmd_vel_payload_t;

typedef struct {
    float ax, ay, az; // m/s^2
    float gx, gy, gz; // rad/s
    float temp;       // °C
} imu_payload_t;

typedef struct {
    double lat;
    double lon;
    uint8_t fix;
} gps_payload_t;
#pragma pack(pop)

typedef enum {
    SM_WAIT_H1,
    SM_WAIT_H2,
    SM_WAIT_ID,
    SM_WAIT_LEN,
    SM_WAIT_PAYLOAD,
    SM_WAIT_CRC
} parse_state_t;

typedef struct {
    parse_state_t state;
    uint8_t id;
    uint8_t len;
    uint8_t index;
    uint8_t buffer[64];
} protocol_parser_t;

// Sadece fonksiyon bildirimleri:
uint8_t protocol_calc_crc(uint8_t id, uint8_t len, const uint8_t *data);
void protocol_parser_init(protocol_parser_t *parser);
bool protocol_parse_byte(protocol_parser_t *p, uint8_t b, uint8_t *out_id, uint8_t *out_data, uint8_t *out_len);