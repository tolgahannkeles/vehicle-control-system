#include "protocol.h"
#include <string.h>

uint8_t protocol_calc_crc(uint8_t id, uint8_t len, const uint8_t *data) {
    uint8_t crc = id ^ len;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}

void protocol_parser_init(protocol_parser_t *parser) {
    parser->state = SM_WAIT_H1;
    parser->index = 0;
}

bool protocol_parse_byte(protocol_parser_t *p, uint8_t b, uint8_t *out_id, uint8_t *out_data, uint8_t *out_len) {
    switch (p->state) {
        case SM_WAIT_H1:
            if (b == FRAME_HEADER1) p->state = SM_WAIT_H2;
            break;

        case SM_WAIT_H2:
            p->state = (b == FRAME_HEADER2) ? SM_WAIT_ID : SM_WAIT_H1;
            break;

        case SM_WAIT_ID:
            p->id = b;
            p->state = SM_WAIT_LEN;
            break;

        case SM_WAIT_LEN:
            p->len = b;
            p->index = 0;
            if (p->len > sizeof(p->buffer)) {
                p->state = SM_WAIT_H1;
            } else {
                p->state = (p->len == 0) ? SM_WAIT_CRC : SM_WAIT_PAYLOAD;
            }
            break;

        case SM_WAIT_PAYLOAD:
            p->buffer[p->index++] = b;
            if (p->index >= p->len) {
                p->state = SM_WAIT_CRC;
            }
            break;

        case SM_WAIT_CRC: {
            uint8_t crc = protocol_calc_crc(p->id, p->len, p->buffer);
            p->state = SM_WAIT_H1;
            if (crc == b) {
                *out_id = p->id;
                *out_len = p->len;
                memcpy(out_data, p->buffer, p->len);
                return true;
            }
            break;
        }
    }
    return false;
}