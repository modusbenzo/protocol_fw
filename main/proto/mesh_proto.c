#include "mesh_proto.h"
#include <string.h>
#include <esp_log.h>

static const char *TAG = "proto";

// ============================================================
// Protobuf Wire Encoding Primitives
// ============================================================

// Wire types
#define WT_VARINT   0
#define WT_64BIT    1
#define WT_LEN      2
#define WT_32BIT    5

#define FIELD_TAG(field_num, wire_type) (((field_num) << 3) | (wire_type))

// --- Write helpers ---

static uint8_t *write_varint(uint8_t *p, uint64_t v) {
    while (v > 0x7F) {
        *p++ = (uint8_t)(v | 0x80);
        v >>= 7;
    }
    *p++ = (uint8_t)v;
    return p;
}

static uint8_t *write_tag(uint8_t *p, int field, int wire_type) {
    return write_varint(p, FIELD_TAG(field, wire_type));
}

// fixed32 (Little Endian 4 Bytes)
static uint8_t *write_fixed32(uint8_t *p, uint32_t v) {
    p[0] = (v      ) & 0xFF;
    p[1] = (v >>  8) & 0xFF;
    p[2] = (v >> 16) & 0xFF;
    p[3] = (v >> 24) & 0xFF;
    return p + 4;
}

// float als fixed32
static uint8_t *write_float(uint8_t *p, float v) {
    uint32_t bits;
    memcpy(&bits, &v, 4);
    return write_fixed32(p, bits);
}

// int32 als varint (zigzag für negative Werte)
static uint8_t *write_int32(uint8_t *p, int32_t v) {
    return write_varint(p, (uint64_t)(uint32_t)v);
}

// Length-delimited Feld (string oder bytes)
static uint8_t *write_bytes(uint8_t *p, int field,
                             const uint8_t *data, size_t len) {
    if (len == 0) return p;
    p = write_tag(p, field, WT_LEN);
    p = write_varint(p, len);
    memcpy(p, data, len);
    return p + len;
}

static uint8_t *write_string(uint8_t *p, int field, const char *s) {
    size_t len = s ? strlen(s) : 0;
    if (len == 0) return p;
    return write_bytes(p, field, (const uint8_t *)s, len);
}

// Embedded message: schreibe Länge + Inhalt
// Strategie: erst Inhalt in Temp-Puffer, dann als LEN-delimited schreiben
#define TEMP_BUF_SIZE 256

// --- Read helpers ---

static const uint8_t *read_varint(const uint8_t *p, const uint8_t *end,
                                   uint64_t *out) {
    uint64_t result = 0;
    int shift = 0;
    while (p < end) {
        uint8_t b = *p++;
        result |= (uint64_t)(b & 0x7F) << shift;
        shift += 7;
        if (!(b & 0x80)) {
            *out = result;
            return p;
        }
        if (shift >= 64) return NULL;
    }
    return NULL; // Truncated
}

static const uint8_t *read_fixed32(const uint8_t *p, const uint8_t *end,
                                    uint32_t *out) {
    if (p + 4 > end) return NULL;
    *out = ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
    return p + 4;
}

static const uint8_t *read_tag(const uint8_t *p, const uint8_t *end,
                                 int *field, int *wire_type) {
    uint64_t tag;
    p = read_varint(p, end, &tag);
    if (!p) return NULL;
    *field = (int)(tag >> 3);
    *wire_type = (int)(tag & 0x07);
    return p;
}

// Skip unbekannte Felder
static const uint8_t *skip_field(const uint8_t *p, const uint8_t *end,
                                   int wire_type) {
    uint64_t v;
    uint32_t f32;
    uint64_t f64;
    switch (wire_type) {
        case WT_VARINT:
            return read_varint(p, end, &v);
        case WT_LEN:
            p = read_varint(p, end, &v);
            if (!p || p + v > end) return NULL;
            return p + v;
        case WT_32BIT:
            if (p + 4 > end) return NULL;
            return p + 4;
        case WT_64BIT:
            if (p + 8 > end) return NULL;
            return p + 8;
        default:
            return NULL;
    }
    (void)f32; (void)f64;
}

// ============================================================
// DataMessage Encoder
// ============================================================

static uint8_t *encode_data_message(uint8_t *p, const data_message_t *dm) {
    // portnum = field 1, varint
    if (dm->portnum != PORTNUM_UNKNOWN) {
        p = write_tag(p, 1, WT_VARINT);
        p = write_varint(p, dm->portnum);
    }
    // payload = field 2, bytes
    if (dm->payload_len > 0) {
        p = write_bytes(p, 2, dm->payload, dm->payload_len);
    }
    // want_response = field 3, varint (bool)
    if (dm->want_response) {
        p = write_tag(p, 3, WT_VARINT);
        p = write_varint(p, 1);
    }
    return p;
}

// ============================================================
// MeshPacket Encoder (für LoRa on-Air)
// ============================================================

bool proto_encode_mesh_packet(const mesh_packet_t *pkt,
                               uint8_t *buf, size_t buf_size,
                               size_t *out_len) {
    uint8_t *p = buf;
    uint8_t *end = buf + buf_size;

    // from = field 1, fixed32
    p = write_tag(p, 1, WT_32BIT);
    p = write_fixed32(p, pkt->from);

    // to = field 2, fixed32
    p = write_tag(p, 2, WT_32BIT);
    p = write_fixed32(p, pkt->to);

    // id = field 6, fixed32
    p = write_tag(p, 6, WT_32BIT);
    p = write_fixed32(p, pkt->id);

    // hop_limit = field 9, varint
    p = write_tag(p, 9, WT_VARINT);
    p = write_varint(p, pkt->hop_limit);

    // want_ack = field 10, varint (bool)
    if (pkt->want_ack) {
        p = write_tag(p, 10, WT_VARINT);
        p = write_varint(p, 1);
    }

    // decoded = field 4, LEN-delimited (DataMessage)
    // Erst DataMessage in Temp-Puffer kodieren
    uint8_t tmp[TEMP_BUF_SIZE];
    uint8_t *tmp_end = encode_data_message(tmp, &pkt->decoded);
    size_t  tmp_len  = tmp_end - tmp;

    p = write_bytes(p, 4, tmp, tmp_len);

    if (p > end) {
        ESP_LOGE(TAG, "encode_mesh_packet: Buffer overflow");
        return false;
    }

    *out_len = p - buf;
    return true;
}

// ============================================================
// MeshPacket Decoder (empfangenes LoRa-Paket)
// ============================================================

static bool decode_data_message(const uint8_t *data, size_t len,
                                  data_message_t *dm) {
    const uint8_t *p = data;
    const uint8_t *end = data + len;
    memset(dm, 0, sizeof(*dm));

    while (p < end) {
        int field, wt;
        p = read_tag(p, end, &field, &wt);
        if (!p) return false;

        uint64_t v;
        switch (field) {
            case 1: // portnum
                p = read_varint(p, end, &v);
                if (!p) return false;
                dm->portnum = (portnum_t)v;
                break;
            case 2: // payload
                if (wt != WT_LEN) return false;
                p = read_varint(p, end, &v);
                if (!p || p + v > end) return false;
                if (v > PROTO_MAX_PAYLOAD_BYTES) {
                    ESP_LOGW(TAG, "Payload zu groß: %u", (unsigned)v);
                    return false;
                }
                memcpy(dm->payload, p, v);
                dm->payload_len = v;
                p += v;
                break;
            case 3: // want_response
                p = read_varint(p, end, &v);
                if (!p) return false;
                dm->want_response = v != 0;
                break;
            default:
                p = skip_field(p, end, wt);
                if (!p) return false;
                break;
        }
    }
    return true;
}

bool proto_decode_mesh_packet(const uint8_t *data, size_t data_len,
                               mesh_packet_t *pkt) {
    const uint8_t *p = data;
    const uint8_t *end = data + data_len;
    memset(pkt, 0, sizeof(*pkt));
    pkt->channel = 0;

    while (p < end) {
        int field, wt;
        p = read_tag(p, end, &field, &wt);
        if (!p) return false;

        uint64_t v;
        uint32_t f32;

        switch (field) {
            case 1: // from — fixed32
                p = read_fixed32(p, end, &pkt->from);
                if (!p) return false;
                break;
            case 2: // to — fixed32
                p = read_fixed32(p, end, &pkt->to);
                if (!p) return false;
                break;
            case 3: // channel
                p = read_varint(p, end, &v);
                if (!p) return false;
                // Ignorieren, immer 0
                break;
            case 4: // decoded (DataMessage) — LEN
                if (wt != WT_LEN) return false;
                p = read_varint(p, end, &v);
                if (!p || p + v > end) return false;
                if (!decode_data_message(p, v, &pkt->decoded)) return false;
                p += v;
                break;
            case 6: // id — fixed32
                p = read_fixed32(p, end, &pkt->id);
                if (!p) return false;
                break;
            case 7: // rx_time — fixed32
                p = read_fixed32(p, end, &pkt->rx_time);
                if (!p) return false;
                break;
            case 8: // rx_snr — float (fixed32)
                p = read_fixed32(p, end, &f32);
                if (!p) return false;
                memcpy(&pkt->rx_snr, &f32, 4);
                break;
            case 9: // hop_limit
                p = read_varint(p, end, &v);
                if (!p) return false;
                pkt->hop_limit = (uint32_t)v;
                break;
            case 10: // want_ack
                p = read_varint(p, end, &v);
                if (!p) return false;
                pkt->want_ack = v != 0;
                break;
            case 12: // rx_rssi — int32 (varint)
                p = read_varint(p, end, &v);
                if (!p) return false;
                pkt->rx_rssi = (int32_t)(uint32_t)v;
                break;
            default:
                p = skip_field(p, end, wt);
                if (!p) return false;
                break;
        }
    }
    return true;
}

// ============================================================
// User Encoder (für NodeInfo)
// ============================================================

static uint8_t *encode_user(uint8_t *p, const mesh_user_t *u) {
    p = write_string(p, 1, u->id);         // id
    p = write_string(p, 2, u->long_name);  // long_name
    p = write_string(p, 3, u->short_name); // short_name
    if (u->hw_model != HW_UNSET) {
        p = write_tag(p, 5, WT_VARINT);
        p = write_varint(p, u->hw_model);
    }
    return p;
}

// ============================================================
// FromRadio Encoder
// ============================================================

bool proto_encode_from_radio(const from_radio_t *fr,
                              uint8_t *buf, size_t buf_size,
                              size_t *out_len) {
    uint8_t *p = buf;
    uint8_t *end = buf + buf_size;
    uint8_t tmp[TEMP_BUF_SIZE];
    uint8_t *tmp_end;
    size_t  tmp_len;

    // id = field 1, varint
    p = write_tag(p, 1, WT_VARINT);
    p = write_varint(p, fr->id);

    switch (fr->type) {

        case FROM_RADIO_PACKET: {
            // packet = field 2, LEN-delimited (MeshPacket)
            if (!proto_encode_mesh_packet(&fr->packet, tmp, sizeof(tmp), &tmp_len)) {
                return false;
            }
            p = write_bytes(p, 2, tmp, tmp_len);
            break;
        }

        case FROM_RADIO_MY_INFO: {
            // my_info = field 3, LEN-delimited (MyNodeInfo)
            uint8_t *q = tmp;
            // my_node_num = field 1, varint
            q = write_tag(q, 1, WT_VARINT);
            q = write_varint(q, fr->my_info.my_node_num);
            // reboot_count = field 2, varint
            q = write_tag(q, 2, WT_VARINT);
            q = write_varint(q, fr->my_info.reboot_count);
            tmp_len = q - tmp;
            p = write_bytes(p, 3, tmp, tmp_len);
            break;
        }

        case FROM_RADIO_NODE_INFO: {
            // node_info = field 4, LEN-delimited (NodeInfo)
            uint8_t *q = tmp;
            // num = field 1, varint
            q = write_tag(q, 1, WT_VARINT);
            q = write_varint(q, fr->node_info.num);
            // user = field 2, LEN-delimited
            if (fr->node_info.has_user) {
                uint8_t utmp[128];
                uint8_t *uend = encode_user(utmp, &fr->node_info.user);
                size_t  ulen  = uend - utmp;
                q = write_bytes(q, 2, utmp, ulen);
            }
            // snr = field 4, float
            q = write_tag(q, 4, WT_32BIT);
            q = write_float(q, fr->node_info.snr);
            // last_heard = field 5, fixed32
            q = write_tag(q, 5, WT_32BIT);
            q = write_fixed32(q, fr->node_info.last_heard);
            tmp_len = q - tmp;
            p = write_bytes(p, 4, tmp, tmp_len);
            break;
        }

        case FROM_RADIO_CONFIG_COMPLETE:
            // config_complete_id = field 7, varint
            p = write_tag(p, 7, WT_VARINT);
            p = write_varint(p, fr->config_complete_id);
            break;

        case FROM_RADIO_REBOOTED:
            // rebooted = field 8, varint (bool)
            p = write_tag(p, 8, WT_VARINT);
            p = write_varint(p, 1);
            break;

        default:
            break;
    }

    if (p > end) {
        ESP_LOGE(TAG, "encode_from_radio: Buffer overflow");
        return false;
    }

    *out_len = p - buf;
    return true;
}

// ============================================================
// ToRadio Decoder
// ============================================================

bool proto_decode_to_radio(const uint8_t *data, size_t data_len,
                            to_radio_t *tr) {
    const uint8_t *p = data;
    const uint8_t *end = data + data_len;
    memset(tr, 0, sizeof(*tr));

    while (p < end) {
        int field, wt;
        p = read_tag(p, end, &field, &wt);
        if (!p) return false;

        uint64_t v;

        switch (field) {
            case 1: // packet — LEN-delimited (MeshPacket)
                if (wt != WT_LEN) return false;
                p = read_varint(p, end, &v);
                if (!p || p + v > end) return false;
                if (!proto_decode_mesh_packet(p, (size_t)v, &tr->packet)) {
                    return false;
                }
                tr->type = TO_RADIO_PACKET;
                p += v;
                break;

            case 3: // want_config_id — varint
                p = read_varint(p, end, &v);
                if (!p) return false;
                tr->want_config_id = (uint32_t)v;
                tr->type = TO_RADIO_WANT_CONFIG;
                break;

            default:
                p = skip_field(p, end, wt);
                if (!p) return false;
                break;
        }
    }
    return true;
}
