#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ============================================================
// PROTOCOL° Firmware — Protobuf Wire Codec
// ============================================================
// Leichtgewichtiger manueller Protobuf-Encoder/Decoder.
// Implementiert genau die Felder, die PROTOCOL° braucht.
// Keine externe Abhängigkeit (kein nanopb nötig).
// ============================================================

// --- Maximale Größen ---
#define PROTO_MAX_PAYLOAD_BYTES     230
#define PROTO_MAX_LONG_NAME         40
#define PROTO_MAX_SHORT_NAME        4
#define PROTO_MAX_NODE_ID           12    // "!a3f8b2c1\0"
#define PROTO_BLE_MTU               250   // Max BLE Write/Read
#define PROTO_FROM_RADIO_QUEUE_SIZE 16    // FIFO Queue für App

// --- PortNum Enum (Meshtastic-kompatibel) ---
typedef enum {
    PORTNUM_UNKNOWN        = 0,
    PORTNUM_TEXT_MESSAGE   = 1,   // App-Payloads (0xE1-0xE8) — opak transportieren
    PORTNUM_POSITION       = 3,
    PORTNUM_NODEINFO       = 4,
    PORTNUM_ADMIN          = 32,  // Lokale Admin-Kommandos
    PORTNUM_TELEMETRY      = 67,
} portnum_t;

// --- HardwareModel Enum (Meshtastic-kompatibel) ---
typedef enum {
    HW_UNSET             = 0,
    HW_HELTEC_V2         = 13,
    HW_HELTEC_V3         = 43,
    HW_HELTEC_LORA32_V4  = 46,
    HW_TBEAM             = 3,
} hw_model_t;

// --- DataMessage (meshtastic.Data) ---
typedef struct {
    portnum_t portnum;
    uint8_t   payload[PROTO_MAX_PAYLOAD_BYTES];
    size_t    payload_len;
    bool      want_response;
} data_message_t;

// --- MeshPacket ---
typedef struct {
    uint32_t       from;          // fixed32 — Sender NodeNum
    uint32_t       to;            // fixed32 — Empfänger (0xFFFFFFFF = Broadcast)
    uint32_t       id;            // fixed32 — Paket-ID
    uint32_t       rx_time;       // fixed32 — Unix Timestamp
    float          rx_snr;        // float
    int32_t        rx_rssi;       // int32
    uint32_t       hop_limit;     // uint32
    bool           want_ack;      // bool
    uint32_t       channel;       // uint32 (immer 0)
    data_message_t decoded;       // DataMessage
} mesh_packet_t;

// --- User (NodeInfo User) ---
typedef struct {
    char       id[PROTO_MAX_NODE_ID + 1];
    char       long_name[PROTO_MAX_LONG_NAME + 1];
    char       short_name[PROTO_MAX_SHORT_NAME + 1];
    hw_model_t hw_model;
} mesh_user_t;

// --- Position ---
typedef struct {
    int32_t  latitude_i;   // Grad * 1e7
    int32_t  longitude_i;  // Grad * 1e7
    int32_t  altitude;
    uint32_t time;
} mesh_position_t;

// --- NodeInfo ---
typedef struct {
    uint32_t       num;
    mesh_user_t    user;
    mesh_position_t position;
    bool           has_user;
    bool           has_position;
    float          snr;
    uint32_t       last_heard;
} node_info_t;

// --- MyNodeInfo ---
typedef struct {
    uint32_t my_node_num;
    uint32_t reboot_count;
} my_node_info_t;

// --- ToRadio (App → Firmware) ---
typedef enum {
    TO_RADIO_NONE       = 0,
    TO_RADIO_PACKET     = 1,   // MeshPacket senden
    TO_RADIO_WANT_CONFIG = 3,  // Config-Handshake
} to_radio_type_t;

typedef struct {
    to_radio_type_t type;
    union {
        mesh_packet_t packet;
        uint32_t      want_config_id;
    };
} to_radio_t;

// --- FromRadio (Firmware → App) ---
typedef enum {
    FROM_RADIO_NONE              = 0,
    FROM_RADIO_PACKET            = 2,   // Empfangenes MeshPacket
    FROM_RADIO_MY_INFO           = 3,   // MyNodeInfo
    FROM_RADIO_NODE_INFO         = 4,   // NodeInfo (bekannter Knoten)
    FROM_RADIO_CONFIG_COMPLETE   = 7,   // Config-Handshake abgeschlossen
    FROM_RADIO_REBOOTED          = 8,   // Neustart-Benachrichtigung
} from_radio_type_t;

typedef struct {
    uint32_t          id;    // Monotone Nachricht-ID
    from_radio_type_t type;
    union {
        mesh_packet_t   packet;
        my_node_info_t  my_info;
        node_info_t     node_info;
        uint32_t        config_complete_id;
        bool            rebooted;
    };
} from_radio_t;

// ============================================================
// Encoder API
// ============================================================

/**
 * @brief Kodiert eine from_radio_t Struktur in Protobuf-Bytes.
 * @param fr         Quelle
 * @param buf        Ausgabepuffer
 * @param buf_size   Puffergröße
 * @param out_len    Tatsächlich geschriebene Bytes
 * @return true bei Erfolg
 */
bool proto_encode_from_radio(const from_radio_t *fr,
                              uint8_t *buf, size_t buf_size,
                              size_t *out_len);

/**
 * @brief Dekodiert Protobuf-Bytes in eine to_radio_t Struktur.
 * @param data       Eingabedaten (BLE Write)
 * @param data_len   Länge
 * @param tr         Ausgabe
 * @return true bei Erfolg
 */
bool proto_decode_to_radio(const uint8_t *data, size_t data_len,
                            to_radio_t *tr);

/**
 * @brief Kodiert ein MeshPacket in Protobuf-Bytes (für LoRa on-Air).
 */
bool proto_encode_mesh_packet(const mesh_packet_t *pkt,
                               uint8_t *buf, size_t buf_size,
                               size_t *out_len);

/**
 * @brief Dekodiert ein empfangenes LoRa-Paket (Protobuf) in mesh_packet_t.
 */
bool proto_decode_mesh_packet(const uint8_t *data, size_t data_len,
                               mesh_packet_t *pkt);
