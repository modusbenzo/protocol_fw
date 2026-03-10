#include "mesh.h"
#include "sx1262.h"
#include "node_config.h"
#include "mesh_proto.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <string.h>
#include <esp_random.h>

static const char *TAG = "mesh";

// ============================================================
// Paket-Deduplikation — Ring-Buffer mit 256 Einträgen
// ============================================================

#define DEDUP_CACHE_SIZE  256
#define DEDUP_TTL_MS      (10 * 60 * 1000)  // 10 Minuten

typedef struct {
    uint32_t from;
    uint32_t id;
    int64_t  timestamp_ms;  // esp_timer_get_time() / 1000
} dedup_entry_t;

static dedup_entry_t s_dedup_cache[DEDUP_CACHE_SIZE];
static uint16_t      s_dedup_head = 0;  // Nächste Schreibposition
static SemaphoreHandle_t s_dedup_mutex;

static bool dedup_check_and_insert(uint32_t from, uint32_t id) {
    if (!s_dedup_mutex) return false;
    xSemaphoreTake(s_dedup_mutex, portMAX_DELAY);

    int64_t now_ms = esp_timer_get_time() / 1000;

    // Suche in Cache (und bereinige abgelaufene Einträge)
    for (int i = 0; i < DEDUP_CACHE_SIZE; i++) {
        dedup_entry_t *e = &s_dedup_cache[i];
        if (e->from == 0 && e->id == 0) continue;

        // TTL-Ablauf
        if ((now_ms - e->timestamp_ms) > DEDUP_TTL_MS) {
            memset(e, 0, sizeof(*e));
            continue;
        }

        // Duplikat gefunden?
        if (e->from == from && e->id == id) {
            xSemaphoreGive(s_dedup_mutex);
            return true; // Duplikat
        }
    }

    // Kein Duplikat — in Cache eintragen
    s_dedup_cache[s_dedup_head].from         = from;
    s_dedup_cache[s_dedup_head].id           = id;
    s_dedup_cache[s_dedup_head].timestamp_ms = now_ms;
    s_dedup_head = (s_dedup_head + 1) % DEDUP_CACHE_SIZE;

    xSemaphoreGive(s_dedup_mutex);
    return false; // Kein Duplikat
}

// ============================================================
// TX Queue — entkoppelt Mesh-Logic von SX1262-Transmit
// ============================================================

#define TX_QUEUE_DEPTH 8

typedef struct {
    uint8_t  data[250];
    size_t   len;
} tx_queue_item_t;

static QueueHandle_t s_tx_queue;

static void tx_task(void *arg) {
    tx_queue_item_t item;
    while (1) {
        if (xQueueReceive(s_tx_queue, &item, portMAX_DELAY) == pdTRUE) {
            sx1262_transmit(item.data, item.len);
            // Kurze Pause nach TX (für CA / Relay-Fenster)
            vTaskDelay(pdMS_TO_TICKS(50 + (esp_random() % 50)));
        }
    }
}

// ============================================================
// State
// ============================================================

static uint32_t                    s_my_node_num = 0;
static mesh_rx_for_app_callback_t  s_rx_for_app  = NULL;
static bool                        s_ghost_mode  = false;
static uint32_t                    s_packet_id   = 0;
static SemaphoreHandle_t           s_mesh_mutex;

// ============================================================
// Init
// ============================================================

void mesh_init(uint32_t my_node_num, mesh_rx_for_app_callback_t rx_cb) {
    s_my_node_num  = my_node_num;
    s_rx_for_app   = rx_cb;
    s_ghost_mode   = node_config_get()->ghost_mode;
    s_packet_id    = esp_random();

    s_dedup_mutex  = xSemaphoreCreateMutex();
    s_mesh_mutex   = xSemaphoreCreateMutex();
    s_tx_queue     = xQueueCreate(TX_QUEUE_DEPTH, sizeof(tx_queue_item_t));

    memset(s_dedup_cache, 0, sizeof(s_dedup_cache));

    xTaskCreatePinnedToCore(tx_task, "mesh_tx", 4096, NULL, 9, NULL, 1);

    ESP_LOGI(TAG, "Mesh init: NodeNum=0x%08lX GhostMode=%d",
             (unsigned long)my_node_num, s_ghost_mode);
}

// ============================================================
// RX-Pfad (wird aus SX1262 RX-Task aufgerufen)
// ============================================================

void mesh_on_rx(const uint8_t *raw_data, size_t raw_len,
                int16_t rssi, float snr) {
    mesh_packet_t pkt;
    if (!proto_decode_mesh_packet(raw_data, raw_len, &pkt)) {
        ESP_LOGW(TAG, "Ungültiges Paket — Decode-Fehler");
        return;
    }

    // Empfangsmetadaten setzen
    pkt.rx_rssi = rssi;
    pkt.rx_snr  = snr;
    pkt.rx_time = (uint32_t)(esp_timer_get_time() / 1000000);

    ESP_LOGD(TAG, "RX: from=0x%08lX to=0x%08lX id=0x%08lX hop=%lu port=%d len=%u",
             (unsigned long)pkt.from, (unsigned long)pkt.to,
             (unsigned long)pkt.id,   (unsigned long)pkt.hop_limit,
             pkt.decoded.portnum,     (unsigned)pkt.decoded.payload_len);

    // Eigene NodeNum? → Kollisionserkennung
    if (pkt.from == s_my_node_num) {
        ESP_LOGW(TAG, "Eigene NodeNum empfangen! Kollision — regeneriere NodeNum");
        node_config_regenerate_node_num();
        s_my_node_num = node_config_get()->node_num;
        return;
    }

    // Duplikat-Check
    if (dedup_check_and_insert(pkt.from, pkt.id)) {
        ESP_LOGD(TAG, "Duplikat verworfen: from=0x%08lX id=0x%08lX",
                 (unsigned long)pkt.from, (unsigned long)pkt.id);
        return;
    }

    // Paket für uns oder Broadcast?
    bool for_us = (pkt.to == s_my_node_num || pkt.to == 0xFFFFFFFF);

    if (for_us) {
        // An App weiterleiten
        // WICHTIG: Payload wird NICHT inspiziert — opaker Transport
        // Nur Portnum=ADMIN (32) wird lokal verarbeitet
        if (pkt.decoded.portnum == PORTNUM_ADMIN && pkt.to == s_my_node_num) {
            // Admin-Kommandos werden separat im Admin-Modul verarbeitet
            // (main.c leitet über ble weiter)
            // Trotzdem an App-Callback — App ignoriert es oder Admin-Handler nimmt es
        }

        if (s_rx_for_app) {
            s_rx_for_app(&pkt);
        }

        // ACK senden wenn angefordert
        if (pkt.want_ack && pkt.to == s_my_node_num) {
            // ACK: MeshPacket mit portnum=routingApp (32), payload=RoutingAck
            // Auch im Ghost Mode werden ACKs gesendet (verraten keine Identität)
            mesh_packet_t ack_pkt = {0};
            ack_pkt.from      = s_my_node_num;
            ack_pkt.to        = pkt.from;
            ack_pkt.id        = pkt.id;  // gleiche ID
            ack_pkt.hop_limit = 3;
            ack_pkt.want_ack  = false;
            ack_pkt.decoded.portnum      = PORTNUM_ADMIN;  // routingApp
            ack_pkt.decoded.payload[0]   = 0x01;  // RoutingAck
            ack_pkt.decoded.payload_len  = 1;

            uint8_t ack_bytes[128];
            size_t  ack_len;
            if (proto_encode_mesh_packet(&ack_pkt, ack_bytes, sizeof(ack_bytes), &ack_len)) {
                tx_queue_item_t item;
                memcpy(item.data, ack_bytes, ack_len);
                item.len = ack_len;
                xQueueSend(s_tx_queue, &item, pdMS_TO_TICKS(100));
                ESP_LOGD(TAG, "ACK gesendet an 0x%08lX", (unsigned long)pkt.from);
            }
        }
    }

    // Relay: Weiterleiten wenn hopLimit > 0
    // (auch wenn Paket für uns ist — Mesh-Verhalten!)
    if (pkt.hop_limit > 0) {
        pkt.hop_limit--;

        uint8_t relay_bytes[250];
        size_t  relay_len;
        if (proto_encode_mesh_packet(&pkt, relay_bytes, sizeof(relay_bytes), &relay_len)) {
            tx_queue_item_t item;
            memcpy(item.data, relay_bytes, relay_len);
            item.len = relay_len;

            // Zufälliges Delay vor Relay (reduziert Kollisionen)
            vTaskDelay(pdMS_TO_TICKS(esp_random() % 200));

            if (xQueueSend(s_tx_queue, &item, pdMS_TO_TICKS(200)) != pdTRUE) {
                ESP_LOGW(TAG, "TX Queue voll — Relay verworfen");
            }
            ESP_LOGD(TAG, "Relay: id=0x%08lX hop=%lu",
                     (unsigned long)pkt.id, (unsigned long)pkt.hop_limit);
        }
    }
}

// ============================================================
// TX-Pfad (App sendet Paket)
// ============================================================

bool mesh_send(uint32_t to, const uint8_t *payload, size_t payload_len,
               portnum_t portnum, bool want_ack) {
    if (payload_len > 230) {
        ESP_LOGE(TAG, "Payload zu groß: %u (max 230)", (unsigned)payload_len);
        return false;
    }

    mesh_packet_t pkt = {0};
    pkt.from      = s_my_node_num;
    pkt.to        = to;
    pkt.id        = mesh_next_packet_id();
    pkt.hop_limit = node_config_get()->hop_limit;
    pkt.want_ack  = want_ack;
    pkt.decoded.portnum = portnum;
    memcpy(pkt.decoded.payload, payload, payload_len);
    pkt.decoded.payload_len = payload_len;

    // In Dedup-Cache eintragen (eigene Pakete nicht relayed)
    dedup_check_and_insert(pkt.from, pkt.id);

    uint8_t bytes[250];
    size_t  len;
    if (!proto_encode_mesh_packet(&pkt, bytes, sizeof(bytes), &len)) {
        ESP_LOGE(TAG, "Encode-Fehler beim Senden");
        return false;
    }

    tx_queue_item_t item;
    memcpy(item.data, bytes, len);
    item.len = len;

    if (xQueueSend(s_tx_queue, &item, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGE(TAG, "TX Queue voll");
        return false;
    }

    ESP_LOGD(TAG, "Send: to=0x%08lX id=0x%08lX port=%d len=%u",
             (unsigned long)to, (unsigned long)pkt.id, portnum, (unsigned)payload_len);
    return true;
}

// ============================================================
// NodeInfo Broadcast (nur wenn Ghost Mode INAKTIV)
// ============================================================

void mesh_send_nodeinfo(void) {
    if (s_ghost_mode) {
        ESP_LOGD(TAG, "Ghost Mode aktiv — NodeInfo unterdrückt");
        return;
    }

    const node_config_t *cfg = node_config_get();

    // MeshtasticUser als Protobuf-Payload für NODEINFO-Paket
    // Wir bauen das manuell (User-Protobuf)
    uint8_t payload[128] = {0};
    size_t  plen = 0;

    // Node-ID "!xxxxxxxx"
    char node_id[16];
    node_config_get_node_id(node_id, sizeof(node_id));

    // Einfache Protobuf-Kodierung für MeshtasticUser
    // field 1 (id): string
    // field 2 (long_name): string
    // field 3 (short_name): string
    // field 5 (hw_model): varint
    // Wir nutzen das Proto-Framework aus mesh_proto.c nicht direkt,
    // deshalb kurze inline Kodierung:
    uint8_t *p = payload;
    #define WRITE_TAG_FIELD(fnum, wt) do { \
        *p++ = (uint8_t)(((fnum) << 3) | (wt)); \
    } while(0)
    #define WRITE_LEN_STR(fnum, s) do { \
        size_t _l = strlen(s); \
        if (_l > 0) { \
            WRITE_TAG_FIELD(fnum, 2); \
            *p++ = (uint8_t)_l; \
            memcpy(p, s, _l); p += _l; \
        } \
    } while(0)

    WRITE_LEN_STR(1, node_id);
    WRITE_LEN_STR(2, cfg->long_name);
    WRITE_LEN_STR(3, cfg->short_name);
    if (cfg->hw_model != HW_UNSET) {
        WRITE_TAG_FIELD(5, 0); // varint
        *p++ = (uint8_t)cfg->hw_model;
    }
    plen = p - payload;

    mesh_send(0xFFFFFFFF, payload, plen, PORTNUM_NODEINFO, false);
    ESP_LOGI(TAG, "NodeInfo gesendet: %s (%s)", cfg->long_name, node_id);
}

// ============================================================
// Utils
// ============================================================

uint32_t mesh_next_packet_id(void) {
    xSemaphoreTake(s_mesh_mutex, portMAX_DELAY);
    uint32_t id = s_packet_id++;
    xSemaphoreGive(s_mesh_mutex);
    return id;
}

bool mesh_is_ghost_mode(void) {
    return s_ghost_mode;
}

void mesh_update_ghost_mode(bool enabled) {
    s_ghost_mode = enabled;
    ESP_LOGI(TAG, "Ghost Mode: %s", enabled ? "AKTIV" : "INAKTIV");
}
