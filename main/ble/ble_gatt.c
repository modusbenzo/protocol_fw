#include "ble_gatt.h"
#include "mesh_proto.h"
#include "mesh.h"
#include "admin_cmd.h"
#include "node_config.h"
#include <esp_log.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <host/ble_hs.h>
#include <host/util/util.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

static const char *TAG = "ble";

// ============================================================
// UUIDs (128-bit, Little Endian für NimBLE)
// ============================================================

// Service: 6BA1B218-15A8-461F-9FA8-5DCAE273EAFD
static const ble_uuid128_t SVCUUID = BLE_UUID128_INIT(
    0xFD, 0xEA, 0x73, 0xE2, 0xCA, 0x5D,
    0xA8, 0x9F, 0x1F, 0x46, 0xA8, 0x15,
    0x18, 0xB2, 0xA1, 0x6B
);

// toRadio: F75C76D2-129E-4DAD-A1DD-7866124401E7
static const ble_uuid128_t UUID_TO_RADIO = BLE_UUID128_INIT(
    0xE7, 0x01, 0x44, 0x12, 0x66, 0x78,
    0xDD, 0xA1, 0xAD, 0x4D, 0x9E, 0x12,
    0xD2, 0x76, 0x5C, 0xF7
);

// fromRadio: 2C55E69E-4993-11ED-B878-0242AC120002
static const ble_uuid128_t UUID_FROM_RADIO = BLE_UUID128_INIT(
    0x02, 0x00, 0x12, 0xAC, 0x42, 0x02,
    0x78, 0xB8, 0xED, 0x11, 0x93, 0x49,
    0x9E, 0xE6, 0x55, 0x2C
);

// fromNum: ED9DA18C-A800-4F66-A670-AA7547E34453
static const ble_uuid128_t UUID_FROM_NUM = BLE_UUID128_INIT(
    0x53, 0x44, 0xE3, 0x47, 0x75, 0xAA,
    0x70, 0xA6, 0x66, 0x4F, 0x00, 0xA8,
    0x8C, 0xA1, 0x9D, 0xED
);

// ============================================================
// State
// ============================================================

static uint16_t s_conn_handle    = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_from_radio_handle  = 0;
static uint16_t s_from_num_handle    = 0;
static bool     s_from_num_notify    = false;

// fromRadio Queue — FIFO, max PROTO_FROM_RADIO_QUEUE_SIZE Einträge
typedef struct {
    uint8_t data[PROTO_BLE_MTU];
    size_t  len;
} from_radio_item_t;

static QueueHandle_t    s_from_radio_queue;
static SemaphoreHandle_t s_ble_mutex;

// fromNum — monoton steigender Counter
static uint32_t s_from_num_value = 0;

// ============================================================
// Characteristic Callbacks
// ============================================================

// toRadio — App schreibt Paket an Firmware
static int cb_to_radio_write(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_REQ_NOT_SUPPORTED;

    uint16_t data_len = OS_MBUF_PKTLEN(ctxt->om);
    if (data_len == 0 || data_len > PROTO_BLE_MTU) {
        ESP_LOGW(TAG, "toRadio: Ungültige Länge %u", data_len);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    // Daten aus mbuf lesen
    uint8_t buf[PROTO_BLE_MTU];
    uint16_t read_len = 0;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &read_len);
    if (rc != 0 || read_len == 0) {
        ESP_LOGE(TAG, "toRadio: mbuf read Fehler");
        return BLE_ATT_ERR_UNLIKELY;
    }

    ESP_LOGD(TAG, "toRadio: %u Bytes empfangen", read_len);

    // Protobuf dekodieren
    to_radio_t tr;
    if (!proto_decode_to_radio(buf, read_len, &tr)) {
        ESP_LOGW(TAG, "toRadio: Decode-Fehler");
        return 0; // Nicht als Fehler zurückgeben
    }

    switch (tr.type) {

        case TO_RADIO_WANT_CONFIG: {
            // Config-Handshake
            ESP_LOGI(TAG, "Config Handshake: nonce=%lu",
                     (unsigned long)tr.want_config_id);

            const node_config_t *cfg = node_config_get();
            char node_id[16];
            node_config_get_node_id(node_id, sizeof(node_id));

            // 1. MyNodeInfo senden
            from_radio_t fr = {0};
            fr.type                   = FROM_RADIO_MY_INFO;
            fr.my_info.my_node_num    = cfg->node_num;
            fr.my_info.reboot_count   = cfg->reboot_count;
            ble_gatt_enqueue_from_radio(&fr);

            // 2. Eigene NodeInfo (als bekannter Knoten)
            memset(&fr, 0, sizeof(fr));
            fr.type              = FROM_RADIO_NODE_INFO;
            fr.node_info.num     = cfg->node_num;
            fr.node_info.has_user = true;
            strncpy(fr.node_info.user.id,         node_id,          sizeof(fr.node_info.user.id)-1);
            strncpy(fr.node_info.user.long_name,  cfg->long_name,   sizeof(fr.node_info.user.long_name)-1);
            strncpy(fr.node_info.user.short_name, cfg->short_name,  sizeof(fr.node_info.user.short_name)-1);
            fr.node_info.user.hw_model = cfg->hw_model;
            ble_gatt_enqueue_from_radio(&fr);

            // 3. Config Complete
            memset(&fr, 0, sizeof(fr));
            fr.type               = FROM_RADIO_CONFIG_COMPLETE;
            fr.config_complete_id = tr.want_config_id;
            ble_gatt_enqueue_from_radio(&fr);

            ESP_LOGI(TAG, "Config Handshake abgeschlossen");
            break;
        }

        case TO_RADIO_PACKET: {
            mesh_packet_t *pkt = &tr.packet;

            // Lokales Admin-Kommando? (to == myNodeNum, portnum=32)
            if (pkt->to == node_config_get()->node_num &&
                pkt->decoded.portnum == PORTNUM_ADMIN) {

                uint8_t resp_payload[64];
                size_t  resp_len = 0;
                if (admin_handle_command(pkt->decoded.payload,
                                          pkt->decoded.payload_len,
                                          resp_payload, sizeof(resp_payload),
                                          &resp_len) && resp_len > 0) {
                    // Antwort als FromRadio zurückschicken
                    from_radio_t fr = {0};
                    fr.type = FROM_RADIO_PACKET;
                    fr.packet.from = node_config_get()->node_num;
                    fr.packet.to   = node_config_get()->node_num;
                    fr.packet.decoded.portnum = PORTNUM_ADMIN;
                    memcpy(fr.packet.decoded.payload, resp_payload, resp_len);
                    fr.packet.decoded.payload_len = resp_len;
                    ble_gatt_enqueue_from_radio(&fr);
                }
            } else {
                // LoRa senden (KEIN Payload-Inspizieren — opaker Transport!)
                // Payload wird direkt weitergegeben
                ESP_LOGD(TAG, "App sendet Paket: to=0x%08lX len=%u port=%d",
                         (unsigned long)pkt->to,
                         (unsigned)pkt->decoded.payload_len,
                         pkt->decoded.portnum);

                mesh_send(pkt->to,
                          pkt->decoded.payload,
                          pkt->decoded.payload_len,
                          pkt->decoded.portnum,
                          pkt->want_ack);
            }
            break;
        }

        default:
            ESP_LOGW(TAG, "toRadio: Unbekannter Typ");
            break;
    }

    return 0;
}

// fromRadio — App liest nächste Nachricht aus Queue
static int cb_from_radio_read(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_REQ_NOT_SUPPORTED;

    from_radio_item_t item;
    if (xQueueReceive(s_from_radio_queue, &item, 0) == pdTRUE) {
        // Daten in Antwort schreiben
        int rc = os_mbuf_append(ctxt->om, item.data, item.len);
        if (rc != 0) {
            ESP_LOGE(TAG, "fromRadio: mbuf append Fehler");
            return BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        ESP_LOGD(TAG, "fromRadio: %u Bytes gelesen", (unsigned)item.len);
    } else {
        // Queue leer → 0 Bytes (Drain-Pattern Ende-Signal)
        // Leere Response = Queue leer
        ESP_LOGD(TAG, "fromRadio: Queue leer");
    }

    return 0;
}

// fromNum — Notification Value
static int cb_from_num_access(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        uint32_t val = s_from_num_value;
        int rc = os_mbuf_append(ctxt->om, &val, 4);
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
}

// fromNum — Subscribe/Unsubscribe Callback
static void cb_from_num_subscribe(uint16_t conn_handle, uint16_t attr_handle,
                                   uint8_t reason, uint8_t prev_notify,
                                   uint8_t cur_notify, uint8_t prev_indicate,
                                   uint8_t cur_indicate, void *arg) {
    s_from_num_notify = cur_notify != 0;
    ESP_LOGI(TAG, "fromNum Subscription: %s", s_from_num_notify ? "aktiv" : "inaktiv");
}

// ============================================================
// GATT Service Definition
// ============================================================

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &SVCUUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            // toRadio: Write with Response
            {
                .uuid       = &UUID_TO_RADIO.u,
                .access_cb  = cb_to_radio_write,
                .flags      = BLE_GATT_CHR_F_WRITE,
            },
            // fromRadio: Read
            {
                .uuid       = &UUID_FROM_RADIO.u,
                .access_cb  = cb_from_radio_read,
                .val_handle = &s_from_radio_handle,
                .flags      = BLE_GATT_CHR_F_READ,
            },
            // fromNum: Read + Notify
            {
                .uuid          = &UUID_FROM_NUM.u,
                .access_cb     = cb_from_num_access,
                .val_handle    = &s_from_num_handle,
                .flags         = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .subscribe_cb  = cb_from_num_subscribe,
            },
            { 0 } // Terminator
        },
    },
    { 0 } // Terminator
};

// ============================================================
// GAP Event Handler
// ============================================================

static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    switch (event->type) {

        case BLE_GAP_EVENT_CONNECT: {
            if (event->connect.status == 0) {
                s_conn_handle = event->connect.conn_handle;
                s_from_num_notify = false;

                // MTU-Negotiation anstoßen
                ble_att_set_preferred_mtu(517);
                ble_gattc_exchange_mtu(s_conn_handle, NULL, NULL);

                ESP_LOGI(TAG, "BLE verbunden: handle=%u", s_conn_handle);
            } else {
                ESP_LOGW(TAG, "BLE Connect Fehler: %d", event->connect.status);
                s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
                ble_gatt_start_advertising();
            }
            break;
        }

        case BLE_GAP_EVENT_DISCONNECT: {
            ESP_LOGI(TAG, "BLE getrennt: reason=%d", event->disconnect.reason);
            s_conn_handle    = BLE_HS_CONN_HANDLE_NONE;
            s_from_num_notify = false;
            // Sofort wieder advertisen
            ble_gatt_start_advertising();
            break;
        }

        case BLE_GAP_EVENT_MTU: {
            ESP_LOGI(TAG, "MTU ausgehandelt: %u", event->mtu.value);
            break;
        }

        case BLE_GAP_EVENT_CONN_UPDATE: {
            ESP_LOGD(TAG, "Connection Update");
            break;
        }

        default:
            break;
    }
    return 0;
}

// ============================================================
// Advertising
// ============================================================

void ble_gatt_start_advertising(void) {
    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;   // Undirected Connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;   // General Discoverable
    adv_params.itvl_min  = BLE_GAP_ADV_ITVL_MS(100);
    adv_params.itvl_max  = BLE_GAP_ADV_ITVL_MS(150);

    // Advertising-Daten: Service UUID + Device Name
    struct ble_hs_adv_fields adv_fields = {0};
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // Service UUID 128-bit
    adv_fields.uuids128       = &SVCUUID;
    adv_fields.num_uuids128   = 1;
    adv_fields.uuids128_is_complete = 1;

    // Gerätename im Scan Response
    const char *device_name = "PROTOCOL";
    struct ble_hs_adv_fields scan_fields = {0};
    scan_fields.name            = (const uint8_t *)device_name;
    scan_fields.name_len        = strlen(device_name);
    scan_fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields Fehler: %d", rc);
        return;
    }

    ble_gap_adv_rsp_set_fields(&scan_fields);

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                            &adv_params, gap_event_handler, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "adv_start Fehler: %d", rc);
    } else {
        ESP_LOGI(TAG, "BLE Advertising gestartet (UUID: 6BA1B218...)");
    }
}

// ============================================================
// NimBLE Host Task
// ============================================================

static void ble_host_task(void *param) {
    ESP_LOGI(TAG, "NimBLE Host Task gestartet");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void on_sync(void) {
    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);
    ble_gatt_start_advertising();
}

static void on_reset(int reason) {
    ESP_LOGE(TAG, "BLE Host Reset: reason=%d", reason);
}

// ============================================================
// Init
// ============================================================

void ble_gatt_init(void) {
    s_from_radio_queue = xQueueCreate(PROTO_FROM_RADIO_QUEUE_SIZE,
                                       sizeof(from_radio_item_t));
    s_ble_mutex        = xSemaphoreCreateMutex();

    // NimBLE initialisieren
    int rc = nimble_port_init();
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init Fehler: %d", rc);
        return;
    }

    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    // BLE Security Konfiguration (LE Secure Connections)
    ble_hs_cfg.sm_io_cap           = BLE_SM_IO_CAP_NO_IO;  // Just Works
    ble_hs_cfg.sm_bonding          = 1;
    ble_hs_cfg.sm_mitm             = 0;
    ble_hs_cfg.sm_sc               = 1;   // Secure Connections
    ble_hs_cfg.sm_our_key_dist     = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    // GAP / GATT Services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // GATT Service registrieren
    rc = ble_gatts_count_cfg(s_gatt_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(s_gatt_svcs);
    assert(rc == 0);

    // NimBLE Host Task starten
    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "BLE GATT Service initialisiert");
}

// ============================================================
// fromRadio enqueue
// ============================================================

bool ble_gatt_enqueue_from_radio(const from_radio_t *fr) {
    from_radio_item_t item;
    size_t encoded_len;

    // Statische ID für fromRadio Nachrichten
    static uint32_t s_fr_id = 1;
    from_radio_t fr_with_id = *fr;
    fr_with_id.id = s_fr_id++;

    if (!proto_encode_from_radio(&fr_with_id,
                                  item.data, sizeof(item.data),
                                  &encoded_len)) {
        ESP_LOGE(TAG, "fromRadio encode Fehler");
        return false;
    }
    item.len = encoded_len;

    if (xQueueSend(s_from_radio_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "fromRadio Queue voll");
        return false;
    }

    // fromNum inkrementieren und Notification senden
    s_from_num_value++;

    if (s_from_num_notify &&
        s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {

        struct os_mbuf *om = ble_hs_mbuf_from_flat(&s_from_num_value, 4);
        if (om) {
            int rc = ble_gattc_notify_custom(s_conn_handle,
                                              s_from_num_handle, om);
            if (rc != 0) {
                ESP_LOGW(TAG, "fromNum notify Fehler: %d", rc);
            }
        }
    }

    ESP_LOGD(TAG, "FromRadio enqueued: type=%d len=%u",
             fr->type, (unsigned)encoded_len);
    return true;
}

bool ble_gatt_is_connected(void) {
    return s_conn_handle != BLE_HS_CONN_HANDLE_NONE;
}
