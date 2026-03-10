#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <nvs_flash.h>

#include "node_config.h"
#include "mesh_proto.h"
#include "ble_gatt.h"
#include "sx1262.h"
#include "mesh.h"

static const char *TAG = "main";

// ============================================================
// PROTOCOL° Firmware — Main
// ============================================================
//
// Boot-Sequenz:
//   1. NVS + Node Config laden
//   2. BLE GATT Service initialisieren + Advertising
//   3. SX1262 LoRa initialisieren
//   4. Mesh Layer initialisieren
//   5. NodeInfo-Timer starten (wenn Ghost Mode inaktiv)
//   6. FreeRTOS läuft
//
// Datenfluss:
//
//   App → BLE Write (toRadio) → ble_gatt.c → mesh_send() → sx1262_transmit()
//   LoRa RX → sx1262.c RX-Task → mesh_on_rx() → ble_gatt_enqueue_from_radio()
//                                              → fromNum Notification → App liest
// ============================================================

// ============================================================
// Mesh → BLE Callback
// (Empfangene Pakete, die für die App bestimmt sind)
// ============================================================

static void on_mesh_rx_for_app(const mesh_packet_t *pkt) {
    from_radio_t fr = {0};
    fr.type   = FROM_RADIO_PACKET;
    fr.packet = *pkt;
    ble_gatt_enqueue_from_radio(&fr);
}

// ============================================================
// SX1262 → Mesh Callback
// (Rohe empfangene LoRa-Bytes → Mesh Layer)
// ============================================================

static void on_lora_rx(const uint8_t *data, size_t len,
                        int16_t rssi, float snr) {
    mesh_on_rx(data, len, rssi, snr);
}

// ============================================================
// NodeInfo Broadcast Timer
// ============================================================

static esp_timer_handle_t s_nodeinfo_timer;

static void nodeinfo_timer_cb(void *arg) {
    const node_config_t *cfg = node_config_get();

    if (cfg->ghost_mode) {
        // Ghost Mode → absolut kein autonomes Senden
        ESP_LOGD(TAG, "NodeInfo Timer: Ghost Mode aktiv, Broadcast unterdrückt");
        return;
    }

    mesh_send_nodeinfo();
}

static void start_nodeinfo_timer(void) {
    const node_config_t *cfg = node_config_get();

    if (cfg->nodeinfo_interval_sec == 0) {
        ESP_LOGI(TAG, "NodeInfo-Broadcast deaktiviert (interval=0)");
        return;
    }

    if (cfg->ghost_mode) {
        ESP_LOGI(TAG, "NodeInfo-Broadcast unterdrückt (Ghost Mode)");
        return;
    }

    esp_timer_create_args_t timer_args = {
        .callback        = nodeinfo_timer_cb,
        .arg             = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name            = "nodeinfo",
    };

    esp_timer_create(&timer_args, &s_nodeinfo_timer);
    uint64_t interval_us = (uint64_t)cfg->nodeinfo_interval_sec * 1000000ULL;
    esp_timer_start_periodic(s_nodeinfo_timer, interval_us);

    ESP_LOGI(TAG, "NodeInfo-Timer: alle %lu Sekunden",
             (unsigned long)cfg->nodeinfo_interval_sec);
}

// ============================================================
// Boot Info Logging
// ============================================================

static void log_boot_info(void) {
    const node_config_t *cfg = node_config_get();
    char node_id[16];
    node_config_get_node_id(node_id, sizeof(node_id));

    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "  PROTOCOL° Custom Firmware v0.1.0");
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "  NodeNum:   0x%08lX (%s)",
             (unsigned long)cfg->node_num, node_id);
    ESP_LOGI(TAG, "  LongName:  %s", cfg->long_name);
    ESP_LOGI(TAG, "  ShortName: %s", cfg->short_name);
    ESP_LOGI(TAG, "  GhostMode: %s", cfg->ghost_mode ? "AKTIV" : "inaktiv");
    ESP_LOGI(TAG, "  Region:    %d", cfg->region);
    ESP_LOGI(TAG, "  TXPower:   %d dBm", cfg->tx_power_dbm);
    ESP_LOGI(TAG, "  HopLimit:  %d", cfg->hop_limit);
    ESP_LOGI(TAG, "  Reboots:   %lu", (unsigned long)cfg->reboot_count);
    ESP_LOGI(TAG, "=================================================");
}

// ============================================================
// app_main
// ============================================================

void app_main(void) {
    // 1. Node-Konfiguration laden (initialisiert auch NVS)
    node_config_init();
    node_config_increment_reboot_count();

    log_boot_info();

    const node_config_t *cfg = node_config_get();

    // 2. BLE GATT Service + Advertising
    ESP_LOGI(TAG, "Starte BLE...");
    ble_gatt_init();

    // Kurze Pause damit NimBLE hochläuft
    vTaskDelay(pdMS_TO_TICKS(500));

    // 3. SX1262 LoRa initialisieren
    ESP_LOGI(TAG, "Starte LoRa (SX1262)...");
    if (!sx1262_init(cfg->region, on_lora_rx)) {
        ESP_LOGE(TAG, "SX1262 Init Fehler! Prüfe Verkabelung.");
        // In Fehlerfall trotzdem BLE laufen lassen
    }

    // 4. Mesh Layer
    ESP_LOGI(TAG, "Starte Mesh Layer...");
    mesh_init(cfg->node_num, on_mesh_rx_for_app);

    // 5. NodeInfo-Timer
    start_nodeinfo_timer();

    ESP_LOGI(TAG, "Boot abgeschlossen — bereit.");
    ESP_LOGI(TAG, "Ghost Mode: %s | LoRa RX: aktiv | BLE: Advertising",
             cfg->ghost_mode ? "AKTIV (unsichtbar)" : "inaktiv (sichtbar)");

    // 6. Hauptloop — Watchdog füttert sich durch FreeRTOS
    //    Alle eigentliche Logik läuft in Tasks:
    //      - lora_rx (Core 1): SX1262 RX
    //      - mesh_tx (Core 1): LoRa TX Queue
    //      - BLE Host Task (Core 0): NimBLE
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));

        // Heartbeat Log (alle 10 Sekunden)
        ESP_LOGD(TAG, "Heartbeat — Heap: %lu B",
                 (unsigned long)esp_get_free_heap_size());
    }
}
