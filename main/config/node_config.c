#include "node_config.h"
#include "board_pins.h"
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_log.h>
#include <esp_random.h>
#include <string.h>
#include <stdio.h>

static const char *TAG = "config";

// NVS Namespace und Keys
#define NVS_NAMESPACE       "protocol_fw"
#define NVS_KEY_NODE_NUM    "node_num"
#define NVS_KEY_LONG_NAME   "long_name"
#define NVS_KEY_SHORT_NAME  "short_name"
#define NVS_KEY_GHOST_MODE  "ghost_mode"
#define NVS_KEY_REGION      "region"
#define NVS_KEY_TX_POWER    "tx_power"
#define NVS_KEY_HOP_LIMIT   "hop_limit"
#define NVS_KEY_NI_INTERVAL "ni_interval"
#define NVS_KEY_REBOOT_CNT  "reboot_cnt"

// RAM-Instanz der Config
static node_config_t s_config;

// ============================================================
// Private Helpers
// ============================================================

static void load_defaults(void) {
    // NodeNum: 32-bit Zufallswert, nie 0 oder 0xFFFFFFFF
    do {
        s_config.node_num = esp_random();
    } while (s_config.node_num == 0 || s_config.node_num == 0xFFFFFFFF);

    strncpy(s_config.long_name,  "PROTOCOL NODE", sizeof(s_config.long_name) - 1);
    s_config.long_name[sizeof(s_config.long_name) - 1] = '\0';
    memcpy(s_config.short_name, "PRTL\0", 5);  // exakt 4 Zeichen + Nullterminator
    s_config.ghost_mode              = false;
    s_config.region                  = REGION_EU868;
    s_config.tx_power_dbm            = LORA_DEFAULT_TX_POWER;
    s_config.hop_limit               = 3;
    s_config.nodeinfo_interval_sec   = 900;   // 15 Minuten
    s_config.reboot_count            = 0;

#if BOARD_TARGET == BOARD_HELTEC_V3
    s_config.hw_model = HW_HELTEC_V3;
#elif BOARD_TARGET == BOARD_HELTEC_V4
    s_config.hw_model = HW_HELTEC_LORA32_V4;
#else
    s_config.hw_model = HW_UNSET;
#endif
}

static nvs_handle_t open_nvs(void) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return 0;
    }
    return handle;
}

// ============================================================
// Public API
// ============================================================

void node_config_init(void) {
    // NVS initialisieren
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS-Partition voll oder neue Version — erase");
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Defaults laden, dann aus NVS überschreiben
    load_defaults();

    nvs_handle_t h = open_nvs();
    if (!h) return;

    uint32_t u32;
    uint8_t  u8;
    size_t   str_len;
    bool     got_node_num = false;

    // node_num
    if (nvs_get_u32(h, NVS_KEY_NODE_NUM, &u32) == ESP_OK) {
        s_config.node_num = u32;
        got_node_num = true;
    }

    // long_name
    str_len = sizeof(s_config.long_name);
    if (nvs_get_str(h, NVS_KEY_LONG_NAME, s_config.long_name, &str_len) != ESP_OK) {
        // behalte Default
    }

    // short_name
    str_len = sizeof(s_config.short_name);
    if (nvs_get_str(h, NVS_KEY_SHORT_NAME, s_config.short_name, &str_len) != ESP_OK) {
        // behalte Default
    }

    // ghost_mode
    if (nvs_get_u8(h, NVS_KEY_GHOST_MODE, &u8) == ESP_OK) {
        s_config.ghost_mode = u8 != 0;
    }

    // region
    if (nvs_get_u8(h, NVS_KEY_REGION, &u8) == ESP_OK) {
        s_config.region = (lora_region_t)u8;
    }

    // tx_power
    if (nvs_get_u8(h, NVS_KEY_TX_POWER, &u8) == ESP_OK) {
        s_config.tx_power_dbm = u8;
    }

    // hop_limit
    if (nvs_get_u8(h, NVS_KEY_HOP_LIMIT, &u8) == ESP_OK) {
        s_config.hop_limit = u8;
    }

    // nodeinfo_interval
    if (nvs_get_u32(h, NVS_KEY_NI_INTERVAL, &u32) == ESP_OK) {
        s_config.nodeinfo_interval_sec = u32;
    }

    // reboot_count
    if (nvs_get_u32(h, NVS_KEY_REBOOT_CNT, &u32) == ESP_OK) {
        s_config.reboot_count = u32;
    }

    nvs_close(h);

    if (!got_node_num) {
        // Erste Initialisierung — NodeNum speichern
        node_config_save();
    }

    ESP_LOGI(TAG, "Config geladen: NodeNum=0x%08lX GhostMode=%d LongName=%s",
             (unsigned long)s_config.node_num,
             s_config.ghost_mode,
             s_config.long_name);
}

const node_config_t *node_config_get(void) {
    return &s_config;
}

void node_config_save(void) {
    nvs_handle_t h = open_nvs();
    if (!h) return;

    nvs_set_u32(h, NVS_KEY_NODE_NUM,   s_config.node_num);
    nvs_set_str(h, NVS_KEY_LONG_NAME,  s_config.long_name);
    nvs_set_str(h, NVS_KEY_SHORT_NAME, s_config.short_name);
    nvs_set_u8 (h, NVS_KEY_GHOST_MODE, s_config.ghost_mode ? 1 : 0);
    nvs_set_u8 (h, NVS_KEY_REGION,     (uint8_t)s_config.region);
    nvs_set_u8 (h, NVS_KEY_TX_POWER,   s_config.tx_power_dbm);
    nvs_set_u8 (h, NVS_KEY_HOP_LIMIT,  s_config.hop_limit);
    nvs_set_u32(h, NVS_KEY_NI_INTERVAL, s_config.nodeinfo_interval_sec);
    nvs_set_u32(h, NVS_KEY_REBOOT_CNT, s_config.reboot_count);

    nvs_commit(h);
    nvs_close(h);

    ESP_LOGD(TAG, "Config gespeichert");
}

void node_config_set_ghost_mode(bool enabled) {
    s_config.ghost_mode = enabled;
    node_config_save();
    ESP_LOGI(TAG, "Ghost Mode: %s", enabled ? "AKTIV" : "INAKTIV");
}

void node_config_set_long_name(const char *name) {
    strncpy(s_config.long_name, name, sizeof(s_config.long_name) - 1);
    s_config.long_name[sizeof(s_config.long_name) - 1] = '\0';
    node_config_save();
}

void node_config_set_short_name(const char *name) {
    strncpy(s_config.short_name, name, sizeof(s_config.short_name) - 1);
    s_config.short_name[sizeof(s_config.short_name) - 1] = '\0';
    node_config_save();
}

void node_config_set_region(lora_region_t region) {
    s_config.region = region;
    node_config_save();
}

void node_config_set_tx_power(uint8_t dbm) {
    if (dbm > LORA_MAX_TX_POWER_DBM) dbm = LORA_MAX_TX_POWER_DBM;
    s_config.tx_power_dbm = dbm;
    node_config_save();
}

void node_config_set_hop_limit(uint8_t hops) {
    if (hops < 1) hops = 1;
    if (hops > 7) hops = 7;
    s_config.hop_limit = hops;
    node_config_save();
}

void node_config_set_nodeinfo_interval(uint32_t seconds) {
    s_config.nodeinfo_interval_sec = seconds;
    node_config_save();
}

void node_config_increment_reboot_count(void) {
    s_config.reboot_count++;
    node_config_save();
}

void node_config_regenerate_node_num(void) {
    ESP_LOGW(TAG, "NodeNum-Kollision erkannt! Generiere neue NodeNum...");
    do {
        s_config.node_num = esp_random();
    } while (s_config.node_num == 0 || s_config.node_num == 0xFFFFFFFF);
    node_config_save();
    ESP_LOGI(TAG, "Neue NodeNum: 0x%08lX", (unsigned long)s_config.node_num);
}

void node_config_nuke(void) {
    ESP_LOGW(TAG, "=== NUKE: Lösche alle Daten ===");
    nvs_handle_t h = open_nvs();
    if (h) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }
    nvs_flash_erase();
    ESP_LOGW(TAG, "=== NUKE abgeschlossen ===");
}

void node_config_get_node_id(char *buf, size_t buf_size) {
    snprintf(buf, buf_size, "!%08lx", (unsigned long)s_config.node_num);
}