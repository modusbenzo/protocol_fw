#include "admin_cmd.h"
#include "node_config.h"
#include "mesh.h"
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

static const char *TAG = "admin";

// Firmware Version (Semantic: major.minor.patch.build)
#define FW_VERSION_MAJOR 0
#define FW_VERSION_MINOR 1
#define FW_VERSION_PATCH 0
#define FW_VERSION_BUILD 1

static void reboot_after_delay(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    vTaskDelete(NULL);
}

bool admin_handle_command(const uint8_t *payload, size_t payload_len,
                           uint8_t *resp_buf, size_t resp_max,
                           size_t *resp_len) {
    if (payload_len == 0) return false;

    uint8_t cmd_id = payload[0];
    const uint8_t *params = payload + 1;
    size_t params_len = payload_len - 1;

    ESP_LOGI(TAG, "Admin Cmd: 0x%02X (params: %u bytes)", cmd_id, (unsigned)params_len);

    // Standard-Antwort: [cmd_id][status]
    resp_buf[0] = cmd_id;
    resp_buf[1] = ADMIN_STATUS_OK;
    *resp_len = 2;

    switch (cmd_id) {

        // ---- 0x01: SET_GHOST_MODE ----
        case ADMIN_CMD_SET_GHOST_MODE: {
            if (params_len < 1) {
                resp_buf[1] = ADMIN_STATUS_ERROR; return true;
            }
            bool enabled = params[0] != 0;
            node_config_set_ghost_mode(enabled);
            mesh_update_ghost_mode(enabled);
            ESP_LOGI(TAG, "Ghost Mode: %s", enabled ? "AKTIV" : "INAKTIV");
            break;
        }

        // ---- 0x02: SET_LONG_NAME ----
        case ADMIN_CMD_SET_LONG_NAME: {
            if (params_len == 0 || params_len > PROTO_MAX_LONG_NAME) {
                resp_buf[1] = ADMIN_STATUS_ERROR; return true;
            }
            char name[PROTO_MAX_LONG_NAME + 1] = {0};
            memcpy(name, params, params_len);
            node_config_set_long_name(name);
            break;
        }

        // ---- 0x03: SET_SHORT_NAME ----
        case ADMIN_CMD_SET_SHORT_NAME: {
            if (params_len == 0 || params_len > PROTO_MAX_SHORT_NAME) {
                resp_buf[1] = ADMIN_STATUS_ERROR; return true;
            }
            char name[PROTO_MAX_SHORT_NAME + 1] = {0};
            memcpy(name, params, params_len);
            node_config_set_short_name(name);
            break;
        }

        // ---- 0x04: SET_REGION ----
        case ADMIN_CMD_SET_REGION: {
            if (params_len < 1) {
                resp_buf[1] = ADMIN_STATUS_ERROR; return true;
            }
            lora_region_t region = (lora_region_t)params[0];
            if (region > REGION_AS923) {
                resp_buf[1] = ADMIN_STATUS_ERROR; return true;
            }
            node_config_set_region(region);
            sx1262_set_region(region);
            break;
        }

        // ---- 0x05: SET_TX_POWER ----
        case ADMIN_CMD_SET_TX_POWER: {
            if (params_len < 1) {
                resp_buf[1] = ADMIN_STATUS_ERROR; return true;
            }
            uint8_t dbm = params[0];
            node_config_set_tx_power(dbm);
            sx1262_set_tx_power(dbm);
            break;
        }

        // ---- 0x06: SET_HOP_LIMIT ----
        case ADMIN_CMD_SET_HOP_LIMIT: {
            if (params_len < 1) {
                resp_buf[1] = ADMIN_STATUS_ERROR; return true;
            }
            node_config_set_hop_limit(params[0]);
            break;
        }

        // ---- 0x07: GET_DEVICE_INFO ----
        case ADMIN_CMD_GET_DEVICE_INFO: {
            const node_config_t *cfg = node_config_get();
            uint32_t uptime_sec = (uint32_t)(esp_timer_get_time() / 1000000ULL);

            // Batterie-Messung (ADC)
            // TODO: Echter ADC-Read — hier Placeholder
            uint32_t battery_mv  = 3800; // Placeholder
            uint8_t  battery_pct = 70;   // Placeholder

            // Antwort: [0x07][0x00][fw_major][fw_minor][fw_patch][fw_build]
            //           [uptime:4LE][battery_mv:4LE][battery_pct:1]
            if (resp_max < 15) {
                resp_buf[1] = ADMIN_STATUS_ERROR; return true;
            }
            resp_buf[0] = cmd_id;
            resp_buf[1] = ADMIN_STATUS_OK;
            resp_buf[2] = FW_VERSION_MAJOR;
            resp_buf[3] = FW_VERSION_MINOR;
            resp_buf[4] = FW_VERSION_PATCH;
            resp_buf[5] = FW_VERSION_BUILD;
            resp_buf[6]  = (uptime_sec      ) & 0xFF;
            resp_buf[7]  = (uptime_sec >>  8) & 0xFF;
            resp_buf[8]  = (uptime_sec >> 16) & 0xFF;
            resp_buf[9]  = (uptime_sec >> 24) & 0xFF;
            resp_buf[10] = (battery_mv      ) & 0xFF;
            resp_buf[11] = (battery_mv >>  8) & 0xFF;
            resp_buf[12] = (battery_mv >> 16) & 0xFF;
            resp_buf[13] = (battery_mv >> 24) & 0xFF;
            resp_buf[14] = battery_pct;
            *resp_len = 15;
            break;
        }

        // ---- 0x08: REBOOT ----
        case ADMIN_CMD_REBOOT: {
            ESP_LOGI(TAG, "Reboot angefordert...");
            xTaskCreate(reboot_after_delay, "reboot", 1024, NULL, 5, NULL);
            break;
        }

        // ---- 0x09: FACTORY_RESET ----
        case ADMIN_CMD_FACTORY_RESET: {
            ESP_LOGW(TAG, "Factory Reset angefordert!");
            node_config_nuke();
            xTaskCreate(reboot_after_delay, "reboot", 1024, NULL, 5, NULL);
            break;
        }

        // ---- 0x0A: ENTER_DEEP_SLEEP ----
        case ADMIN_CMD_ENTER_DEEP_SLEEP: {
            uint32_t duration_sec = 0;
            if (params_len >= 4) {
                duration_sec = ((uint32_t)params[0])       |
                               ((uint32_t)params[1] << 8)  |
                               ((uint32_t)params[2] << 16) |
                               ((uint32_t)params[3] << 24);
            }
            ESP_LOGI(TAG, "Deep Sleep für %lu Sekunden", (unsigned long)duration_sec);
            // TODO: esp_deep_sleep implementieren
            // sx1262_sleep();
            // esp_deep_sleep(duration_sec == 0 ? 0 : (uint64_t)duration_sec * 1000000ULL);
            break;
        }

        // ---- 0x0B: SET_NODEINFO_INTERVAL ----
        case ADMIN_CMD_SET_NODEINFO_INTERVAL: {
            if (params_len < 4) {
                resp_buf[1] = ADMIN_STATUS_ERROR; return true;
            }
            uint32_t seconds = ((uint32_t)params[0])       |
                               ((uint32_t)params[1] << 8)  |
                               ((uint32_t)params[2] << 16) |
                               ((uint32_t)params[3] << 24);
            node_config_set_nodeinfo_interval(seconds);
            break;
        }

        // ---- 0x0C: NUKE ----
        case ADMIN_CMD_NUKE: {
            ESP_LOGW(TAG, "=== NUKE AUSGEFÜHRT ===");
            node_config_nuke();
            xTaskCreate(reboot_after_delay, "reboot", 1024, NULL, 5, NULL);
            break;
        }

        default: {
            ESP_LOGW(TAG, "Unbekanntes Admin-Kommando: 0x%02X", cmd_id);
            resp_buf[0] = cmd_id;
            resp_buf[1] = ADMIN_STATUS_UNKNOWN;
            *resp_len = 2;
            break;
        }
    }

    return true;
}
