#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "mesh_proto.h"

// ============================================================
// PROTOCOL° Firmware — Node Configuration (NVS-persistent)
// ============================================================

// LoRa Frequenzregion
typedef enum {
    REGION_EU868 = 0,
    REGION_US915 = 1,
    REGION_AU915 = 2,
    REGION_AS923 = 3,
} lora_region_t;

// Vollständige Node-Konfiguration
typedef struct {
    uint32_t       node_num;                          // 32-bit NodeNum
    char           long_name[PROTO_MAX_LONG_NAME + 1];
    char           short_name[PROTO_MAX_SHORT_NAME + 1];
    bool           ghost_mode;                        // Ghost Mode aktiv?
    lora_region_t  region;
    uint8_t        tx_power_dbm;
    uint8_t        hop_limit;
    uint32_t       nodeinfo_interval_sec;             // 0 = nie senden
    uint32_t       reboot_count;
    hw_model_t     hw_model;
} node_config_t;

/**
 * @brief Initialisiert NVS und lädt Konfiguration.
 *        Wenn keine Config vorhanden → Defaults mit zufälliger NodeNum.
 */
void node_config_init(void);

/**
 * @brief Liefert Zeiger auf aktuelle (RAM-)Konfiguration.
 */
const node_config_t *node_config_get(void);

/**
 * @brief Speichert geänderte Konfiguration in NVS.
 */
void node_config_save(void);

/**
 * @brief Setzt Ghost Mode und speichert.
 */
void node_config_set_ghost_mode(bool enabled);

/**
 * @brief Setzt long_name und speichert.
 */
void node_config_set_long_name(const char *name);

/**
 * @brief Setzt short_name und speichert.
 */
void node_config_set_short_name(const char *name);

/**
 * @brief Setzt LoRa-Region und speichert.
 */
void node_config_set_region(lora_region_t region);

/**
 * @brief Setzt TX-Power und speichert.
 */
void node_config_set_tx_power(uint8_t dbm);

/**
 * @brief Setzt Hop-Limit und speichert.
 */
void node_config_set_hop_limit(uint8_t hops);

/**
 * @brief Setzt NodeInfo-Broadcast-Intervall und speichert.
 */
void node_config_set_nodeinfo_interval(uint32_t seconds);

/**
 * @brief Inkrementiert Reboot-Counter und speichert.
 */
void node_config_increment_reboot_count(void);

/**
 * @brief Generiert neue zufällige NodeNum (bei Kollision).
 */
void node_config_regenerate_node_num(void);

/**
 * @brief Löscht ALLE NVS-Daten (NUKE-Funktion).
 */
void node_config_nuke(void);

/**
 * @brief Gibt Meshtastic-kompatible Node-ID zurück ("!xxxxxxxx").
 */
void node_config_get_node_id(char *buf, size_t buf_size);
