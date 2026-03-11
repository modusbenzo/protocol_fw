#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "sx1262.h"

// ============================================================
// PROTOCOL° Firmware — Admin Command Handler
// ============================================================
// Verarbeitet eingehende Admin-Kommandos (portnum=32 an myNode)
// und gibt Antwort-Bytes zurück.
// ============================================================

// Admin Kommando IDs (wie in Spec)
#define ADMIN_CMD_SET_GHOST_MODE        0x01
#define ADMIN_CMD_SET_LONG_NAME         0x02
#define ADMIN_CMD_SET_SHORT_NAME        0x03
#define ADMIN_CMD_SET_REGION            0x04
#define ADMIN_CMD_SET_TX_POWER          0x05
#define ADMIN_CMD_SET_HOP_LIMIT         0x06
#define ADMIN_CMD_GET_DEVICE_INFO       0x07
#define ADMIN_CMD_REBOOT                0x08
#define ADMIN_CMD_FACTORY_RESET         0x09
#define ADMIN_CMD_ENTER_DEEP_SLEEP      0x0A
#define ADMIN_CMD_SET_NODEINFO_INTERVAL 0x0B
#define ADMIN_CMD_NUKE                  0x0C

// Admin Response Status
#define ADMIN_STATUS_OK                 0x00
#define ADMIN_STATUS_ERROR              0x01
#define ADMIN_STATUS_UNKNOWN            0x02

/**
 * @brief Verarbeitet einen Admin-Payload.
 * @param payload       Admin-Kommando-Bytes [cmdId:1][params:N]
 * @param payload_len   Länge des Payloads
 * @param resp_buf      Ausgabepuffer für Antwort
 * @param resp_max      Max Antwortgröße
 * @param resp_len      Tatsächliche Antwortlänge
 * @return true wenn Antwort zu senden ist
 */
bool admin_handle_command(const uint8_t *payload, size_t payload_len,
                           uint8_t *resp_buf, size_t resp_max,
                           size_t *resp_len);