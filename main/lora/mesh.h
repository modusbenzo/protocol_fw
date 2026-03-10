#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "mesh_proto.h"

// ============================================================
// PROTOCOL° Firmware — Mesh Layer
// ============================================================
// Flood-basiertes Mesh mit Hop-Limit und Duplikat-Erkennung.
// Opaker Transport für App-Payloads.
// Ghost Mode: keine autonomen Broadcasts.
// ============================================================

// Callback: Paket für die App (BLE fromRadio)
// Wird aufgerufen wenn Paket für uns (to==myNodeNum oder Broadcast)
typedef void (*mesh_rx_for_app_callback_t)(const mesh_packet_t *pkt);

/**
 * @brief Initialisiert den Mesh Layer.
 * @param my_node_num   Eigene 32-bit NodeNum
 * @param rx_cb         Callback für App-gebundene Pakete
 */
void mesh_init(uint32_t my_node_num, mesh_rx_for_app_callback_t rx_cb);

/**
 * @brief Verarbeitet ein empfangenes LoRa-Paket (Protobuf-Bytes).
 *        Führt Dedup, ACK, Relay und App-Callback durch.
 * @param raw_data  Empfangene Protobuf-Bytes
 * @param raw_len   Länge
 * @param rssi      Empfangsstärke
 * @param snr       Signal/Noise
 */
void mesh_on_rx(const uint8_t *raw_data, size_t raw_len,
                int16_t rssi, float snr);

/**
 * @brief Sendet ein Paket an einen Peer oder Broadcast.
 * @param to         Ziel NodeNum (0xFFFFFFFF = Broadcast)
 * @param payload    App-Payload (opak)
 * @param payload_len Länge (max 230 Bytes)
 * @param portnum    PortNum (normalerweise PORTNUM_TEXT_MESSAGE)
 * @param want_ack   ACK anfordern
 * @return true wenn Paket in TX-Queue eingereiht
 */
bool mesh_send(uint32_t to,
               const uint8_t *payload, size_t payload_len,
               portnum_t portnum, bool want_ack);

/**
 * @brief Sendet ein NodeInfo-Paket (nur wenn Ghost Mode INAKTIV).
 *        Wird von Timer oder App-Befehl aufgerufen.
 */
void mesh_send_nodeinfo(void);

/**
 * @brief Generiert eine neue eindeutige Paket-ID.
 */
uint32_t mesh_next_packet_id(void);

/**
 * @brief Prüft ob Ghost Mode aktiv ist (cached aus node_config).
 */
bool mesh_is_ghost_mode(void);

/**
 * @brief Aktualisiert Ghost Mode (nach BLE Admin-Kommando).
 */
void mesh_update_ghost_mode(bool enabled);
