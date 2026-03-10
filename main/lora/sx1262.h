#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "node_config.h"

// ============================================================
// PROTOCOL° Firmware — SX1262 LoRa Driver
// ============================================================

#define SX1262_MAX_PAYLOAD 255

// Bandbreite Enum (SX1262 Register-Werte)
typedef enum {
    LORA_BW_7   = 0x00,  // 7.8 kHz
    LORA_BW_10  = 0x08,
    LORA_BW_15  = 0x01,
    LORA_BW_20  = 0x09,
    LORA_BW_31  = 0x02,
    LORA_BW_41  = 0x0A,
    LORA_BW_62  = 0x03,
    LORA_BW_125 = 0x04,  // 125 kHz (Standard)
    LORA_BW_250 = 0x05,
    LORA_BW_500 = 0x06,
} lora_bw_t;

// Coding Rate Enum
typedef enum {
    LORA_CR_4_5 = 1,
    LORA_CR_4_6 = 2,
    LORA_CR_4_7 = 3,
    LORA_CR_4_8 = 4,  // Maximum Error Correction
} lora_cr_t;

// RX-Callback — wird aus ISR-Kontext aufgerufen
// data:    Empfangene Bytes (MeshPacket Protobuf)
// len:     Länge
// rssi:    RSSI in dBm
// snr:     SNR in dB (float)
typedef void (*sx1262_rx_callback_t)(const uint8_t *data, size_t len,
                                      int16_t rssi, float snr);

/**
 * @brief Initialisiert SPI und SX1262 Hardware.
 *        Konfiguriert LoRa-Modulation und startet RX-Modus.
 * @param region    Frequenzregion (bestimmt Mittenfrequenz)
 * @param rx_cb     Callback bei empfangenem Paket (aus ISR-Task)
 * @return true bei Erfolg
 */
bool sx1262_init(lora_region_t region, sx1262_rx_callback_t rx_cb);

/**
 * @brief Sendet Daten über LoRa (blockierend bis TX Done).
 * @param data      Zu sendende Bytes
 * @param len       Länge (max SX1262_MAX_PAYLOAD)
 * @return true bei Erfolg
 */
bool sx1262_transmit(const uint8_t *data, size_t len);

/**
 * @brief Schaltet SX1262 in kontinuierlichen RX-Modus.
 */
void sx1262_start_rx(void);

/**
 * @brief Setzt TX-Power.
 * @param dbm   Sendeleistung in dBm (0-22)
 */
void sx1262_set_tx_power(uint8_t dbm);

/**
 * @brief Setzt Spreading Factor (7-12).
 */
void sx1262_set_sf(uint8_t sf);

/**
 * @brief Setzt Frequenz für Region.
 */
void sx1262_set_region(lora_region_t region);

/**
 * @brief Liefert zuletzt gemessene RSSI des letzten Pakets.
 */
int16_t sx1262_get_last_rssi(void);

/**
 * @brief Liefert zuletzt gemessenes SNR.
 */
float sx1262_get_last_snr(void);

/**
 * @brief Setzt SX1262 in Sleep-Modus (mit DIO1-Wake fähig).
 */
void sx1262_sleep(void);
