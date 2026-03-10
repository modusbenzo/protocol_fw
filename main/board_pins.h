#pragma once

// ============================================================
// PROTOCOL° Firmware — Board Pin Definitions
// ============================================================
// Wechsle das #define BOARD_xxx um ein anderes Board zu targeten.
// ============================================================

#define BOARD_HELTEC_V3   1
#define BOARD_HELTEC_V4   2
#define BOARD_TBEAM_S3    3

// --- Aktives Board ---
#ifndef BOARD_TARGET
#define BOARD_TARGET BOARD_HELTEC_V3
#endif

// ============================================================
// Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262)
// ============================================================
#if BOARD_TARGET == BOARD_HELTEC_V3

#define LORA_SCK_PIN    9
#define LORA_MISO_PIN   11
#define LORA_MOSI_PIN   10
#define LORA_CS_PIN     8
#define LORA_RST_PIN    12
#define LORA_DIO1_PIN   14    // RX Done / TX Done IRQ
#define LORA_BUSY_PIN   13

#define OLED_SDA_PIN    17
#define OLED_SCL_PIN    18
#define OLED_RST_PIN    21
#define OLED_VEXT_PIN   36    // VEXT für OLED Power

#define VBAT_ADC_PIN    1     // ADC1_CH0 — Batteriespannung
#define VBAT_DIV_FACTOR 2.0f  // Spannungsteiler 1:2

#define USER_BUTTON_PIN 0     // BOOT Button

#define LORA_FREQ_HZ    868000000UL  // EU868 Default

// ============================================================
// Heltec LoRa32 V4 (ESP32-S3 + SX1262) — neueres Board
// ============================================================
#elif BOARD_TARGET == BOARD_HELTEC_V4

#define LORA_SCK_PIN    9
#define LORA_MISO_PIN   11
#define LORA_MOSI_PIN   10
#define LORA_CS_PIN     8
#define LORA_RST_PIN    5
#define LORA_DIO1_PIN   14
#define LORA_BUSY_PIN   13

#define OLED_SDA_PIN    17
#define OLED_SCL_PIN    18
#define OLED_RST_PIN    21
#define OLED_VEXT_PIN   36

#define VBAT_ADC_PIN    1
#define VBAT_DIV_FACTOR 2.0f

#define USER_BUTTON_PIN 0

#define LORA_FREQ_HZ    868000000UL

// ============================================================
// LilyGO T-Beam S3 (geplant)
// ============================================================
#elif BOARD_TARGET == BOARD_TBEAM_S3

#define LORA_SCK_PIN    12
#define LORA_MISO_PIN   13
#define LORA_MOSI_PIN   11
#define LORA_CS_PIN     10
#define LORA_RST_PIN    5
#define LORA_DIO1_PIN   1
#define LORA_BUSY_PIN   4

#define VBAT_ADC_PIN    35
#define VBAT_DIV_FACTOR 2.0f

#define USER_BUTTON_PIN 38

#define LORA_FREQ_HZ    868000000UL

#else
#error "Kein gültiges BOARD_TARGET definiert!"
#endif

// ============================================================
// SPI Host
// ============================================================
#define LORA_SPI_HOST   SPI2_HOST
#define LORA_SPI_SPEED  8000000   // 8 MHz

// ============================================================
// LoRa Default Parameter (können via BLE überschrieben werden)
// ============================================================
#define LORA_DEFAULT_SF         11
#define LORA_DEFAULT_BW         LORA_BW_125
#define LORA_DEFAULT_CR         LORA_CR_4_8
#define LORA_DEFAULT_PREAMBLE   16
#define LORA_DEFAULT_SYNC_WORD  0x2B   // Meshtastic Private
#define LORA_DEFAULT_TX_POWER   14     // dBm (EU868 max)
#define LORA_MAX_TX_POWER_DBM   22     // Absolutes Maximum
