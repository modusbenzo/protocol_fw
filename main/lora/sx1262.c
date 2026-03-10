#include "sx1262.h"
#include "board_pins.h"
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <string.h>
#include <math.h>

static const char *TAG = "sx1262";

// ============================================================
// SX1262 Opcodes
// ============================================================
#define CMD_SET_SLEEP                   0x84
#define CMD_SET_STANDBY                 0x80
#define CMD_SET_FS                      0xC1
#define CMD_SET_TX                      0x83
#define CMD_SET_RX                      0x82
#define CMD_STOP_TIMER_ON_PREAMBLE      0x9F
#define CMD_SET_RX_DUTY_CYCLE           0x94
#define CMD_SET_CAD                     0xC5
#define CMD_SET_TX_CONTINUOUS_WAVE      0xD1
#define CMD_SET_TX_INFINITE_PREAMBLE    0xD2
#define CMD_SET_REGULATOR_MODE          0x96
#define CMD_CALIBRATE                   0x89
#define CMD_CALIBRATE_IMAGE             0x98
#define CMD_SET_PA_CONFIG               0x95
#define CMD_SET_RX_TX_FALLBACK_MODE     0x93
#define CMD_WRITE_REGISTER              0x0D
#define CMD_READ_REGISTER               0x1D
#define CMD_WRITE_BUFFER                0x0E
#define CMD_READ_BUFFER                 0x1E
#define CMD_SET_DIO_IRQ_PARAMS          0x08
#define CMD_GET_IRQ_STATUS              0x12
#define CMD_CLEAR_IRQ_STATUS            0x02
#define CMD_SET_DIO2_AS_RF_SWITCH       0x9D
#define CMD_SET_DIO3_AS_TCXO_CTRL       0x97
#define CMD_SET_RF_FREQUENCY            0x86
#define CMD_SET_PACKET_TYPE             0x8A
#define CMD_GET_PACKET_TYPE             0x11
#define CMD_SET_TX_PARAMS               0x8E
#define CMD_SET_MODULATION_PARAMS       0x8B
#define CMD_SET_PACKET_PARAMS           0x8C
#define CMD_GET_RX_BUFFER_STATUS        0x13
#define CMD_GET_PACKET_STATUS           0x14
#define CMD_GET_RSSI_INST               0x15
#define CMD_GET_STATS                   0x10
#define CMD_RESET_STATS                 0x00
#define CMD_GET_STATUS                  0xC0
#define CMD_SET_FS2                     0xC1

// IRQ Bits
#define IRQ_TX_DONE                 (1 << 0)
#define IRQ_RX_DONE                 (1 << 1)
#define IRQ_PREAMBLE_DETECTED       (1 << 2)
#define IRQ_SYNC_WORD_VALID         (1 << 3)
#define IRQ_HEADER_VALID            (1 << 4)
#define IRQ_HEADER_ERR              (1 << 5)
#define IRQ_CRC_ERR                 (1 << 6)
#define IRQ_CAD_DONE                (1 << 7)
#define IRQ_CAD_DETECTED            (1 << 8)
#define IRQ_TIMEOUT                 (1 << 9)

// Packet type
#define PACKET_TYPE_LORA            0x01

// Standby config
#define STDBY_RC                    0x00
#define STDBY_XOSC                  0x01

// ============================================================
// State
// ============================================================
static spi_device_handle_t  s_spi_dev;
static sx1262_rx_callback_t s_rx_callback;
static int16_t              s_last_rssi;
static float                s_last_snr;
static SemaphoreHandle_t    s_tx_done_sem;
static TaskHandle_t         s_rx_task_handle;
static uint32_t             s_current_freq_hz = LORA_FREQ_HZ;

// ============================================================
// SPI Low-Level
// ============================================================

static void spi_wait_busy(void) {
    // BUSY-Pin muss LOW sein bevor jede Transaktion
    uint32_t timeout = 10000;
    while (gpio_get_level(LORA_BUSY_PIN) && timeout--) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (!timeout) {
        ESP_LOGW(TAG, "SX1262 BUSY timeout");
    }
}

static void spi_write_cmd(uint8_t cmd, const uint8_t *data, size_t len) {
    spi_wait_busy();

    spi_transaction_t t = {
        .cmd   = cmd,
        .length = len * 8,
        .tx_buffer = (len > 0) ? data : NULL,
        .flags = SPI_TRANS_USE_TXDATA * (len == 0),
    };
    if (len > 0 && len <= 4) {
        memcpy(t.tx_data, data, len);
        t.flags = SPI_TRANS_USE_TXDATA;
        t.tx_buffer = NULL;
    }

    // Für längere Daten tx_buffer verwenden
    spi_transaction_t t2;
    if (len > 4) {
        memset(&t2, 0, sizeof(t2));
        t2.length = (1 + len) * 8;
        uint8_t *buf = heap_caps_malloc(1 + len, MALLOC_CAP_DMA);
        if (!buf) return;
        buf[0] = cmd;
        memcpy(buf + 1, data, len);
        t2.tx_buffer = buf;
        spi_device_polling_transmit(s_spi_dev, &t2);
        free(buf);
        return;
    }

    uint8_t cmd_buf = cmd;
    spi_transaction_t t3 = {0};
    t3.length = (1 + len) * 8;
    uint8_t tmp[64];
    tmp[0] = cmd;
    if (len > 0 && data) memcpy(tmp + 1, data, len);
    t3.tx_buffer = tmp;
    spi_device_polling_transmit(s_spi_dev, &t3);
}

static void spi_read_cmd(uint8_t cmd, uint8_t *rx, size_t rx_len) {
    spi_wait_busy();

    // SX1262: [cmd][status_byte][data...]
    size_t total = 1 + 1 + rx_len; // cmd + status + data
    uint8_t *tx_buf = heap_caps_calloc(total, 1, MALLOC_CAP_DMA);
    uint8_t *rx_buf = heap_caps_calloc(total, 1, MALLOC_CAP_DMA);
    if (!tx_buf || !rx_buf) {
        free(tx_buf); free(rx_buf);
        return;
    }
    tx_buf[0] = cmd;

    spi_transaction_t t = {
        .length    = total * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };
    spi_device_polling_transmit(s_spi_dev, &t);
    // rx_buf[0] = status, rx_buf[1] = status byte 2, rx_buf[2+] = data
    memcpy(rx, rx_buf + 2, rx_len);

    free(tx_buf);
    free(rx_buf);
}

static void write_register(uint16_t addr, uint8_t value) {
    uint8_t data[] = { (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF),
                       value };
    spi_write_cmd(CMD_WRITE_REGISTER, data, 3);
}

static uint8_t read_register(uint16_t addr) {
    // [cmd][addr_h][addr_l][nop][data]
    uint8_t total[5] = { CMD_READ_REGISTER,
                         (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF),
                         0x00, 0x00 };
    uint8_t rx[5] = {0};
    spi_wait_busy();
    spi_transaction_t t = {
        .length    = 5 * 8,
        .tx_buffer = total,
        .rx_buffer = rx,
    };
    spi_device_polling_transmit(s_spi_dev, &t);
    return rx[4];
}

// ============================================================
// DIO1 IRQ Handler
// ============================================================

static void IRAM_ATTR dio1_isr_handler(void *arg) {
    TaskHandle_t task = (TaskHandle_t)arg;
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(task, &woken);
    portYIELD_FROM_ISR(woken);
}

// ============================================================
// RX Task — wartet auf DIO1-Notification, liest Paket
// ============================================================

static void rx_task(void *arg) {
    while (1) {
        // Warte auf DIO1-IRQ (ulTaskNotifyTake blockiert)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // IRQ Status lesen
        uint8_t irq_rx[2];
        spi_read_cmd(CMD_GET_IRQ_STATUS, irq_rx, 2);
        uint16_t irq_status = ((uint16_t)irq_rx[0] << 8) | irq_rx[1];

        // IRQ löschen
        uint8_t clr[] = { (uint8_t)(irq_status >> 8),
                          (uint8_t)(irq_status & 0xFF) };
        spi_write_cmd(CMD_CLEAR_IRQ_STATUS, clr, 2);

        if (irq_status & IRQ_TX_DONE) {
            // TX fertig — Semaphor freigeben
            xSemaphoreGive(s_tx_done_sem);
            // Zurück in RX
            sx1262_start_rx();
        }

        if (irq_status & IRQ_RX_DONE) {
            // RX Buffer Status lesen
            uint8_t buf_status[2];
            spi_read_cmd(CMD_GET_RX_BUFFER_STATUS, buf_status, 2);
            uint8_t payload_len = buf_status[0];
            uint8_t buf_offset  = buf_status[1];

            if (payload_len > 0 && payload_len <= SX1262_MAX_PAYLOAD) {
                // Paket aus Buffer lesen
                // [CMD_READ_BUFFER][offset][nop][data...]
                size_t  total    = 2 + 1 + payload_len;
                uint8_t *tx_buf  = heap_caps_calloc(total, 1, MALLOC_CAP_DMA);
                uint8_t *rx_buf  = heap_caps_calloc(total, 1, MALLOC_CAP_DMA);

                if (tx_buf && rx_buf) {
                    tx_buf[0] = CMD_READ_BUFFER;
                    tx_buf[1] = buf_offset;
                    tx_buf[2] = 0x00; // NOP

                    spi_wait_busy();
                    spi_transaction_t t = {
                        .length    = total * 8,
                        .tx_buffer = tx_buf,
                        .rx_buffer = rx_buf,
                    };
                    spi_device_polling_transmit(s_spi_dev, &t);

                    // Paket-Status (RSSI, SNR)
                    uint8_t pkt_status[3];
                    spi_read_cmd(CMD_GET_PACKET_STATUS, pkt_status, 3);
                    // pkt_status[0] = RssiPkt, [1] = SnrPkt, [2] = SignalRssiPkt
                    s_last_rssi = -(int16_t)pkt_status[0] / 2;
                    s_last_snr  = (float)(int8_t)pkt_status[1] / 4.0f;

                    uint8_t *payload = rx_buf + 3; // skip cmd+offset+nop
                    ESP_LOGD(TAG, "RX: %u Bytes RSSI=%d SNR=%.1f",
                             payload_len, s_last_rssi, s_last_snr);

                    if (s_rx_callback) {
                        s_rx_callback(payload, payload_len,
                                      s_last_rssi, s_last_snr);
                    }
                }
                free(tx_buf);
                free(rx_buf);
            }

            if (!(irq_status & IRQ_TX_DONE)) {
                // Nach RX: zurück in RX-Modus
                sx1262_start_rx();
            }
        }

        if (irq_status & IRQ_CRC_ERR) {
            ESP_LOGW(TAG, "CRC Error — Paket verworfen");
            sx1262_start_rx();
        }

        if (irq_status & IRQ_HEADER_ERR) {
            ESP_LOGW(TAG, "Header Error");
            sx1262_start_rx();
        }
    }
}

// ============================================================
// Frequenzberechnung
// ============================================================

static uint32_t region_to_freq_hz(lora_region_t region) {
    switch (region) {
        case REGION_EU868: return 868100000UL;
        case REGION_US915: return 915000000UL;
        case REGION_AU915: return 915000000UL;
        case REGION_AS923: return 923200000UL;
        default:           return 868100000UL;
    }
}

static void set_rf_frequency(uint32_t freq_hz) {
    // PLL Step: freq_hz / (32e6 / 2^25)
    // = freq_hz * 33554432 / 32000000
    uint64_t pll = ((uint64_t)freq_hz << 25) / 32000000ULL;
    uint8_t params[] = {
        (uint8_t)(pll >> 24),
        (uint8_t)(pll >> 16),
        (uint8_t)(pll >> 8),
        (uint8_t)(pll)
    };
    spi_write_cmd(CMD_SET_RF_FREQUENCY, params, 4);
    s_current_freq_hz = freq_hz;
    ESP_LOGI(TAG, "Frequenz: %lu Hz", (unsigned long)freq_hz);
}

// ============================================================
// Init
// ============================================================

bool sx1262_init(lora_region_t region, sx1262_rx_callback_t rx_cb) {
    s_rx_callback = rx_cb;
    s_tx_done_sem = xSemaphoreCreateBinary();

    // --- GPIO Setup ---
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << LORA_RST_PIN) | (1ULL << LORA_CS_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    // BUSY als Input
    gpio_config_t busy_io = {
        .pin_bit_mask = (1ULL << LORA_BUSY_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&busy_io);

    // DIO1 als Input mit IRQ
    gpio_config_t dio_io = {
        .pin_bit_mask = (1ULL << LORA_DIO1_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    gpio_config(&dio_io);

    gpio_set_level(LORA_CS_PIN, 1);

    // --- SPI Init ---
    spi_bus_config_t spi_bus = {
        .mosi_io_num   = LORA_MOSI_PIN,
        .miso_io_num   = LORA_MISO_PIN,
        .sclk_io_num   = LORA_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 512,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LORA_SPI_HOST, &spi_bus, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = LORA_SPI_SPEED,
        .mode           = 0,   // SX1262: CPOL=0, CPHA=0
        .spics_io_num   = LORA_CS_PIN,
        .queue_size     = 8,
        .flags          = 0,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(LORA_SPI_HOST, &dev_cfg, &s_spi_dev));

    // --- RX Task + DIO1 ISR ---
    xTaskCreatePinnedToCore(rx_task, "lora_rx", 4096, NULL, 10,
                             &s_rx_task_handle, 1); // Core 1

    gpio_install_isr_service(0);
    gpio_isr_handler_add(LORA_DIO1_PIN, dio1_isr_handler,
                          (void *)s_rx_task_handle);

    // --- SX1262 Hardware Reset ---
    gpio_set_level(LORA_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(LORA_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Warte bis BUSY low
    uint32_t timeout = 5000;
    while (gpio_get_level(LORA_BUSY_PIN) && timeout--) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // --- Konfiguration ---

    // Standby RC
    uint8_t stdby = STDBY_RC;
    spi_write_cmd(CMD_SET_STANDBY, &stdby, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // DIO3 als TCXO Referenz (Heltec V3/V4 hat TCXO)
    uint8_t tcxo[] = { 0x07, 0x00, 0x00, 0x64 }; // 1.7V, 100ms Timeout
    spi_write_cmd(CMD_SET_DIO3_AS_TCXO_CTRL, tcxo, 4);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Calibrierung
    uint8_t calib = 0x7F; // Alle Blöcke kalibrieren
    spi_write_cmd(CMD_CALIBRATE, &calib, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Standby XOSC (für LoRa)
    uint8_t stdby_xosc = STDBY_XOSC;
    spi_write_cmd(CMD_SET_STANDBY, &stdby_xosc, 1);

    // DIO2 als RF Switch
    uint8_t rfswitch = 0x01;
    spi_write_cmd(CMD_SET_DIO2_AS_RF_SWITCH, &rfswitch, 1);

    // Packet Type: LoRa
    uint8_t pkt_type = PACKET_TYPE_LORA;
    spi_write_cmd(CMD_SET_PACKET_TYPE, &pkt_type, 1);

    // Frequenz setzen
    set_rf_frequency(region_to_freq_hz(region));

    // Image Kalibrierung für Region
    uint8_t img_cal[2];
    switch (region) {
        case REGION_EU868: img_cal[0] = 0xD7; img_cal[1] = 0xDB; break;
        case REGION_US915:
        case REGION_AU915: img_cal[0] = 0xE1; img_cal[1] = 0xE9; break;
        case REGION_AS923: img_cal[0] = 0xE1; img_cal[1] = 0xE9; break;
        default:           img_cal[0] = 0xD7; img_cal[1] = 0xDB; break;
    }
    spi_write_cmd(CMD_CALIBRATE_IMAGE, img_cal, 2);
    vTaskDelay(pdMS_TO_TICKS(10));

    // PA Config (SX1262, max 22 dBm)
    uint8_t pa_cfg[] = { 0x04, 0x07, 0x00, 0x01 };
    spi_write_cmd(CMD_SET_PA_CONFIG, pa_cfg, 4);

    // TX Params: TX Power + Ramp Time
    uint8_t tx_params[] = {
        (uint8_t)LORA_DEFAULT_TX_POWER,
        0x04  // 200µs Ramp
    };
    spi_write_cmd(CMD_SET_TX_PARAMS, tx_params, 2);

    // Modulation: SF, BW, CR, LowDataRateOptimize
    // LDRO = 1 wenn SF11/SF12 mit BW125
    uint8_t ldro = (LORA_DEFAULT_SF >= 11) ? 1 : 0;
    uint8_t mod_params[] = {
        LORA_DEFAULT_SF,    // Spreading Factor
        LORA_DEFAULT_BW,    // Bandwidth
        LORA_DEFAULT_CR,    // Coding Rate
        ldro,               // Low Data Rate Optimize
    };
    spi_write_cmd(CMD_SET_MODULATION_PARAMS, mod_params, 4);

    // Packet Params: Preamble, Header, Payload, CRC, IQ
    uint8_t pkt_params[] = {
        (uint8_t)(LORA_DEFAULT_PREAMBLE >> 8), // PreambleLength MSB
        (uint8_t)(LORA_DEFAULT_PREAMBLE),       // PreambleLength LSB
        0x00,   // Header: EXPLICIT
        0xFF,   // PayloadLength (max, wird beim Senden gesetzt)
        0x01,   // CRC: ON
        0x00,   // Invert IQ: Standard
    };
    spi_write_cmd(CMD_SET_PACKET_PARAMS, pkt_params, 6);

    // Sync Word (Meshtastic Private Network: 0x2724)
    // SX1262 Register 0x0740 (MSB) + 0x0741 (LSB)
    // Meshtastic: 0x2B → ergibt 0x2724
    write_register(0x0740, 0x27);
    write_register(0x0741, 0x1C);  // Meshtastic Sync Word

    // IRQ Konfiguration: TX Done, RX Done, CRC Err, Header Err auf DIO1
    uint8_t irq_params[] = {
        0x02, 0xC3,  // IRQ Mask: TX Done + RX Done + Header Err + CRC Err
        0x02, 0xC3,  // DIO1 Mask
        0x00, 0x00,  // DIO2 Mask
        0x00, 0x00,  // DIO3 Mask
    };
    spi_write_cmd(CMD_SET_DIO_IRQ_PARAMS, irq_params, 8);

    // Fallback nach TX/RX: zurück in STDBY_RC
    uint8_t fallback = 0x20; // FS Mode nach TX, STDBY nach RX
    spi_write_cmd(CMD_SET_RX_TX_FALLBACK_MODE, &fallback, 1);

    ESP_LOGI(TAG, "SX1262 initialisiert (SF%d BW125 CR4/8 Preamble%d)",
             LORA_DEFAULT_SF, LORA_DEFAULT_PREAMBLE);

    // Starte kontinuierlichen RX
    sx1262_start_rx();

    return true;
}

// ============================================================
// TX
// ============================================================

bool sx1262_transmit(const uint8_t *data, size_t len) {
    if (len == 0 || len > SX1262_MAX_PAYLOAD) return false;

    // In Standby gehen
    uint8_t stdby = STDBY_RC;
    spi_write_cmd(CMD_SET_STANDBY, &stdby, 1);
    vTaskDelay(pdMS_TO_TICKS(2));

    // Payload Length in Packet Params setzen
    uint8_t pkt_params[] = {
        (uint8_t)(LORA_DEFAULT_PREAMBLE >> 8),
        (uint8_t)(LORA_DEFAULT_PREAMBLE),
        0x00,           // Explicit Header
        (uint8_t)len,   // Payload Length
        0x01,           // CRC ON
        0x00,           // Normal IQ
    };
    spi_write_cmd(CMD_SET_PACKET_PARAMS, pkt_params, 6);

    // Daten in TX Buffer schreiben
    // [CMD_WRITE_BUFFER][offset=0][data...]
    size_t  total   = 1 + 1 + len;
    uint8_t *tx_buf = heap_caps_malloc(total, MALLOC_CAP_DMA);
    if (!tx_buf) return false;
    tx_buf[0] = CMD_WRITE_BUFFER;
    tx_buf[1] = 0x00; // Offset
    memcpy(tx_buf + 2, data, len);

    spi_wait_busy();
    spi_transaction_t t = {
        .length    = total * 8,
        .tx_buffer = tx_buf,
    };
    spi_device_polling_transmit(s_spi_dev, &t);
    free(tx_buf);

    // TX starten (Timeout: 5 Sekunden in SX1262 Time Units)
    // Time Unit = 15.625µs → 5s = 320000 * 15.625µs
    uint8_t tx_params[] = { 0x00, 0x4E, 0x20 }; // ~5 Sekunden
    spi_write_cmd(CMD_SET_TX, tx_params, 3);

    ESP_LOGD(TAG, "TX: %u Bytes gesendet", (unsigned)len);

    // Warte auf TX Done (max 5s)
    if (xSemaphoreTake(s_tx_done_sem, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(TAG, "TX Timeout!");
        sx1262_start_rx();
        return false;
    }

    return true;
}

// ============================================================
// RX
// ============================================================

void sx1262_start_rx(void) {
    // Kontinuierlicher RX: Timeout = 0xFFFFFF = kein Timeout
    uint8_t rx_params[] = { 0xFF, 0xFF, 0xFF };
    spi_write_cmd(CMD_SET_RX, rx_params, 3);
}

// ============================================================
// Config Änderungen
// ============================================================

void sx1262_set_tx_power(uint8_t dbm) {
    if (dbm > LORA_MAX_TX_POWER_DBM) dbm = LORA_MAX_TX_POWER_DBM;
    uint8_t tx_params[] = { dbm, 0x04 };
    spi_write_cmd(CMD_SET_TX_PARAMS, tx_params, 2);
    ESP_LOGI(TAG, "TX Power: %d dBm", dbm);
}

void sx1262_set_sf(uint8_t sf) {
    if (sf < 7) sf = 7;
    if (sf > 12) sf = 12;
    uint8_t ldro = (sf >= 11) ? 1 : 0;
    uint8_t mod_params[] = { sf, LORA_DEFAULT_BW, LORA_DEFAULT_CR, ldro };
    spi_write_cmd(CMD_SET_MODULATION_PARAMS, mod_params, 4);
}

void sx1262_set_region(lora_region_t region) {
    set_rf_frequency(region_to_freq_hz(region));
}

int16_t sx1262_get_last_rssi(void) { return s_last_rssi; }
float   sx1262_get_last_snr(void)  { return s_last_snr;  }

void sx1262_sleep(void) {
    // Warm Sleep: behält Konfiguration
    uint8_t sleep_cfg = 0x04;
    spi_write_cmd(CMD_SET_SLEEP, &sleep_cfg, 1);
}
