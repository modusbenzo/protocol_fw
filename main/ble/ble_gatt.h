#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "mesh_proto.h"

// ============================================================
// PROTOCOL° Firmware — BLE GATT Service
// ============================================================
// Meshtastic-kompatible BLE UUIDs.
// Drei Characteristics: toRadio (Write), fromRadio (Read),
// fromNum (Notify).
// ============================================================

// GATT Service UUID: 6BA1B218-15A8-461F-9FA8-5DCAE273EAFD
// toRadio:   F75C76D2-129E-4DAD-A1DD-7866124401E7
// fromRadio: 2C55E69E-4993-11ED-B878-0242AC120002
// fromNum:   ED9DA18C-A800-4F66-A670-AA7547E34453

/**
 * @brief Initialisiert NimBLE Stack und registriert GATT Service.
 *        Startet BLE Advertising.
 */
void ble_gatt_init(void);

/**
 * @brief Stellt ein FromRadio-Paket in die Queue ein (für App).
 *        Löst fromNum-Notification aus wenn verbunden.
 * @param fr    FromRadio-Nachricht
 * @return true wenn erfolgreich eingereiht
 */
bool ble_gatt_enqueue_from_radio(const from_radio_t *fr);

/**
 * @brief Liefert true wenn BLE verbunden (App connectiert).
 */
bool ble_gatt_is_connected(void);

/**
 * @brief Startet BLE Advertising (nach Disconnect).
 */
void ble_gatt_start_advertising(void);
