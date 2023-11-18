///////////////////////////////////////////////////////////////////////
// File: gap.h
//
// Subject: Standardy i Systemy Komunikacyjne, AGH, EiT
//
// Author: Paweł Majewski
//
///////////////////////////////////////////////////////////////////////
#ifndef GAP_H
#define GAP_H

///////////////////////////////////////////////////////////////////////////////////
// HEDER FILE
///////////////////////////////////////////////////////////////////////////////////
#include "esp_gap_ble_api.h"
#include "ble_spp_server.h"

/**
 * @brief Bluetooth Generic Access Profile (GAP) event handler.
 *
 * This function handles events related to the Bluetooth Low Energy (BLE) Generic Access Profile.
 * Specifically, it responds to the completion event of setting raw advertising data by restarting advertising.
 *
 * @param[in] event   The specific GAP BLE callback event.
 * @param[in] param   Pointer to the callback parameters containing event-specific data.
 */
 void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

 #endif /* GAP_H */