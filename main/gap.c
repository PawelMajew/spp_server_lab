///////////////////////////////////////////////////////////////////////
// File: gap.c
//
// Subject: Standardy i Systemy Komunikacyjne, AGH, EiT
//
// Author: Paweł Majewski
//
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////
// HEDER FILE
///////////////////////////////////////////////////////////////////////////////////

#include "gap.h"

///////////////////////////////////////////////////////////////////////////////////
// LOCAL DATA
///////////////////////////////////////////////////////////////////////////////////
static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

///////////////////////////////////////////////////////////////////////////////////
// GLOBAL FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Bluetooth Generic Access Profile (GAP) event handler.
 *
 * This function handles events related to the Bluetooth Low Energy (BLE) Generic Access Profile.
 * Specifically, it responds to the completion event of setting raw advertising data by restarting advertising.
 *
 * @param[in] event   The specific GAP BLE callback event.
 * @param[in] param   Pointer to the callback parameters containing event-specific data.
 */
//ZAD_1

//TODO_1

///////////////////////////////////
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    default:
        break;
    }
}