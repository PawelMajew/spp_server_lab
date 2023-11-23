///////////////////////////////////////////////////////////////////////
// File: ble_spp_server.c
//
// Subject: Standardy i Systemy Komunikacyjne, AGH, EiT
//
// Author: Paweł Majewski
//
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////
// HEDER FILE
///////////////////////////////////////////////////////////////////////////////////

#include "ble_spp_server.h"

#include "gatt.h"
#include "gap.h"

void app_main(void)
{
    // Initialize NVS.
    nvs_flash_init();

    // Bluetooth controller initialization with default config.
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);

    // Turning on the controller in ble mode.
    //ZAD_1
    esp_bt_controller_enable(//TODO_1);
    ///////////////////////////////////

    // Init and alloc the resource for bluetooth.
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_bt_main.html
    //ZAD_1

    //TODO_1

    ///////////////////////////////////

    // Enables the BlueDroid Bluetooth stack after it initializes.
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_bt_main.html
    //ZAD_1

    //TODO_1

    ///////////////////////////////////

    // Registering a function that supports gatt.
    esp_ble_gatts_register_callback(gatts_event_handler);

    // Registers a gap handling function.
    esp_ble_gap_register_callback(gap_event_handler);

    // Gatt application registration. (arg = ESP_SPP_APP_ID)
    //https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_gatts.html
    //ZAD_1

    //TODO_1

    ///////////////////////////////////

    // Task initialization SPP.
    spp_task_init();

    return;
}
