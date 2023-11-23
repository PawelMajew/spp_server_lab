///////////////////////////////////////////////////////////////////////
// File: ble_spp_server.c
//
// Subject: Standardy i Systemy Komunikacyjne, AGH, EiT
//
// Author: Paweł Majewski
//
///////////////////////////////////////////////////////////////////////
#ifndef BLE_SPP_SERVER
#define BLE_SPP_SERVER

///////////////////////////////////////////////////////////////////////////////////
// HEDER FILE
///////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "driver/uart.h"
#include "string.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

#define ESP_SPP_APP_ID              0x56
#define SPP_DATA_MAX_LEN           (512)
#define SPP_CMD_MAX_LEN            (20)
#define SPP_STATUS_MAX_LEN         (20)
#define SPP_DATA_BUFF_MAX_LEN      (2*1024)

#define GATTS_TABLE_TAG  "GATTS_SPP"

#define SAMPLE_DEVICE_NAME          "ESP_SPP_SERVER"    //The Device Name Characteristics in GAP
#define SPP_SVC_INST_ID             0
#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
/// Characteristic UUID
#define ESP_GATT_UUID_SPP_DATA_RECEIVE      0xABF1
#define ESP_GATT_UUID_SPP_DATA_NOTIFY       0xABF2
#define ESP_GATT_UUID_SPP_COMMAND_RECEIVE   0xABF3
#define ESP_GATT_UUID_SPP_COMMAND_NOTIFY    0xABF4

//ZAD_2 add new characteristic UUID

//TODO_2

///////////////////////////////////
/**
 * @brief Structure representing a node in the buffer for received SPP data.
 *
 * The structure includes information about the data length, a pointer to the data buffer,
 * and a pointer to the next node in the buffer. It is used to store individual pieces
 * of data received over the Serial Port Profile (SPP).
 */
typedef struct spp_receive_data_node{
    int32_t len;
    uint8_t * node_buff;
    struct spp_receive_data_node * next_node;
}spp_receive_data_node_t;

/**
 * @brief Structure representing a buffer for storing received SPP data.
 *
 * The structure includes information about the number of nodes, total buffer size,
 * and a pointer to the first node in the buffer. It is used to manage and process
 * data received over the Serial Port Profile (SPP).
 */
typedef struct spp_receive_data_buff{
    int32_t node_num;
    int32_t buff_size;
    spp_receive_data_node_t * first_node;
}spp_receive_data_buff_t;

/**
 * @brief Structure representing an instance of the GATT server profile.
 *
 * The structure contains information necessary for managing a specific GATT server profile instance,
 * including callback function, interface, application ID, connection ID, service handle,
 * service ID, characteristic handle, characteristic UUID, permissions, properties,
 * descriptor handle, and descriptor UUID.
 */
struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};


// Enumeration defining indices for attributes in the Serial Port Profile (SPP) GATT server.
enum{
    SPP_IDX_SVC,

    SPP_IDX_SPP_DATA_RECV_CHAR,
    SPP_IDX_SPP_DATA_RECV_VAL,

    SPP_IDX_SPP_DATA_NOTIFY_CHAR,
    SPP_IDX_SPP_DATA_NTY_VAL,
    SPP_IDX_SPP_DATA_NTF_CFG,

    SPP_IDX_SPP_COMMAND_CHAR,
    SPP_IDX_SPP_COMMAND_VAL,

    SPP_IDX_SPP_STATUS_CHAR,
    SPP_IDX_SPP_STATUS_VAL,
//ZAD_2 add new indexes

//TODO_2

///////////////////////////////////  
    SPP_IDX_SPP_STATUS_CFG,
    SPP_IDX_NB,
};

#endif /* BLE_SPP_SERVER */