///////////////////////////////////////////////////////////////////////
// File: gatt.c
//
// Subject: Standardy i Systemy Komunikacyjne, AGH, EiT
//
// Author: Paweł Majewski
//
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////
// HEDER FILE
///////////////////////////////////////////////////////////////////////////////////

#include "gatt.h"

///////////////////////////////////////////////////////////////////////////////////
// STATIC DATA
///////////////////////////////////////////////////////////////////////////////////

// Pointers to structures for temporary storage of received SPP data nodes.
static spp_receive_data_node_t * temp_spp_recv_data_node_p1 = NULL;
static spp_receive_data_node_t * temp_spp_recv_data_node_p2 = NULL;

static QueueHandle_t cmd_cmd_queue = NULL;
static uint16_t spp_mtu_size = 23;
static uint16_t spp_handle_table[SPP_IDX_NB];
static esp_bd_addr_t spp_remote_bda = {0x0,};
static esp_gatt_if_t spp_gatts_if = 0xff;
static uint16_t spp_conn_id = 0xffff;
static bool enable_data_ntf = false;
static bool is_connected = false;

// Constants defining various GATT UUIDs for the Serial Port Profile (SPP) service.
static const uint16_t spp_service_uuid = 0xABF0;
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

// Constants defining GATT characteristic properties.
static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_READ;

///SPP Service - data receive characteristic, read&write without response
static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t  spp_data_receive_val[20] = {0x00};

///SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t  spp_data_notify_val[20] = {0x00};
static const uint8_t  spp_data_notify_ccc[2] = {0x00, 0x00};

///SPP Service - command characteristic, read&write without response
static const uint16_t spp_command_uuid = ESP_GATT_UUID_SPP_COMMAND_RECEIVE;
static const uint8_t  spp_command_val[10] = {0x00};

///SPP Service - status characteristic, notify&read
static const uint16_t spp_status_uuid = ESP_GATT_UUID_SPP_COMMAND_NOTIFY;
static const uint8_t  spp_status_val[10] = {0x00};
static const uint8_t  spp_status_ccc[2] = {0x00, 0x00};

//ZAD_2 add new sevice value

//TODO_2

///////////////////////////////////  

// Structure defining the advertising parameters for the Serial Port Profile (SPP) service.
static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// Array defining the advertising data for the Serial Port Profile (SPP) service.
static const uint8_t spp_adv_data[23] = {
    /* Flags */
    0x02,0x01,0x06,
    /* Complete List of 16-bit Service Class UUIDs */
    0x03,0x03,0xF0,0xAB,
    /* Complete Local Name in advertising */
    0x0F,0x09, 'E', 'S', 'P', '_', 'S', 'P', 'P', '_', 'S', 'E', 'R','V', 'E', 'R'
};

// Structure representing the buffer for storing received SPP data.
static spp_receive_data_buff_t SppRecvDataBuff = {
    .node_num   = 0,
    .buff_size  = 0,
    .first_node = NULL
};

// Array defining the GATT database for the Serial Port Profile (SPP).
// Each element corresponds to a specific attribute in the GATT profile,
// including service declaration, characteristic declaration, and their values.
// The array follows the SPP_IDX_NB enumeration order, defining attributes
// such as service UUID, data receive characteristic, data notify characteristic, command characteristic,
// status characteristic, and associated descriptors.
// The array is used to initialize the GATT server database during the SPP service registration.
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] =
{
    //SPP -  Service Declaration
    [SPP_IDX_SVC]                      	=
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
    sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

    //SPP -  data receive characteristic Declaration
    [SPP_IDX_SPP_DATA_RECV_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  data receive characteristic Value
    [SPP_IDX_SPP_DATA_RECV_VAL]             	=
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_DATA_MAX_LEN,sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val}},

    //SPP -  data notify characteristic Declaration
    [SPP_IDX_SPP_DATA_NOTIFY_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  data notify characteristic Value
    [SPP_IDX_SPP_DATA_NTY_VAL]   =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_notify_uuid, ESP_GATT_PERM_READ,
    SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *)spp_data_notify_val}},

    //SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_DATA_NTF_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},

    //SPP -  command characteristic Declaration
    [SPP_IDX_SPP_COMMAND_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  command characteristic Value
    [SPP_IDX_SPP_COMMAND_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_command_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_CMD_MAX_LEN,sizeof(spp_command_val), (uint8_t *)spp_command_val}},

    //SPP -  status characteristic Declaration
    [SPP_IDX_SPP_STATUS_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  status characteristic Value
    [SPP_IDX_SPP_STATUS_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_status_uuid, ESP_GATT_PERM_READ,
    SPP_STATUS_MAX_LEN,sizeof(spp_status_val), (uint8_t *)spp_status_val}},

//ZAD_2 add new sevice

//TODO_2

///////////////////////////////////  

    //SPP -  status characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_STATUS_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_status_ccc), (uint8_t *)spp_status_ccc}},
};

///////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES
///////////////////////////////////////////////////////////////////////////////////

// Global queue handle for UART communication in SPP.
QueueHandle_t spp_uart_queue = NULL;

///////////////////////////////////////////////////////////////////////////////////
// LOCAL FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Finds the index of the characteristic or descriptor based on the handle.
 *
 * This function iterates through the SPP handle table and returns the index
 * corresponding to the given handle. If the handle is not found, it returns 0xFF.
 *
 * @param[in] handle  Handle to search for in the SPP handle table.
 * @return Index of the handle in the table, or 0xFF if not found.
 */
static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;

    for(int i = 0; i < SPP_IDX_NB ; i++){
        if(handle == spp_handle_table[i]){
            return i;
        }
    }

    return error;
}

/**
 * @brief Stores received SPP data in the write buffer.
 *
 * This function allocates memory for a new node in the write buffer,
 * copies the received data to the node, and updates buffer-related parameters.
 *
 * @param[in] p_data  Pointer to the GATT server callback parameters.
 * @return True if successful, false if memory allocation fails.
 */
static bool store_wr_buffer(esp_ble_gatts_cb_param_t *p_data)
{
    temp_spp_recv_data_node_p1 = (spp_receive_data_node_t *)malloc(sizeof(spp_receive_data_node_t));

    if(temp_spp_recv_data_node_p1 == NULL){
        return false;
    }
    if(temp_spp_recv_data_node_p2 != NULL){
        temp_spp_recv_data_node_p2->next_node = temp_spp_recv_data_node_p1;
    }
    temp_spp_recv_data_node_p1->len = p_data->write.len;
    SppRecvDataBuff.buff_size += p_data->write.len;
    temp_spp_recv_data_node_p1->next_node = NULL;
    temp_spp_recv_data_node_p1->node_buff = (uint8_t *)malloc(p_data->write.len);
    temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1;
    memcpy(temp_spp_recv_data_node_p1->node_buff,p_data->write.value,p_data->write.len);
    if(SppRecvDataBuff.node_num == 0){
        SppRecvDataBuff.first_node = temp_spp_recv_data_node_p1;
        SppRecvDataBuff.node_num++;
    }else{
        SppRecvDataBuff.node_num++;
    }

    return true;
}

/**
 * @brief Prints the contents of the write buffer to the UART.
 *
 * This function iterates through the write buffer nodes and writes
 * their contents to the UART for debugging or logging purposes.
 */
static void print_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while(temp_spp_recv_data_node_p1 != NULL){
        uart_write_bytes(UART_NUM_0, (char *)(temp_spp_recv_data_node_p1->node_buff), temp_spp_recv_data_node_p1->len);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p1->next_node;
    }
}

/**
 * @brief Frees the write buffer used for storing received SPP data.
 *
 * This function frees the allocated memory for the write buffer nodes
 * and resets the buffer-related parameters.
 */
static void free_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while(temp_spp_recv_data_node_p1 != NULL){
        temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1->next_node;
        free(temp_spp_recv_data_node_p1->node_buff);
        free(temp_spp_recv_data_node_p1);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p2;
    }

    SppRecvDataBuff.node_num = 0;
    SppRecvDataBuff.buff_size = 0;
    SppRecvDataBuff.first_node = NULL;
}

/**
 * @brief Task for handling UART communication.
 *
 * This task continuously waits for UART events, such as data reception,
 * and sends the received data as notifications over BLE if connected.
 *
 * @param[in] pvParameters  Task parameters (not used in this task).
 */
static void uart_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t total_num = 0;
    uint8_t current_num = 0;

    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(spp_uart_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            switch (event.type) {
            //Event of UART receving data
            case UART_DATA:
                if ((event.size)&&(is_connected)) {
                    uint8_t * temp = NULL;
                    uint8_t * ntf_value_p = NULL;
                    if(!enable_data_ntf){
                        break;
                    }
                    temp = (uint8_t *)malloc(sizeof(uint8_t)*event.size);
                    if(temp == NULL){
                        break;
                    }
                    memset(temp,0x0,event.size);
                    uart_read_bytes(UART_NUM_0,temp,event.size,portMAX_DELAY);
                    if(event.size <= (spp_mtu_size - 3)){
                        esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],event.size, temp, false);
                    }else if(event.size > (spp_mtu_size - 3)){
                        if((event.size%(spp_mtu_size - 7)) == 0){
                            total_num = event.size/(spp_mtu_size - 7);
                        }else{
                            total_num = event.size/(spp_mtu_size - 7) + 1;
                        }
                        current_num = 1;
                        ntf_value_p = (uint8_t *)malloc((spp_mtu_size-3)*sizeof(uint8_t));
                        if(ntf_value_p == NULL){
                            free(temp);
                            break;
                        }
                        while(current_num <= total_num){
                            if(current_num < total_num){
                                ntf_value_p[0] = '#';
                                ntf_value_p[1] = '#';
                                ntf_value_p[2] = total_num;
                                ntf_value_p[3] = current_num;
                                memcpy(ntf_value_p + 4,temp + (current_num - 1)*(spp_mtu_size-7),(spp_mtu_size-7));
                                esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],(spp_mtu_size-3), ntf_value_p, false);
                            }else if(current_num == total_num){
                                ntf_value_p[0] = '#';
                                ntf_value_p[1] = '#';
                                ntf_value_p[2] = total_num;
                                ntf_value_p[3] = current_num;
                                memcpy(ntf_value_p + 4,temp + (current_num - 1)*(spp_mtu_size-7),(event.size - (current_num - 1)*(spp_mtu_size - 7)));
                                esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],(event.size - (current_num - 1)*(spp_mtu_size - 7) + 4), ntf_value_p, false);
                            }
                            vTaskDelay(20 / portTICK_PERIOD_MS);
                            current_num++;
                        }
                        free(ntf_value_p);
                    }
                    free(temp);
                }
                break;
            default:
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

/**
 * @brief Initializes the UART for Serial Port Profile (SPP) communication.
 *
 * This function configures the UART parameters, installs the UART driver,
 * sets the UART pins, and creates a task for UART communication.
 */
static void spp_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, 4096, 8192, 10,&spp_uart_queue,0);
    //Set UART parameters
    uart_param_config(UART_NUM_0, &uart_config);
    //Set UART pins
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(uart_task, "uTask", 2048, (void*)UART_NUM_0, 8, NULL);
}

/**
 * @brief Task for handling SPP commands.
 *
 * This task continuously waits for SPP commands in the command queue,
 * logs the received command using ESP_LOG, and frees the allocated memory.
 *
 * @param[in] arg  Task argument (not used in this task).
 */
static void spp_cmd_task(void * arg)
{
    uint8_t * cmd_id;

    for(;;){
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if(xQueueReceive(cmd_cmd_queue, &cmd_id, portMAX_DELAY)) {
            esp_log_buffer_char(GATTS_TABLE_TAG,(char *)(cmd_id),strlen((char *)cmd_id));
            free(cmd_id);
        }
    }
    vTaskDelete(NULL);
}

///////////////////////////////////////////////////////////////////////////////////
// GLOBAL FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Handles GATT server events for the Generic Attribute Profile (GATT) server.
 *
 * This function processes GATT server events, such as registration events,
 * and dispatches the events to the appropriate callback functions.
 *
 * @param[in] event     GATT server callback event.
 * @param[in] gatts_if  GATT interface.
 * @param[in] param     Pointer to the GATT server callback parameters.
 */
//ZAD_1

//TODO_1

///////////////////////////////////
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *) param;
    uint8_t res = 0xff;

    switch (event) {
    	case //TODO_1:
            if (event == ESP_GATTS_REG_EVT) {
                if (param->reg.status != ESP_GATT_OK) {
                    return;
                }
            }
        	esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
        	esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, sizeof(spp_adv_data));
        	esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
       	break;
    	case //TODO_1:
            find_char_and_desr_index(p_data->read.handle);
       	 break;
    	case //TODO_1: {
    	    res = find_char_and_desr_index(p_data->write.handle);
            if(p_data->write.is_prep == false){
                if(res == SPP_IDX_SPP_COMMAND_VAL){
                    uint8_t * spp_cmd_buff = NULL;
                    spp_cmd_buff = (uint8_t *)malloc((spp_mtu_size - 3) * sizeof(uint8_t));
                    if(spp_cmd_buff == NULL){
                        break;
                    }
                    memset(spp_cmd_buff,0x0,(spp_mtu_size - 3));
                    memcpy(spp_cmd_buff,p_data->write.value,p_data->write.len);
                    xQueueSend(cmd_cmd_queue,&spp_cmd_buff,10/portTICK_PERIOD_MS);
                }else if(res == SPP_IDX_SPP_DATA_NTF_CFG){
                    if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)){
                        enable_data_ntf = true;
                    }else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)){
                        enable_data_ntf = false;
                    }
                }
                else if(res == SPP_IDX_SPP_DATA_RECV_VAL){
                    uart_write_bytes(UART_NUM_0, (char *)(p_data->write.value), p_data->write.len);
                }
            }else if((p_data->write.is_prep == true)&&(res == SPP_IDX_SPP_DATA_RECV_VAL)){
                store_wr_buffer(p_data);
            }
      	 	break;
    	}
    	case //TODO_1:{
    	    if(p_data->exec_write.exec_write_flag){
    	        print_write_buffer();
    	        free_write_buffer();
    	    }
    	    break;
    	}
    	case //TODO_1:
    	    spp_mtu_size = p_data->mtu.mtu;
    	    break;
    	case //TODO_1:
    	    spp_conn_id = p_data->connect.conn_id;
    	    spp_gatts_if = gatts_if;
    	    is_connected = true;
    	    memcpy(&spp_remote_bda,&p_data->connect.remote_bda,sizeof(esp_bd_addr_t));
        	break;
    	case //TODO_1:
    	    is_connected = false;
    	    enable_data_ntf = false;
    	    esp_ble_gap_start_advertising(&spp_adv_params);
    	    break;
    	case //TODO_1:{
    	    if (param->add_attr_tab.num_handle == SPP_IDX_NB && param->add_attr_tab.status == ESP_GATT_OK){
    	        memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
    	        esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
    	    }
    	    break;
    	}
    	default:
    	    break;
    }
}

/**
 * @brief Initializes the SPP (Serial Port Profile) task.
 *
 * This function initializes the necessary elements for handling SPP,
 * including the UART interface initialization and the creation of a command queue.
 * Additionally, it creates a task to handle SPP commands.
 */
void spp_task_init(void)
{
    spp_uart_init();
    cmd_cmd_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(spp_cmd_task, "spp_cmd_task", 2048, NULL, 10, NULL);
}