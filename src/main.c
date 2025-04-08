/* main.c - Application main entry point */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_random.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"

#include "driver/uart.h"

#define UART_NUM UART_NUM_2
#define RX_PIN GPIO_NUM_17       // RX2
#define UART_BUF_SIZE 33

#include "board.h"
#include "ble_mesh_example_init.h"

uint8_t * data;
uint8_t readyToSend = 0;
SemaphoreHandle_t xSerialData = NULL;

// Buffer sizes
#define BUF_SIZE (1024)

#define TAG "EXAMPLE"

#define CID_ESP 0x02E5

#define ESP_BLE_MESH_VND_MODEL_ID_CLIENT 0x0000
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER 0x0001

#define ESP_BLE_MESH_VND_MODEL_OP_SEND ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_STATUS ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)
#define OP_CODE_SIZE 0x03
#define ESP_BLE_MESH_VND_MODEL_OP_SUBS_READ ESP_BLE_MESH_MODEL_OP_3(0x03, CID_ESP)

static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = {0x32, 0x10};

static esp_ble_mesh_cfg_srv_t config_server = {
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
};

static esp_ble_mesh_model_op_t vnd_op[] = {
    // ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_SEND, 0),
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_SUBS_READ, 3),
    ESP_BLE_MESH_MODEL_OP_END,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(
    sensor_pub,
    UART_BUF_SIZE + OP_CODE_SIZE + 4,
    ROLE_NODE
);

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_SERVER,
                              vnd_op, &sensor_pub, NULL),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .element_count = ARRAY_SIZE(elements),
    .elements = elements,
};

static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
};

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "net_idx 0x%03x, addr 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags 0x%02x, iv_index 0x%08" PRIx32, flags, iv_index);
    board_led_operation(LED_G, LED_OFF);
}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
                 param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
                 param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
                      param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}

static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT)
    {
        switch (param->ctx.recv_op)
        {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                     param->value.state_change.appkey_add.net_idx,
                     param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                     param->value.state_change.mod_app_bind.element_addr,
                     param->value.state_change.mod_app_bind.app_idx,
                     param->value.state_change.mod_app_bind.company_id,
                     param->value.state_change.mod_app_bind.model_id);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET");
            ESP_LOGI(TAG, "elem_addr 0x%04x, pub_addr 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_pub_set.element_addr,
                param->value.state_change.mod_pub_set.pub_addr,
                param->value.state_change.mod_pub_set.company_id,
                param->value.state_change.mod_pub_set.model_id);

            break;
        default:
            break;
        }
    }
}

// uint8_t trans_data[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd };
uint8_t trans_data[] = {
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 
    0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 
    0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 
    0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 
    0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 
    0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 
    0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 
    0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 
    0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 
    0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 
    0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 
    0xdd, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9
};

static void example_ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                                             esp_ble_mesh_model_cb_param_t *param)
{

    switch (event)
    {

    case ESP_BLE_MESH_MODEL_OPERATION_EVT:

        if ( param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_SEND )
        {
            uint16_t tid = *(uint16_t *)param->model_operation.msg;
            ESP_LOGI(TAG, "Recv 0x%06" PRIx32 ", tid 0x%04x", param->model_operation.opcode, tid);
            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                                                               param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                                                               sizeof(tid), (uint8_t *)&tid);
            if (err)
            {
                ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            }

        } else if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_SUBS_READ) {

            ESP_LOG_BUFFER_HEX(
                "ESP_BLE_MESH_VND_MODEL_OP_SUBS_READ",
                param->model_operation.msg,
                param->model_operation.length 
            );
            
        }
        break;
    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:

        if (param->model_send_comp.err_code) {
            ESP_LOGE(TAG, "Failed to send message 0x%06" PRIx32, param->model_send_comp.opcode);
            break;
        }
        ESP_LOGI(TAG, "Send 0x%06" PRIx32, param->model_send_comp.opcode);
        break;

    case ESP_BLE_MESH_MODEL_PUBLISH_UPDATE_EVT:

        readyToSend = 1;
        if ( readyToSend ) {
            uint8_t pub_data[UART_BUF_SIZE + OP_CODE_SIZE];

            pub_data[0] = (ESP_BLE_MESH_VND_MODEL_OP_SUBS_READ >> 16) & 0xFF;
            pub_data[2] = (ESP_BLE_MESH_VND_MODEL_OP_SUBS_READ >> 8) & 0xFF;
            pub_data[1] = ESP_BLE_MESH_VND_MODEL_OP_SUBS_READ & 0xFF;

            for ( int i = OP_CODE_SIZE; i < UART_BUF_SIZE + OP_CODE_SIZE; i++  ) {
                pub_data[i] = trans_data[i - OP_CODE_SIZE];
            }

            sensor_pub.msg->len = UART_BUF_SIZE + OP_CODE_SIZE;
            memcpy( sensor_pub.msg->data, pub_data, UART_BUF_SIZE + OP_CODE_SIZE );

            ESP_LOG_BUFFER_HEX( "ENVIOU", sensor_pub.msg->data, OP_CODE_SIZE + UART_BUF_SIZE );
            ESP_LOG_BUFFER_HEX( "OPCODE", sensor_pub.model->op, OP_CODE_SIZE );
            ESP_LOG_BUFFER_HEX( "BUF", sensor_pub.msg->__buf, OP_CODE_SIZE + UART_BUF_SIZE );

            // esp_err_t err = esp_ble_mesh_model_publish (
            //     param->model_publish_update.model,
            //     ESP_BLE_MESH_VND_MODEL_OP_SUBS_READ,
            //     UART_BUF_SIZE + OP_CODE_SIZE,
            //     pub_data,
            //     ROLE_NODE
            // );

            // if ( !err ) { readyToSend = 0; }
        }

        break;

    default:
        break;
    }
}

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err;

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
    esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize mesh stack");
        return err;
    }

    err = esp_ble_mesh_node_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable mesh node");
        return err;
    }

    // board_led_operation(LED_G, LED_ON);

    ESP_LOGI(TAG, "BLE Mesh Node initialized");

    return ESP_OK;
}

void ble_task ( void * param ) {
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    board_init();

    err = bluetooth_init();
    if (err)
    {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err)
    {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }

    err = esp_ble_mesh_set_unprovisioned_device_name( "O_q_envia" );
    if (err)
    {
        ESP_LOGE(TAG, "Node name failed (err %d)", err);
    }

    for (;;) {
        vTaskDelay( pdMS_TO_TICKS(100) );
    }
}

void cst_uart_task ( void * param ) {

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    esp_err_t err;

    err = uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if ( err != ESP_OK ) ESP_LOGE("UART", "Error seting up the pins");
    err = uart_driver_install(UART_NUM, 1024 * 2, 0, 0, NULL, 0);
    if ( err != ESP_OK ) ESP_LOGE("UART", "Error installing drivers");
    err = uart_param_config(UART_NUM, &uart_config);
    if ( err != ESP_OK ) ESP_LOGE("UART", "Error configuring UART");


    // if ( xSemaphoreTake( xSerialData, portMAX_DELAY ) == pdTRUE ) {
    //     data = (uint8_t*) malloc(UART_BUF_SIZE + 1);
    //     xSemaphoreGive( xSerialData );
    // }
    data = (uint8_t*) malloc(UART_BUF_SIZE + 1);

    while (1) {
        // if ( xSemaphoreTake( xSerialData, portMAX_DELAY ) == pdTRUE ) {
            
            int len = uart_read_bytes(UART_NUM, data, UART_BUF_SIZE, pdMS_TO_TICKS(50) );
            memcpy( trans_data, data, (size_t) UART_BUF_SIZE );

            // printf("Opa\n");
            // if ( len > 0 ) printf("Leu\n");
            // else printf("n leu\n");

            if (len >= UART_BUF_SIZE) {
                readyToSend = 1;
                printf("Received %d bytes: ", len);
                ESP_LOG_BUFFER_HEX("UART", data, len);
                uart_flush_input(UART_NUM);
            }

        //     xSemaphoreGive( xSerialData );
        // }

        vTaskDelay( pdMS_TO_TICKS( 100 ) );
    }
    free(data);
}

void app_main(void)
{
    xSerialData = xSemaphoreCreateBinary();
    
    xTaskCreatePinnedToCore( ble_task, "BLE task", 1024 * 6, NULL, 1, NULL, 1 );
    xTaskCreatePinnedToCore( cst_uart_task, "UART task", 1024 * 2, NULL, 2, NULL, 0 );
}
