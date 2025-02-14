/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/****************************************************************************
*
* This demo showcases BLE GATT server. It can send adv data, be connected by client.
* Run the gatt_client demo, the client demo will automatically connect to the gatt_server demo.
* Client demo will enable gatt_server's notify after connection. The two devices will then exchange
* data.
*
****************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"

#include "sdkconfig.h"

#include "memfault/esp_port/core.h"
#include "memfault/core/platform/device_info.h"
#include <memfault/components.h>

#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "driver/uart.h"
#include "esp_vfs_usb_serial_jtag.h"

#define GATTS_TAG "GATTS_DEMO"
#define TEST_SERVICE_UUID        0x00FF
#define TEST_CHAR_UUID          0xFF01
#define TEST_DEVICE_NAME        "TYLER_DEVICE"
#define TEST_MANUFACTURER_DATA_LEN  17
#define PROFILE_NUM 1
#define PROFILE_APP_ID 0

// Add MDS specific defines
#define MDS_NUM_CHARACTERISTICS 5  // Supported Features, Device ID, Data URI, Auth, Data Export
#define MDS_NUM_DESCRIPTORS    1   // CCCD for Data Export
#define MDS_NUM_HANDLES        (1 + (2 * MDS_NUM_CHARACTERISTICS) + MDS_NUM_DESCRIPTORS)  // 12 total

static uint16_t profile_handle = 0;

// Profile structure definition
struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
};

// Profile table
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_ID] = {
        .gatts_if = ESP_GATT_IF_NONE,
    },
};

// Update the service UUID array to use MDS UUID in little-endian format
static uint8_t adv_service_uuid128[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // MDS Service UUID: 54220000-f6a5-4007-a371-722f4ebd8436 (in little-endian)
    0x36, 0x84, 0xbd, 0x4e, 0x2f, 0x72, 0x71, 0xa3, 0x07, 0x40, 0xa5, 0xf6, 0x00, 0x00, 0x22, 0x54,
};

// MDS characteristic values
static uint8_t mds_supported_features[] = {0x00};  // No features supported
static const char *mds_device_id = "ESP32-TEST-DEVICE";
static char mds_data_uri[256];  // Make it big enough for base URI + serial
static const char *mds_auth = "Memfault-Project-Key:kWRfBKMZ9T5Mtw3yrzZBRIIGoTRBpr8V";

// MDS state
static struct {
    bool subscribed;
    uint16_t conn_id;
    uint8_t export_mode;
    uint8_t chunk_sequence;
    uint16_t mtu;  // Add MTU tracking
    bool congested;  // Add congestion tracking
} mds_state = {
    .subscribed = false,
    .conn_id = 0xFFFF,
    .export_mode = 0,
    .chunk_sequence = 0,
    .mtu = 23,  // Default minimum MTU size
    .congested = false,  // Initialize as not congested
};

// Add handle tracking structure after other static variables
static struct {
    uint16_t supported_features_handle;
    uint16_t device_id_handle;
    uint16_t data_uri_handle;
    uint16_t auth_handle;
    uint16_t data_export_handle;
    uint16_t data_export_cccd_handle;
} mds_handles = {0};

// Add MDS error codes
#define MDS_ERR_INVALID_LENGTH           0x80
#define MDS_ERR_CLIENT_ALREADY_SUBSCRIBED 0x81
#define MDS_ERR_CLIENT_NOT_SUBSCRIBED    0x82

// Add this structure for the chunk payload format
typedef struct __attribute__((packed)) {
    uint8_t chunk_number:5;  // 5 bits for sequence number (0-31)
    uint8_t rfu:3;          // Reserved for future use
    uint8_t data[];         // Variable length chunk data
} mds_data_export_payload_t;

// Update send_chunk_notification to use Memfault's chunk API
static void send_chunk_notification(esp_gatt_if_t gatts_if) {
    if (!mds_state.subscribed || mds_state.export_mode != 0x01 || mds_state.congested) {
        return;
    }

    // Use the tracked MTU size
    uint16_t mtu_size = mds_state.mtu;
    
    // Account for ATT header (3 bytes) and chunk header (1 byte)
    size_t max_chunk_size = mtu_size - 4;

    // Allocate buffer for the notification
    size_t total_len = sizeof(mds_data_export_payload_t) + max_chunk_size;
    uint8_t *notify_data = malloc(total_len);
    if (!notify_data) {
        ESP_LOGE(GATTS_TAG, "Failed to allocate notification buffer");
        return;
    }

    // Prepare the notification data
    mds_data_export_payload_t *payload = (mds_data_export_payload_t *)notify_data;
    payload->chunk_number = mds_state.chunk_sequence;
    payload->rfu = 0;

    // Get chunk data from Memfault
    size_t chunk_size = max_chunk_size;
    bool data_available = memfault_packetizer_get_chunk(payload->data, &chunk_size);
    
    if (!data_available) {
        ESP_LOGI(GATTS_TAG, "No more chunks available");
        free(notify_data);
        mds_state.export_mode = 0x00;  // Disable export mode when no more data
        return;
    }

    // Adjust total length based on actual chunk size
    total_len = sizeof(mds_data_export_payload_t) + chunk_size;

    // Send the notification
    esp_err_t err = esp_ble_gatts_send_indicate(gatts_if,
                                               mds_state.conn_id,
                                               mds_handles.data_export_handle,
                                               total_len,
                                               notify_data,
                                               false);  // false for notification, true for indication
    if (err != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "Failed to send notification: %d", err);
        memfault_packetizer_abort();
    } else {
        // Update sequence number (0-31)
        mds_state.chunk_sequence = (mds_state.chunk_sequence + 1) & 0x1F;
        ESP_LOGI(GATTS_TAG, "Sent chunk %d, size %d", mds_state.chunk_sequence, chunk_size);
    }

    free(notify_data);
}

// Update advertising data configuration
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// Add scan response data configuration
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// Advertising parameters
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising start failed");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed");
        } else {
            ESP_LOGI(GATTS_TAG, "Stop adv successfully");
        }
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT: {
        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
        
        // Initialize MDS service
        gl_profile_tab[PROFILE_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_128;
        memcpy(gl_profile_tab[PROFILE_APP_ID].service_id.id.uuid.uuid.uuid128, 
               adv_service_uuid128, ESP_UUID_LEN_128);

        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_APP_ID].service_id, 
                                   MDS_NUM_HANDLES);
        break;
    }

    case ESP_GATTS_CREATE_EVT: {
        ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d, service_handle %d",
                 param->create.status, param->create.service_handle);
        
        profile_handle = param->create.service_handle;
        
        // Start adding characteristics with 128-bit UUIDs
        esp_bt_uuid_t uuid = {
            .len = ESP_UUID_LEN_128,
            .uuid.uuid128 = {
                // 54220001-f6a5-4007-a371-722f4ebd8436 in little-endian
                0x36, 0x84, 0xbd, 0x4e, 0x2f, 0x72, 0x71, 0xa3, 0x07, 0x40, 0xa5, 0xf6, 0x01, 0x00, 0x22, 0x54
            }
        };
        
        esp_ble_gatts_add_char(profile_handle, &uuid,
                              ESP_GATT_PERM_READ,
                              ESP_GATT_CHAR_PROP_BIT_READ,
                              NULL, NULL);
        break;
    }

    case ESP_GATTS_ADD_CHAR_EVT: {
        ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d, attr_handle %d, service_handle %d",
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

        // Instead, check the last byte of the 128-bit UUID:
        if (param->add_char.char_uuid.uuid.uuid128[12] == 0x01) {
            mds_handles.supported_features_handle = param->add_char.attr_handle;
        } else if (param->add_char.char_uuid.uuid.uuid128[12] == 0x02) {
            mds_handles.device_id_handle = param->add_char.attr_handle;
        } else if (param->add_char.char_uuid.uuid.uuid128[12] == 0x03) {
            mds_handles.data_uri_handle = param->add_char.attr_handle;
        } else if (param->add_char.char_uuid.uuid.uuid128[12] == 0x04) {
            mds_handles.auth_handle = param->add_char.attr_handle;
        } else if (param->add_char.char_uuid.uuid.uuid128[12] == 0x05) {
            mds_handles.data_export_handle = param->add_char.attr_handle;
        }

        // Add next characteristic based on which one was just added
        if (param->add_char.char_uuid.uuid.uuid128[12] == 0x01) {
            // Add Device ID characteristic
            esp_bt_uuid_t uuid = {
                .len = ESP_UUID_LEN_128,
                .uuid.uuid128 = {
                    // 54220002-f6a5-4007-a371-722f4ebd8436 in little-endian
                    0x36, 0x84, 0xbd, 0x4e, 0x2f, 0x72, 0x71, 0xa3, 0x07, 0x40, 0xa5, 0xf6, 0x02, 0x00, 0x22, 0x54
                }
            };
            esp_ble_gatts_add_char(profile_handle, &uuid,
                                  ESP_GATT_PERM_READ,
                                  ESP_GATT_CHAR_PROP_BIT_READ,
                                  NULL, NULL);
        }
        else if (param->add_char.char_uuid.uuid.uuid128[12] == 0x02) {  // After Device ID
            // Add Data URI characteristic
            esp_bt_uuid_t uuid = {
                .len = ESP_UUID_LEN_128,
                .uuid.uuid128 = {
                    // 54220003-f6a5-4007-a371-722f4ebd8436 in little-endian
                    0x36, 0x84, 0xbd, 0x4e, 0x2f, 0x72, 0x71, 0xa3, 0x07, 0x40, 0xa5, 0xf6, 0x03, 0x00, 0x22, 0x54
                }
            };
            esp_ble_gatts_add_char(profile_handle, &uuid,
                                  ESP_GATT_PERM_READ,
                                  ESP_GATT_CHAR_PROP_BIT_READ,
                                  NULL, NULL);
        }
        else if (param->add_char.char_uuid.uuid.uuid128[12] == 0x03) {  // After Data URI
            // Add Auth characteristic
            esp_bt_uuid_t uuid = {
                .len = ESP_UUID_LEN_128,
                .uuid.uuid128 = {
                    // 54220004-f6a5-4007-a371-722f4ebd8436 in little-endian
                    0x36, 0x84, 0xbd, 0x4e, 0x2f, 0x72, 0x71, 0xa3, 0x07, 0x40, 0xa5, 0xf6, 0x04, 0x00, 0x22, 0x54
                }
            };
            esp_ble_gatts_add_char(profile_handle, &uuid,
                                  ESP_GATT_PERM_READ,
                                  ESP_GATT_CHAR_PROP_BIT_READ,
                                  NULL, NULL);
        }
        else if (param->add_char.char_uuid.uuid.uuid128[12] == 0x04) {  // After Auth
            // Add Data Export characteristic
            esp_bt_uuid_t uuid = {
                .len = ESP_UUID_LEN_128,
                .uuid.uuid128 = {
                    // 54220005-f6a5-4007-a371-722f4ebd8436 in little-endian
                    0x36, 0x84, 0xbd, 0x4e, 0x2f, 0x72, 0x71, 0xa3, 0x07, 0x40, 0xa5, 0xf6, 0x05, 0x00, 0x22, 0x54
                }
            };
            esp_ble_gatts_add_char(profile_handle, &uuid,
                                  ESP_GATT_PERM_WRITE,
                                  ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                  NULL, NULL);
        }
        else if (param->add_char.char_uuid.uuid.uuid128[12] == 0x05) {  // After Data Export
            // Add CCCD descriptor
            esp_bt_uuid_t uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
            };
            esp_ble_gatts_add_char_descr(profile_handle, &uuid,
                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                        NULL, NULL);
        }
        break;
    }

    case ESP_GATTS_ADD_CHAR_DESCR_EVT: {
        ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d",
                 param->add_char_descr.status, param->add_char_descr.attr_handle,
                 param->add_char_descr.service_handle);
        
        if (param->add_char_descr.descr_uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG) {
            mds_handles.data_export_cccd_handle = param->add_char_descr.attr_handle;
        }
        
        // Start the service after all characteristics are added
        esp_ble_gatts_start_service(profile_handle);
        break;
    }

    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, handle %d", param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        
        // Handle reads based on characteristic
        if (param->read.handle == mds_handles.supported_features_handle) {
            rsp.attr_value.len = sizeof(mds_supported_features);
            memcpy(rsp.attr_value.value, mds_supported_features, sizeof(mds_supported_features));
        }
        else if (param->read.handle == mds_handles.device_id_handle) {
            rsp.attr_value.len = strlen(mds_device_id);
            memcpy(rsp.attr_value.value, mds_device_id, rsp.attr_value.len);
        }
        else if (param->read.handle == mds_handles.data_uri_handle) {
            rsp.attr_value.len = strlen(mds_data_uri);
            memcpy(rsp.attr_value.value, mds_data_uri, rsp.attr_value.len);
        }
        else if (param->read.handle == mds_handles.auth_handle) {
            rsp.attr_value.len = strlen(mds_auth);
            memcpy(rsp.attr_value.value, mds_auth, rsp.attr_value.len);
        }
        else {
            // Unknown handle
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                      ESP_GATT_INVALID_HANDLE, NULL);
            break;
        }
        
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                   ESP_GATT_OK, &rsp);
        break;
    }

    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, handle %d", param->write.handle);

        if (param->write.handle == mds_handles.data_export_handle) {
            // Handle Data Export write
            if (param->write.len != 1) {
                // Invalid length
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                          MDS_ERR_INVALID_LENGTH, NULL);
                break;
            }

            if (!mds_state.subscribed) {
                // Client must subscribe first
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                          MDS_ERR_CLIENT_NOT_SUBSCRIBED, NULL);
                break;
            }

            uint8_t mode = param->write.value[0];
            if (mode != 0x00 && mode != 0x01) {
                // Invalid mode
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                          ESP_GATT_INVALID_PDU, NULL);
                break;
            }

            mds_state.export_mode = mode;
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                      ESP_GATT_OK, NULL);

            // If mode is enabled, start sending notifications
            if (mode == 0x01) {
                send_chunk_notification(gatts_if);
            }
        }
        else if (param->write.handle == mds_handles.data_export_cccd_handle) {
            // Handle CCCD write
            if (param->write.len != 2) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                          MDS_ERR_INVALID_LENGTH, NULL);
                break;
            }

            uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
            bool notifications_enabled = (descr_value & 0x0001) != 0;

            if (notifications_enabled && mds_state.subscribed && 
                mds_state.conn_id != param->write.conn_id) {
                // Only one client can be subscribed at a time
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                          MDS_ERR_CLIENT_ALREADY_SUBSCRIBED, NULL);
                break;
            }

            mds_state.subscribed = notifications_enabled;
            mds_state.conn_id = notifications_enabled ? param->write.conn_id : 0xFFFF;
            if (!notifications_enabled) {
                mds_state.export_mode = 0x00;  // Disable export mode when unsubscribing
            }

            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                      ESP_GATT_OK, NULL);
        }
        break;
    }

    case ESP_GATTS_CONF_EVT: {
        // This event is triggered after a notification is sent
        if (mds_state.export_mode == 0x01) {
            // Send next chunk
            send_chunk_notification(gatts_if);
        }
        break;
    }

    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        mds_state.mtu = param->mtu.mtu;
        break;

    case ESP_GATTS_DISCONNECT_EVT: {
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
        
        // Reset MDS state
        mds_state.subscribed = false;
        mds_state.conn_id = 0xFFFF;
        mds_state.export_mode = 0;

        // Restart advertising
        esp_ble_gap_start_advertising(&adv_params);
        break;
    }

    case ESP_GATTS_START_EVT:
        if (param->start.status == ESP_GATT_OK) {
            ESP_LOGI(GATTS_TAG, "GATTS service started successfully");
        } else {
            ESP_LOGE(GATTS_TAG, "GATTS service start failed, error status = %d", param->start.status);
        }
        break;

    case ESP_GATTS_CONGEST_EVT: {
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONGEST_EVT, congested: %d", param->congest.congested);
        mds_state.congested = param->congest.congested;
        
        // If congestion cleared, resume sending chunks
        if (!mds_state.congested && mds_state.export_mode == 0x01) {
            send_chunk_notification(gatts_if);
        }
        break;
    }

    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        }
        break;
    case ESP_GATTS_READ_EVT:
    case ESP_GATTS_WRITE_EVT:
    case ESP_GATTS_EXEC_WRITE_EVT:
    case ESP_GATTS_MTU_EVT:
    case ESP_GATTS_CONF_EVT:
    case ESP_GATTS_UNREG_EVT:
    case ESP_GATTS_CREATE_EVT:
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
    case ESP_GATTS_ADD_CHAR_EVT:
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
    case ESP_GATTS_DELETE_EVT:
    case ESP_GATTS_START_EVT:
    case ESP_GATTS_STOP_EVT:
    case ESP_GATTS_CONNECT_EVT:
    case ESP_GATTS_DISCONNECT_EVT:
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    case ESP_GATTS_RESPONSE_EVT:
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    case ESP_GATTS_SET_ATTR_VAL_EVT:
    case ESP_GATTS_SEND_SERVICE_CHANGE_EVT:
        // Forward all other events to the profile handler
        break;
    default:
        break;
    }

    // Forward all events to the profile handler
    gatts_profile_event_handler(event, gatts_if, param);
}

static void initialize_console() {
    /* Disable buffering on stdin and stdout */
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);  // Revert to using the existing API

    /* Initialize the console */
    esp_console_config_t console_config = {
        .max_cmdline_args = 8,
        .max_cmdline_length = 256,
    };
    ESP_ERROR_CHECK(esp_console_init(&console_config));

    /* Configure linenoise line completion library */
    linenoiseSetMultiLine(1);
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback((linenoiseHintsCallback *)&esp_console_get_hint);
    linenoiseHistorySetMaxLen(10);
}

// Add a function to update the URI with device info
static void update_data_uri(void) {
    // Get device info from Memfault
    sMemfaultDeviceInfo info;
    memfault_platform_get_device_info(&info);
    
    // Format the URI with the device serial
    snprintf(mds_data_uri, sizeof(mds_data_uri),
             "https://chunks.memfault.com/api/v0/chunks/%s",
             info.device_serial);
    
    ESP_LOGI(GATTS_TAG, "Updated MDS data URI: %s", mds_data_uri);
}

// Add task handle at the top with other static variables
static TaskHandle_t console_task_handle = NULL;

// Move the console loop into its own task function
static void console_task(void *pvParameters) {
    /* Prompt to be printed before each line */
    const char* prompt = LOG_COLOR_I "esp32> " LOG_RESET_COLOR;

    /* Figure out if the terminal supports escape sequences */
    int probe_status = linenoiseProbe();
    if (probe_status) { /* zero indicates success */
        printf("\n"
               "Your terminal application does not support escape sequences.\n"
               "Line editing and history features are disabled.\n"
               "On Windows, try using Putty instead.\n");
        linenoiseSetDumbMode(1);
        prompt = "esp32> ";
    }

    /* Main loop */
    while (true) {
        /* Get a line using linenoise.
         * The line is returned when ENTER is pressed.
         */
        char* line = linenoise(prompt);
        if (line == NULL) { /* Ignore empty lines */
            continue;
        }

        /* Add the command to the history */
        linenoiseHistoryAdd(line);

        /* Try to run the command */
        int ret;
        esp_err_t err = esp_console_run(line, &ret);
        if (err == ESP_ERR_NOT_FOUND) {
            printf("Unrecognized command\n");
        } else if (err == ESP_ERR_INVALID_ARG) {
            // command was empty
        } else if (err == ESP_OK && ret != ESP_OK) {
            printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(ret));
        } else if (err != ESP_OK) {
            printf("Internal error: %s\n", esp_err_to_name(err));
        }

        /* linenoise allocates line buffer on the heap, so need to free it */
        linenoiseFree(line);
    }
}

// Update app_main to create the console task instead of running the loop
void app_main(void)
{
    // Initialize Memfault before starting BLE advertising
    memfault_boot();
    memfault_device_info_dump();
    
    // Update MDS data URI with device serial
    update_data_uri();

    esp_err_t ret;

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // Initialize BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    // Initialize Bluedroid
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    // Register GATTS callback
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    // Register GAP callback
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }

    // Set device name before configuring advertising data
    ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", ret);
        return;
    }

    // Configure advertising data
    ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        return;
    }

    // Configure scan response data
    ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
        return;
    }

    // Then register the application
    ret = esp_ble_gatts_app_register(PROFILE_APP_ID);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    // Initialize console
    initialize_console();

    /* Register commands */
    esp_console_register_help_command();

    // Create console task
    xTaskCreate(console_task, "console", 8192, NULL, 2, &console_task_handle);
}
