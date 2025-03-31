#include <math.h>
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "ble_gatt.h"
#include "main.h"

static const char *TAG = "BLE";
uint16_t gatt_chr_battery_level_val_handle;
uint16_t gatt_chr_sync_signal_duration_val_handle;
uint16_t conn_hdl_ext;
uint16_t battery_level_descriptor_config = 0x0000;  // Initially disable notification and indication
uint16_t sync_signal_duration_descriptor_config = 0x0000;  // Initially disable notification and indication
uint8_t ble_addr_type;
system_status_t *system_status_addr;
void (*update_screen_from_BLE_cb_addr)(uint8_t);

static int gatt_chr_manufacturer_name_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_chr_battery_level_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_dsc_battery_level_desciptor_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_chr_wifi_ssid_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_chr_wifi_password_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_chr_POI_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_chr_max_sync_error_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_chr_target_imu_sample_period_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_chr_sync_signal_pulse_width_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_chr_sync_signal_duration_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_dsc_sync_signal_duration_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_chr_onboard_imu_status_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_chr_onboard_imu_sample_period_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

static int ble_gap_event(struct ble_gap_event *event, void *arg);
static void ble_app_advertise(void);
static void ble_app_on_sync(void);
static void ble_app_on_reset(int reason);
static void host_task(void *param);
void start_ble(system_status_t *system_status, void (*update_screen_from_BLE_cb)(uint8_t));
void stop_ble(void);

// -------------------------------------- Structure of the gatt services --------------------------------------------
static const struct ble_gatt_svc_def gatt_svcs[] = {
    // Device Information Service
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(DEVICE_INFO_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[])
        {
            // Characteristic: Manufacturer Name
            {
                .uuid = BLE_UUID16_DECLARE(MANUFACTURER_NAME_UUID),
                .flags = BLE_GATT_CHR_F_READ,
                .access_cb = gatt_chr_manufacturer_name_cb
            },

            // TERMINATE
            {
                0
            }
        }
    },

    // Battery Service
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BATTERY_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[])
        {
            // Characteristic: Battery Level
            {
                .uuid = BLE_UUID16_DECLARE(BATTERY_LEVEL_UUID),
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .access_cb = gatt_chr_battery_level_cb,
                .val_handle = &gatt_chr_battery_level_val_handle,
                .descriptors = (struct ble_gatt_dsc_def[])
                {
                    // Battery Level Descriptor
                    {
                        .uuid = BLE_UUID16_DECLARE(CLIENT_CHARACTERISTIC_CONFIGURATION_DESCRIPTOR_UUID),
                        .att_flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                        .access_cb = gatt_dsc_battery_level_desciptor_cb
                    },

                    // TERMINATE
                    {
                        0
                    }
                }
            },

            // TERMINATE
            {
                0
            }
        }
    },

    // WiFi SSID and Password Service
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID128_DECLARE(WIFI_SERVICE_UUID_LITTLE_ENDIAN),
        .characteristics = (struct ble_gatt_chr_def[])
        {
            // Characteristic: Wi-Fi SSID
            {
                .uuid = BLE_UUID128_DECLARE(WIFI_SSID_UUID_LITTLE_ENDIAN),
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .access_cb = gatt_chr_wifi_ssid_cb
            },

            // Characteristic: Wi-Fi Password
            {
                .uuid = BLE_UUID128_DECLARE(WIFI_PASSWORD_UUID_LITTLE_ENDIAN),
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .access_cb = gatt_chr_wifi_password_cb
            },

            // TERMINATE
            {
                0
            }
        }
    },

    // Synchronisation Parameters Service
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID128_DECLARE(SYNCHRONISATION_PARAMETERS_SERVICE_UUID_LITTLE_ENDIAN),
        .characteristics = (struct ble_gatt_chr_def[])
        {
            // Characteristic: Point of Interest
            {
                .uuid = BLE_UUID128_DECLARE(POI_UUID_LITTLE_ENDIAN),
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .access_cb = gatt_chr_POI_cb
            },

            // Characteristic: Maximum Synchronisation Error (ms)
            {
                .uuid = BLE_UUID128_DECLARE(MAX_SYNC_ERROR_UUID_LITTLE_ENDIAN),
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .access_cb = gatt_chr_max_sync_error_cb
            },

            // Characteristic: Target IMU Sample Period (ms)
            {
                .uuid = BLE_UUID128_DECLARE(TARGET_IMU_SAMPLE_PERIOD_UUID_LITTLE_ENDIAN),
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .access_cb = gatt_chr_target_imu_sample_period_cb
            },

            // Characteristic: Synchronisation Signal Pulse Width (ms)
            {
                .uuid = BLE_UUID128_DECLARE(SYNC_SIGNAL_PULSE_WIDTH_UUID_LITTLE_ENDIAN),
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .access_cb = gatt_chr_sync_signal_pulse_width_cb
            },

            // Characteristic: Synchronisation Signal Duration (ms)
            {
                .uuid = BLE_UUID128_DECLARE(SYNC_SIGNAL_DURATION_UUID_LITTLE_ENDIAN),
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .access_cb = gatt_chr_sync_signal_duration_cb,
                .val_handle = &gatt_chr_sync_signal_duration_val_handle,
                .descriptors = (struct ble_gatt_dsc_def[])
                {
                    {
                        .uuid = BLE_UUID16_DECLARE(CLIENT_CHARACTERISTIC_CONFIGURATION_DESCRIPTOR_UUID),
                        .att_flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                        .access_cb = gatt_dsc_sync_signal_duration_cb
                    },

                    // TERMINATE
                    {
                        0
                    }
                }

            },

            // TERMINATE
            {
                0
            }
        }
    },

    // On-board IMU Settings Service
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID128_DECLARE(ONBOARD_IMU_SETTINGS_SERVICE_UUID_LITTLE_ENDIAN),
        .characteristics = (struct ble_gatt_chr_def[])
        {
            // Characteristic: On-board IMU Status
            {
                .uuid = BLE_UUID128_DECLARE(ONBOARD_IMU_STATUS_UUID_LITTLE_ENDIAN),
                .flags = BLE_GATT_CHR_F_READ,
                .access_cb = gatt_chr_onboard_imu_status_cb
            },

            // Characteristic: On-board IMU Sample Period
            {
                .uuid = BLE_UUID128_DECLARE(ONBOARD_IMU_SAMPLE_PERIOD_UUID_LITTLE_ENDIAN),
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .access_cb = gatt_chr_onboard_imu_sample_period_cb
            },

            // TERMINATE
            {
                0
            }
        }
    },

    // TERMINATE
    {
        0
    }
};


// -------------------------------------- GATT Service Callbacks --------------------------------------------
// Callback function for the manufacturer name
static int gatt_chr_manufacturer_name_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    rc = os_mbuf_append(ctxt->om, MANUFACTURER_NAME, strlen(MANUFACTURER_NAME));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

// Callback function for battery level
static int gatt_chr_battery_level_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    uint8_t battery_level_look_up_table[5] = {100, 75, 50, 25, 5};
    uint8_t battery_level = battery_level_look_up_table[system_status_addr->battery_estimated_soc - 1];
    rc = os_mbuf_append(ctxt->om, &battery_level, sizeof(battery_level));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

// Callback function for battery level descriptor
static int gatt_dsc_battery_level_desciptor_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC)
    {
        rc = os_mbuf_append(ctxt->om, &battery_level_descriptor_config, sizeof(battery_level_descriptor_config));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else
    {
        rc = os_mbuf_copydata(ctxt->om, 0, ctxt->om->om_len, &battery_level_descriptor_config);
        if (rc != 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        ESP_LOGI(TAG, "Battery level CCCD writed. value = %#x", battery_level_descriptor_config);
        return 0;
    }
}

// Callback function for Wi-Fi SSID
static int gatt_chr_wifi_ssid_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
        rc = os_mbuf_append(ctxt->om, system_status_addr->wifi_ssid, strlen(system_status_addr->wifi_ssid));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else
    {
        int data_len = OS_MBUF_PKTLEN(ctxt->om);

        // Avoid too long invalid SSID input
        if (data_len > 31)
        {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        
        rc = os_mbuf_copydata(ctxt->om, 0, data_len, system_status_addr->wifi_ssid);
        if (rc != 0)
        {
            return BLE_ATT_ERR_UNLIKELY;
        }

        system_status_addr->wifi_ssid[data_len] = '\0';  // Null terminate
        update_screen_from_BLE_cb_addr(1);  // Refresh Screen
        return 0;
    }
}

// Callback function for Wi-Fi Password
static int gatt_chr_wifi_password_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
        rc = os_mbuf_append(ctxt->om, system_status_addr->wifi_password, strlen(system_status_addr->wifi_password));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else
    {
        int data_len = OS_MBUF_PKTLEN(ctxt->om);

        // Avoid too long invalid SSID input
        if (data_len > 63)
        {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        
        rc = os_mbuf_copydata(ctxt->om, 0, data_len, system_status_addr->wifi_password);
        if (rc != 0)
        {
            return BLE_ATT_ERR_UNLIKELY;
        }

        system_status_addr->wifi_password[data_len] = '\0';  // Null terminate
        return 0;
    }
}

// Callback function for Point of Interest
static int gatt_chr_POI_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
        rc = os_mbuf_append(ctxt->om, &system_status_addr->poi, sizeof(system_status_addr->poi));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else
    {
        rc = os_mbuf_copydata(ctxt->om, 0, ctxt->om->om_len, &system_status_addr->poi);
        if (rc != 0)
        {
            return BLE_ATT_ERR_UNLIKELY;
        }
        update_screen_from_BLE_cb_addr(2);
        return 0;
    }

}

// Callback function for Maximum Synchronisation Error (minimum 0.001 ms)
static int gatt_chr_max_sync_error_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // Float value is encoded into UTF-8 string for read and write operations
    char char_buffer[16];  // Size can be changed according to need
    int rc;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
        sprintf(char_buffer, "%.3f", system_status_addr->maximum_sync_error);
        rc = os_mbuf_append(ctxt->om, char_buffer, strlen(char_buffer));
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else
    {
        int data_len = OS_MBUF_PKTLEN(ctxt->om);

        // Avoid too long input (self defined)
        if (data_len > 15)
        {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        rc = os_mbuf_copydata(ctxt->om, 0, data_len, char_buffer);
        if (rc != 0)
        {
            return BLE_ATT_ERR_UNLIKELY;
        }

        char_buffer[data_len] = '\0';  // NULL Terminate

        float float_buffer = strtof(char_buffer, NULL);  // Convert to float
        float_buffer = floor(float_buffer * 1000) / 1000;  // truncate to 3 decimals
        system_status_addr->maximum_sync_error = float_buffer;
        update_screen_from_BLE_cb_addr(3);
    }
    return 0;
}

// Callback function for Target IMU Sample Period (minimum 0.001 ms, should be integer multiplier of max sync error)
static int gatt_chr_target_imu_sample_period_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // Float value is encoded into UTF-8 string for read and write operations
    char char_buffer[16];  // Size can be changed according to need
    int rc;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
        sprintf(char_buffer, "%.3f", system_status_addr->target_imu_sample_period);
        rc = os_mbuf_append(ctxt->om, char_buffer, strlen(char_buffer));
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else
    {
        int data_len = OS_MBUF_PKTLEN(ctxt->om);

        // Avoid too long input (self defined)
        if (data_len > 15)
        {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        rc = os_mbuf_copydata(ctxt->om, 0, data_len, char_buffer);
        if (rc != 0)
        {
            return BLE_ATT_ERR_UNLIKELY;
        }

        char_buffer[data_len] = '\0';  // NULL Terminate

        float float_buffer = strtof(char_buffer, NULL);  // Convert to float
        float_buffer = floor(float_buffer * 1000) / 1000;  // truncate to 3 decimals
        system_status_addr->target_imu_sample_period = float_buffer;
        update_screen_from_BLE_cb_addr(4);
    }
    return 0;
}

// Callback function for Synchronisation Signal Pulse Width (minimum 0.001 ms)
static int gatt_chr_sync_signal_pulse_width_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // Float value is encoded into UTF-8 string for read and write operations
    char char_buffer[16];  // Size can be changed according to need
    int rc;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
        sprintf(char_buffer, "%.3f", system_status_addr->sync_signal_pulse_width);
        rc = os_mbuf_append(ctxt->om, char_buffer, strlen(char_buffer));
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else
    {
        int data_len = OS_MBUF_PKTLEN(ctxt->om);

        // Avoid too long input (self defined)
        if (data_len > 15)
        {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        rc = os_mbuf_copydata(ctxt->om, 0, data_len, char_buffer);
        if (rc != 0)
        {
            return BLE_ATT_ERR_UNLIKELY;
        }

        char_buffer[data_len] = '\0';  // NULL Terminate

        float float_buffer = strtof(char_buffer, NULL);  // Convert to float
        float_buffer = floor(float_buffer * 1000) / 1000;  // truncate to 3 decimals
        system_status_addr->sync_signal_pulse_width = float_buffer;
        update_screen_from_BLE_cb_addr(5);
    }
    return 0;
}

// Callback function for Synchronisation Signal Duration (ms)
static int gatt_chr_sync_signal_duration_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // Float value is encoded into UTF-8 string for read operation
    char char_buffer[16];  // Size can be changed according to need
    int rc;
    sprintf(char_buffer, "%.3f", system_status_addr->sync_signal_duration);
    rc = os_mbuf_append(ctxt->om, char_buffer, strlen(char_buffer));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

// Callback function for Synchronisation Signal Duration Descriptor
static int gatt_dsc_sync_signal_duration_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC)
    {
        rc = os_mbuf_append(ctxt->om, &sync_signal_duration_descriptor_config, sizeof(sync_signal_duration_descriptor_config));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else
    {
        rc = os_mbuf_copydata(ctxt->om, 0, ctxt->om->om_len, &sync_signal_duration_descriptor_config);
        if (rc != 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        ESP_LOGI(TAG, "Synchronisation signal duration CCCD writed. value = %#x", sync_signal_duration_descriptor_config);
        return 0;
    }
}

// Callback function for On-board IMU Status
static int gatt_chr_onboard_imu_status_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    rc = os_mbuf_append(ctxt->om, &system_status_addr->onboard_imu_status, sizeof(system_status_addr->onboard_imu_status));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

// Callback function for On-board IMU Sample Period
static int gatt_chr_onboard_imu_sample_period_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
        rc = os_mbuf_append(ctxt->om, &system_status_addr->onboard_imu_sample_period, sizeof(system_status_addr->onboard_imu_sample_period));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else
    {
        rc = os_mbuf_copydata(ctxt->om, 0, ctxt->om->om_len, &system_status_addr->onboard_imu_sample_period);
        if (rc != 0)
        {
            return BLE_ATT_ERR_UNLIKELY;
        }
        update_screen_from_BLE_cb_addr(6);
        return 0;
    }
}

// -------------------------------------- GAP Events --------------------------------------------
// GAP Events
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "BLE_GAP_EVENT_CONNECT %s", event->connect.status == 0 ? "OK" : "Failed");
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        conn_hdl_ext = event->connect.conn_handle;
        system_status_addr->BLE_connection_status = 1;
        update_screen_from_BLE_cb_addr(0);
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE_GAP_EVENT_DISCONNECT");
        system_status_addr->BLE_connection_status = 0;
        update_screen_from_BLE_cb_addr(0);
        ble_app_advertise();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "BLE_GAP_EVENT_ADV_COMPLETE");
        if (system_status_addr->BLE_advertisement_status)  // If not manually terminated
        {
            ble_app_advertise();
        }
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "BLE_GAP_EVENT_SUBSCRIBE");
        break;
    default:
        break;
    }
    return 0;
}

// -------------------------------------- Enable Advertising --------------------------------------------
// Enable Advertising
static void ble_app_advertise(void)
{
    int rc;

    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_DISC_LTD;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    fields.name = (uint8_t *)ble_svc_gap_device_name();
    fields.name_len = strlen(ble_svc_gap_device_name());
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting advertisement data; rc=%d\n", rc);
        return;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error enabling advertisement; rc=%d\n", rc);
        return;
    }
    system_status_addr->BLE_advertisement_status = 1;
}

// Host Task
static void host_task(void *param)
{
    nimble_port_run();  // Only return when nimble_port_stop() is executed

    nimble_port_freertos_deinit();
}

// Start BLE
void start_ble(system_status_t *system_status, void (*update_screen_from_BLE_cb)(uint8_t))
{
    system_status_addr = system_status;
    update_screen_from_BLE_cb_addr = update_screen_from_BLE_cb;
    int rc;

    ESP_ERROR_CHECK(nimble_port_init());

    ble_svc_gap_device_name_set(DEVICE_NAME);
    ble_svc_gap_device_appearance_set(DEVICE_APPEARANCE_VALUE);
    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svcs);
    assert (rc == 0);

    rc = ble_gatts_add_svcs(gatt_svcs);
    assert (rc == 0);

    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ble_hs_cfg.reset_cb = ble_app_on_reset;
    nimble_port_freertos_init(host_task);
}

// Stop BLE
void stop_ble(void)
{
    ESP_LOGI(TAG, "Stopping BLE");
    // Terminate Connection
    if (system_status_addr->BLE_connection_status)
    {
        int rc = ble_gap_terminate(conn_hdl_ext, BLE_ERR_REM_USER_CONN_TERM);
        if (rc == 0)
        {
            ESP_LOGI(TAG, "Connection terminated");
        }
        else
        {
            ESP_LOGI(TAG, "Connection termination failed, error code: %d", rc);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    system_status_addr->BLE_advertisement_status = 0;
    nimble_port_stop();
}

static void ble_app_on_sync(void)
{
    int rc;

    rc = ble_hs_id_infer_auto(0, &ble_addr_type);
    assert(rc == 0);

    ble_app_advertise();
}

static void ble_app_on_reset(int reason)
{
    ESP_LOGW(TAG, "Resetting state; reason=%d\n", reason);
}
