#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "ble_gatt.h"
#include "main.h"

// If a call to (re)start advertising fails (e.g. the NimBLE host is still
// tearing down the previous connection and returns BLE_HS_EBUSY), retry
// after this delay instead of silently giving up forever.
#define BLE_ADV_RETRY_INTERVAL_US (500 * 1000)
#define BLE_WATCHDOG_INTERVAL_US (500 * 1000)
// If a phone drops its GATT client without sending a link-layer terminate,
// release the stale connection quickly and return to advertising.
#define BLE_CONNECTION_SUPERVISION_TIMEOUT (200)  // 2 seconds, in 10 ms units
#define BLE_DISCONNECT_DELAY_US (100 * 1000)
#define BLE_DISCONNECT_COMMAND (0x01)
#define BLE_HEARTBEAT_COMMAND (0x02)
#define BLE_CLIENT_HEARTBEAT_TIMEOUT_US (8 * 1000 * 1000)

static const char *TAG = "BLE";
static const uint8_t cached_gatt_disconnect_command[] = {0x00, 0xD1, 0x5C, 0x0A, 0x4E, 0x44};
static const uint8_t cached_gatt_heartbeat_command[] = {0x00, 0x48, 0x42, 0x0A, 0x4E, 0x44};
uint16_t gatt_chr_battery_level_val_handle;
uint16_t gatt_chr_sync_signal_duration_val_handle;
uint16_t conn_hdl_ext;
uint16_t battery_level_descriptor_config = 0x0000;  // Initially disable notification and indication
uint16_t sync_signal_duration_descriptor_config = 0x0000;  // Initially disable notification and indication
uint8_t ble_addr_type;
system_status_t *system_status_addr;
void (*update_screen_from_BLE_cb_addr)(uint8_t);
static esp_timer_handle_t adv_retry_timer = NULL;
static esp_timer_handle_t disconnect_timer = NULL;
static esp_timer_handle_t ble_watchdog_timer = NULL;
static uint16_t disconnect_conn_handle;
static volatile bool disconnect_pending = false;
static volatile bool heartbeat_monitoring = false;
static volatile uint32_t last_client_heartbeat_us = 0;
// Set for the duration of stop_ble(); prevents the disconnect handler (or a
// pending retry) from restarting advertising into a NimBLE stack that's
// concurrently being torn down by nimble_port_stop().
static volatile bool ble_stopping = false;

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
static int gatt_chr_disconnect_command_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int ble_schedule_client_disconnect(uint16_t conn_handle);

static int ble_gap_event(struct ble_gap_event *event, void *arg);
static void ble_app_advertise(void);
static void ble_adv_retry_timer_cb(void *arg);
static void ble_disconnect_timer_cb(void *arg);
static void ble_watchdog_timer_cb(void *arg);
static void ble_recover_disconnected_state(const char *source);
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

    // Connection Control Service
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID128_DECLARE(CONNECTION_CONTROL_SERVICE_UUID_LITTLE_ENDIAN),
        .characteristics = (struct ble_gatt_chr_def[])
        {
            {
                .uuid = BLE_UUID128_DECLARE(DISCONNECT_COMMAND_UUID_LITTLE_ENDIAN),
                .flags = BLE_GATT_CHR_F_WRITE,
                .access_cb = gatt_chr_disconnect_command_cb
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

        // Cache-safe fallback for Android phones which retained the GATT
        // database from firmware predating the Connection Control service.
        if (data_len == sizeof(cached_gatt_disconnect_command))
        {
            uint8_t command[sizeof(cached_gatt_disconnect_command)];
            rc = os_mbuf_copydata(ctxt->om, 0, data_len, command);
            if (rc != 0)
            {
                return BLE_ATT_ERR_UNLIKELY;
            }
            if (memcmp(command,
                       cached_gatt_disconnect_command,
                       sizeof(cached_gatt_disconnect_command)) == 0)
            {
                return ble_schedule_client_disconnect(conn_handle);
            }
            if (memcmp(command,
                       cached_gatt_heartbeat_command,
                       sizeof(cached_gatt_heartbeat_command)) == 0)
            {
                heartbeat_monitoring = true;
                last_client_heartbeat_us = (uint32_t)esp_timer_get_time();
                return 0;
            }
        }

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

static int ble_schedule_client_disconnect(uint16_t conn_handle)
{
    disconnect_conn_handle = conn_handle;
    disconnect_pending = true;
    esp_timer_stop(disconnect_timer);
    esp_err_t err = esp_timer_start_once(disconnect_timer, BLE_DISCONNECT_DELAY_US);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not schedule requested disconnect; err=%d", err);
        return BLE_ATT_ERR_UNLIKELY;
    }

    ESP_LOGI(TAG, "Disconnect requested by GATT client");
    return 0;
}

// The phone asks the peripheral to terminate the controller link itself.
// Delaying termination briefly allows the write response to reach Android.
static int gatt_chr_disconnect_command_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint8_t command;

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR ||
        OS_MBUF_PKTLEN(ctxt->om) != sizeof(command))
    {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    int rc = os_mbuf_copydata(ctxt->om, 0, sizeof(command), &command);
    if (rc != 0)
    {
        return BLE_ATT_ERR_UNLIKELY;
    }
    if (command == BLE_HEARTBEAT_COMMAND)
    {
        heartbeat_monitoring = true;
        last_client_heartbeat_us = (uint32_t)esp_timer_get_time();
        return 0;
    }
    if (command != BLE_DISCONNECT_COMMAND)
    {
        return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    }

    return ble_schedule_client_disconnect(conn_handle);
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
            // A failed connection attempt ends the current advertising
            // procedure, but it is not a connection.  The old code continued
            // below and left the on-screen Bluetooth icon stuck on.
            system_status_addr->BLE_connection_status = 0;
            update_screen_from_BLE_cb_addr(0);
            ble_app_advertise();
            break;
        }

        conn_hdl_ext = event->connect.conn_handle;
        disconnect_pending = false;
        heartbeat_monitoring = false;
        system_status_addr->BLE_connection_status = 1;
        update_screen_from_BLE_cb_addr(0);

        // Do not keep accepting additional connections; all runtime state and
        // notification handles in this application belong to one phone.
        if (ble_gap_adv_active())
        {
            ble_gap_adv_stop();
        }

        struct ble_gap_upd_params connection_params = {
            .itvl_min = 12,  // 15 ms
            .itvl_max = 24,  // 30 ms
            .latency = 0,
            .supervision_timeout = BLE_CONNECTION_SUPERVISION_TIMEOUT,
            .min_ce_len = 0,
            .max_ce_len = 0,
        };
        int rc = ble_gap_update_params(conn_hdl_ext, &connection_params);
        if (rc != 0)
        {
            ESP_LOGW(TAG, "Could not request short connection supervision timeout; rc=%d", rc);
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE_GAP_EVENT_DISCONNECT; reason=%d", event->disconnect.reason);
        disconnect_pending = false;
        heartbeat_monitoring = false;
        ble_recover_disconnected_state("GAP event");
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "BLE_GAP_EVENT_ADV_COMPLETE; reason=%d", event->adv_complete.reason);
        if (system_status_addr->BLE_advertisement_status &&
            !system_status_addr->BLE_connection_status)  // If not connected or manually terminated
        {
            ble_app_advertise();
        }
        break;
    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGI(TAG, "BLE_GAP_EVENT_CONN_UPDATE; status=%d", event->conn_update.status);
        break;
    case BLE_GAP_EVENT_TERM_FAILURE:
        ESP_LOGW(TAG, "BLE_GAP_EVENT_TERM_FAILURE; status=%d", event->term_failure.status);
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
// Schedules a retry of ble_app_advertise() after a failed (re)start attempt, instead of
// silently leaving the radio permanently off the air.
static void ble_app_advertise_schedule_retry(void)
{
    if (ble_stopping || !system_status_addr->BLE_advertisement_status)
    {
        return;
    }

    // Harmless if the timer isn't currently running; avoids stacking up retries if
    // ble_app_advertise() gets called again (e.g. from a disconnect event) while a
    // retry is already pending.
    esp_timer_stop(adv_retry_timer);
    esp_err_t err = esp_timer_start_once(adv_retry_timer, BLE_ADV_RETRY_INTERVAL_US);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to schedule advertising retry; err=%d", err);
    }
}

// Timer callback: simply re-attempts advertising; ble_app_advertise() re-arms itself
// again via this same mechanism if the retry also fails.
static void ble_adv_retry_timer_cb(void *arg)
{
    ble_app_advertise();
}

static void ble_disconnect_timer_cb(void *arg)
{
    if (ble_stopping || !disconnect_pending)
    {
        return;
    }

    struct ble_gap_conn_desc conn_desc;
    int find_rc = ble_gap_conn_find(disconnect_conn_handle, &conn_desc);
    if (find_rc != 0)
    {
        disconnect_pending = false;
        ble_recover_disconnected_state("disconnect timer");
        return;
    }

    int rc = ble_gap_terminate(disconnect_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    if (rc != 0)
    {
        ESP_LOGW(TAG, "Peripheral-initiated disconnect attempt failed; rc=%d", rc);
    }
}

static void ble_recover_disconnected_state(const char *source)
{
    bool screen_needs_update = system_status_addr->BLE_connection_status;

    disconnect_pending = false;
    heartbeat_monitoring = false;
    system_status_addr->BLE_connection_status = 0;
    battery_level_descriptor_config = 0x0000;
    sync_signal_duration_descriptor_config = 0x0000;

    if (screen_needs_update)
    {
        ESP_LOGW(TAG, "BLE watchdog cleared stale connection state (%s)", source);
        update_screen_from_BLE_cb_addr(0);
    }

    ble_app_advertise();
}

// Independent of GAP callbacks, verify that the recorded connection still
// exists and that the radio is advertising whenever it does not.
static void ble_watchdog_timer_cb(void *arg)
{
    if (ble_stopping || !system_status_addr->BLE_advertisement_status)
    {
        return;
    }

    if (system_status_addr->BLE_connection_status || disconnect_pending)
    {
        uint16_t handle = disconnect_pending ? disconnect_conn_handle : conn_hdl_ext;
        struct ble_gap_conn_desc conn_desc;
        int find_rc = ble_gap_conn_find(handle, &conn_desc);

        if (find_rc != 0)
        {
            disconnect_pending = false;
            heartbeat_monitoring = false;
            ble_recover_disconnected_state("connection-table check");
            return;
        }

        if (heartbeat_monitoring &&
            (uint32_t)esp_timer_get_time() - last_client_heartbeat_us >
                BLE_CLIENT_HEARTBEAT_TIMEOUT_US)
        {
            ESP_LOGW(TAG, "BLE client heartbeat expired; terminating stale connection");
            heartbeat_monitoring = false;
            disconnect_conn_handle = handle;
            disconnect_pending = true;
        }

        if (disconnect_pending)
        {
            int terminate_rc = ble_gap_terminate(handle, BLE_ERR_REM_USER_CONN_TERM);
            if (terminate_rc != 0)
            {
                ESP_LOGW(TAG, "BLE watchdog disconnect retry failed; rc=%d", terminate_rc);
            }
        }
        return;
    }

    if (!ble_gap_adv_active())
    {
        ESP_LOGW(TAG, "BLE watchdog found advertising stopped; restarting");
        ble_app_advertise();
    }
}

// Enable Advertising
static void ble_app_advertise(void)
{
    if (ble_stopping || !system_status_addr->BLE_advertisement_status)
    {
        return;
    }

    if (system_status_addr->BLE_connection_status)
    {
        return;
    }

    // A retry can race with another GAP event which has already restarted
    // advertising.  Treat that as success rather than repeatedly receiving
    // BLE_HS_EALREADY and rearming the timer.
    if (ble_gap_adv_active())
    {
        if (adv_retry_timer != NULL)
        {
            esp_timer_stop(adv_retry_timer);
        }
        return;
    }

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
        ESP_LOGE(TAG, "Error setting advertisement data; rc=%d. Retrying in %d ms\n", rc, BLE_ADV_RETRY_INTERVAL_US / 1000);
        ble_app_advertise_schedule_retry();
        return;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error enabling advertisement; rc=%d. Retrying in %d ms\n", rc, BLE_ADV_RETRY_INTERVAL_US / 1000);
        ble_app_advertise_schedule_retry();
        return;
    }
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
    ble_stopping = false;
    // This flag represents whether BLE is enabled, rather than whether the
    // controller happens to be transmitting an advertisement at this instant.
    // It therefore stays true while connected and while an advertising retry
    // is pending.
    system_status_addr->BLE_advertisement_status = 1;
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

    if (adv_retry_timer == NULL)
    {
        const esp_timer_create_args_t adv_retry_timer_args = {
            .callback = &ble_adv_retry_timer_cb,
            .name = "ble_adv_retry",
        };
        ESP_ERROR_CHECK(esp_timer_create(&adv_retry_timer_args, &adv_retry_timer));
    }
    if (disconnect_timer == NULL)
    {
        const esp_timer_create_args_t disconnect_timer_args = {
            .callback = &ble_disconnect_timer_cb,
            .name = "ble_disconnect",
        };
        ESP_ERROR_CHECK(esp_timer_create(&disconnect_timer_args, &disconnect_timer));
    }
    if (ble_watchdog_timer == NULL)
    {
        const esp_timer_create_args_t ble_watchdog_timer_args = {
            .callback = &ble_watchdog_timer_cb,
            .name = "ble_watchdog",
        };
        ESP_ERROR_CHECK(esp_timer_create(&ble_watchdog_timer_args, &ble_watchdog_timer));
    }

    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ble_hs_cfg.reset_cb = ble_app_on_reset;
    nimble_port_freertos_init(host_task);
}

// Stop BLE
void stop_ble(void)
{
    ESP_LOGI(TAG, "Stopping BLE");

    // Block any further advertise attempts (including the one the disconnect
    // handler below is about to make) for the rest of this shutdown, so nothing
    // can schedule a retry that fires after nimble_port_stop() tears the host down.
    ble_stopping = true;
    system_status_addr->BLE_advertisement_status = 0;
    if (adv_retry_timer != NULL)
    {
        esp_timer_stop(adv_retry_timer);
    }
    if (disconnect_timer != NULL)
    {
        esp_timer_stop(disconnect_timer);
    }
    if (ble_watchdog_timer != NULL)
    {
        esp_timer_stop(ble_watchdog_timer);
    }
    disconnect_pending = false;
    heartbeat_monitoring = false;

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

    // The disconnect event handled during ble_gap_terminate()/vTaskDelay() above may
    // have raced in a new retry before ble_stopping was checked; cancel it again now
    // that it's no longer possible for another one to be scheduled.
    if (adv_retry_timer != NULL)
    {
        esp_timer_stop(adv_retry_timer);
    }
    nimble_port_stop();
}

static void ble_app_on_sync(void)
{
    int rc;

    rc = ble_hs_id_infer_auto(0, &ble_addr_type);
    assert(rc == 0);

    ble_app_advertise();
    esp_timer_stop(ble_watchdog_timer);
    ESP_ERROR_CHECK(esp_timer_start_periodic(ble_watchdog_timer, BLE_WATCHDOG_INTERVAL_US));
}

static void ble_app_on_reset(int reason)
{
    ESP_LOGW(TAG, "Resetting state; reason=%d\n", reason);
}
