#ifndef MAIN_H
#define MAIN_H

#define SNTP_SYNC_TIMEOUT_MS 4000
#define WIFI_CONNECT_TIMEOUT_MS 10000
#define BATTERY_VOLTAGE_SAMPLE_PERIOD 15000
#define IMU_DATA_AUTOSAVE_SAMPLES 1000

#define MENU_Y 11
#define HLINE_Y 14
#define PAGE_SCROLLER_INITIAL_Y 18
#define SUB1_Y 30
#define SUB2_Y 46
#define SUB3_Y 62
#define SUBMENU_INDENT 9
#define VALUE_INDENT 5
#define CURSOR_INDENT 0
#define FONT_MENU u8g2_font_7x13B_mr
#define FONT_SUBMENU u8g2_font_6x13_mr
#define FONT_VALUE u8g2_font_6x10_mr
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define REFRESH_EVENT_FORCE_REFRESH 1
#define REFRESH_EVENT_TIME_CHANGE 1 << 1
#define REFRESH_EVENT_DATE_CHANGE 1 << 2
#define REFRESH_EVENT_TZ_CHANGE 1 << 3
#define REFRESH_EVENT_AUTOSYNC_STATUS_CHANGE 1 << 4
#define REFRESH_EVENT_WIFI_STATUS_CHANGE 1 << 5
#define REFRESH_EVENT_CHARGE_STATUS_CHANGE 1 << 6
#define REFRESH_EVENT_SYNC_COUNT_CHANGE 1 << 7
#define REFRESH_EVENT_BATTERY_VOLTAGE_CHANGE 1 << 8
#define REFRESH_EVENT_SYNC_SIGNAL_DURATION_CHANGE 1 << 9
#define REFRESH_EVENT_ONBOARD_IMU_STATUS_CHANGE 1 << 10
#define REFRESH_EVENT_ONBOARD_IMU_SAMPLE_PERIOD_CHANGE 1 << 11
#define REFRESH_EVENT_START_STOP_SYNCING 1 << 12
#define REFRESH_EVENT_SD_CONNECTION_STATUS_CHANGE 1 << 13
#define REFRESH_EVENT_SD_CAPACITY_INFO_CHANGE 1 << 14
#define REFRESH_EVENT_WIFI_RSSI_CHANGE 1 << 15
#define REFRESH_EVENT_WIFI_SSID_CHANGE 1 << 16
#define REFRESH_EVENT_POI_CHANGE 1 << 17
#define REFRESH_EVENT_MAX_SYNC_ERROR_CHANGE 1 << 18
#define REFRESH_EVENT_TARGET_IMU_SAMPLE_PERIOD_CHANGE 1 << 19
#define REFRESH_EVENT_SYNC_SIGNAL_PULSE_WIDTH_CHANGE 1 << 20

#define ICON_BATTERY_0 32
#define ICON_BATTERY_25 33
#define ICON_BATTERY_50 34
#define ICON_BATTERY_75 35
#define ICON_BATTERY_100 36
#define ICON_BATTERY_CHG 37
#define ICON_WIFI_CONNECTED_GOOD 38
#define ICON_WIFI_CONNECTED_WEAK 39
#define ICON_WIFI_CONNECTED_POOR 40
#define ICON_BT_CONNECTED 41

// Typedefs
typedef struct menu
{
    char name[15];
    struct menu *next_menu;
    struct submenu *submenu_ptr;
    uint8_t pageScroller_length;
} menu_t;

typedef struct submenu
{
    char name[20];
    char value[20];
    uint32_t key;
    struct submenu *next_submenu_ptr;
    void (*function_ptr)(void);
} submenu_t;

typedef struct system_status
{
    uint16_t sync_count;
    uint16_t sync_num;
    uint16_t poi;
    float maximum_sync_error;
    float target_imu_sample_period;
    float sync_signal_pulse_width;
    float sync_signal_duration;
    bool onboard_imu_status;
    uint16_t onboard_imu_sample_period;
    bool wifi_status;
    int8_t wifi_RSSI;
    bool BLE_connection_status;
    bool BLE_advertisement_status;  // Whether BLE is on or off
    bool sntp_status;
    bool is_syncing;
    bool sending_sync_signal;
    bool sd_connected;
    uint8_t charging_status; // 1 Completed; 2 Charging; 3 R/Fault; 4 NR/Fault
    float battery_voltage;
    uint8_t battery_estimated_soc; // 1: 100%; 2: 75%; 3: 50%; 4: 25%; 5: 5%
    char POSIX_tz[32];
    char wifi_ssid[32];
    char wifi_password[64];
} system_status_t;

#endif