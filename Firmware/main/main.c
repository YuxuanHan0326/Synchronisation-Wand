#include <stdio.h>
#include <time.h>
#include <string.h>
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "esp_sntp.h"
#include "driver/gptimer.h"
#include "esp_wifi.h"
#include "esp_netif.h"

#include "u8g2_esp32_hal.h"
#include "wifi_connect.h"
#include "time_client.h"
#include "sd_card.h"
#include "icm20948.h"
#include "board_config.h"
#include "user_config.h"

#define SNTP_SYNC_TIMEOUT_MS 4000
#define WIFI_CONNECT_TIMEOUT_MS 10000
#define BATTERY_VOLTAGE_SAMPLE_PERIOD 15000

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
#define REFRESH_EVENT_SYNC_SIGNAL_PARAMETER_CHANGE 1 << 9
#define REFRESH_EVENT_ONBOARD_IMU_STATUS_CHANGE 1 << 10
#define REFRESH_EVENT_ONBOARD_IMU_SAMPLE_PERIOD_CHANGE 1 << 11
#define REFRESH_EVENT_START_STOP_SYNCING 1 << 12
#define REFRESH_EVENT_SD_CONNECTION_STATUS_CHANGE 1 << 13
#define REFRESH_EVENT_SD_CAPACITY_INFO_CHANGE 1 << 14
#define REFRESH_EVENT_WIFI_RSSI_CHANGE 1 << 15

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
    bool BLE_status;
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

// Function Declaration
static void IRAM_ATTR gpio_isr_cb_TS(void *args);
static void TS_pushed_task(void *params);
static void IRAM_ATTR gpio_isr_cb_CHG(void *args);
static void IRAM_ATTR gpio_isr_cb_SD(void *args);
static void IRAM_ATTR timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data);
static void CHG_changed_task(void *params);
static void sync_finished_task(void *params);
static void SD_card_detect_task(void *params);
static void detect_charge_status(void);
static void wifi_connected_cb(void);
static void wifi_disconnected_cb(void);
static void calculate_sync_signal_duration(void);
static uint8_t calculate_number_of_pulses(void);
static void ipv4_address_to_string(uint32_t addr, char *addr_string);
static void refresh_WiFi_RSSI(void *params);
static void menu_init(void);
static void screen_refresh(void *params);
static void refresh_time(void *params);
static void battery_voltage_sampling(void *params);
static uint8_t get_number_of_submenus(submenu_t *submenu);
static uint8_t calculate_pageScroller_length(menu_t *menu);
static void toggle_rtc_autoSync(void);
static void disable_rtc_autoSync(void);
static void enable_rtc_autoSync(void);
static void wifi_connection_control(void);
static void start_stop_sync_group(void);
static void sync_imu_now(void);
static void toggle_on_board_imu(void);
static void eject_sd_card(void);
static esp_err_t create_file_sync_data(struct tm current_time);
static void log_sync_data(int64_t timestamp, uint16_t sync_num, uint16_t point_of_interest, float max_error, uint8_t pulse_num);
static void enter_deep_sleep(void);
static esp_err_t icm20948_configure(icm20948_acce_fs_t acce_fs, icm20948_gyro_fs_t gyro_fs);
void icm_read_task(void *params);
static esp_err_t create_file_onboard_IMU_data(struct tm current_time);
static void log_IMU_data(icm20948_acce_value_t acce, icm20948_gyro_value_t gyro);

// Global Variables
u8g2_t u8g2;
QueueHandle_t interrupt_queue_TS;                    // Tactile Switch Interrupt Queue
TaskHandle_t CHG_status_change_notification_handler; // Charging Status Interrupt Notification
TaskHandle_t SD_card_detect_notification_handler;    // SD Card Detect Interrupt Notification
TaskHandle_t screen_refresh_notification_handler;    // Screen Refresh Request Notification
TaskHandle_t sync_finished_notification_handler;     // Sync Finished Notification
TaskHandle_t imu_read_suspend_handler;               // Used to suspend and continue imu read
gptimer_handle_t gptimer = NULL;
RTC_DATA_ATTR bool from_deep_sleep = 0;
RTC_DATA_ATTR system_status_t system_status_record; // Used to preserve system_status during deep sleep
system_status_t system_status = {
    .poi = DEFAULT_POINT_OF_INTEREST,
    .is_syncing = 0,
    .sending_sync_signal = 0,
    .onboard_imu_status = 0,
    .wifi_status = 0,
    .wifi_RSSI = 0,
    .BLE_status = 0,
    .sntp_status = 1,
    .sd_connected = 0,
    .POSIX_tz = DEFAULT_TZ,
    .wifi_ssid = DEFAULT_WIFI_SSID,
    .wifi_password = DEFAULT_WIFI_PASSWORD,
    .maximum_sync_error = DEFAULT_MAX_SYNC_ERROR,
    .target_imu_sample_period = DEFAULT_TARGET_IMU_SAMPLE_PERIOD,
    .sync_signal_pulse_width = DEFAULT_SYNC_SIGNAL_PULSE_WIDTH,
    .onboard_imu_sample_period = DEFAULT_ONBOARD_IMU_SAMPLE_PERIOD}; // Struct to store some of the status
bool prev_cd_gpio_status;
volatile bool EM_gpio_status = 0;
volatile uint8_t number_of_pulses;
char file_syncData_path[40] = {0};
char file_IMUData_path[40] = {0};
sd_capacity_t sd_capacity_info;
uint8_t wifi_quality = 3; // 3: Good, 2: Weak; 3: Poor
bool sleep_flag = 0;
static icm20948_handle_t icm20948 = NULL;
FILE *fptr_imu = NULL;

// Screen related global variables
// Front Menu
submenu_t FM_sub_1 = {.name = " START SYNC ", .function_ptr = &start_stop_sync_group, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_START_STOP_SYNCING};
submenu_t FM_sub_2 = {.name = "* SYNC IMU *", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_SYNC_COUNT_CHANGE};
submenu_t FM_sub_3 = {.name = " POI: ", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH};
submenu_t FM_sub_4 = {.name = " Max. Error: ", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_SYNC_SIGNAL_PARAMETER_CHANGE};
submenu_t FM_sub_5 = {.name = " TGT IMU T: ", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_SYNC_SIGNAL_PARAMETER_CHANGE};
submenu_t FM_sub_6 = {.name = " Pulse W: ", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_SYNC_SIGNAL_PARAMETER_CHANGE};
submenu_t FM_sub_7 = {.name = " SIG Dura: ", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_SYNC_SIGNAL_PARAMETER_CHANGE};
submenu_t FM_sub_8 = {.name = " On Board IMU ", .function_ptr = &toggle_on_board_imu, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_ONBOARD_IMU_STATUS_CHANGE, .value = "OFF"};
submenu_t FM_sub_9 = {.name = " IMU sample-T: ", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_ONBOARD_IMU_SAMPLE_PERIOD_CHANGE};
menu_t front_menu = {.name = "FRONT MENU", .submenu_ptr = &FM_sub_1};
// WiFi Menu
submenu_t WiFi_sub_1 = {.name = " Status: ", .function_ptr = &wifi_connection_control, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_WIFI_STATUS_CHANGE, .value = "D/C"};
submenu_t WiFi_sub_2 = {.name = " SSID: ", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH};
submenu_t WiFi_sub_3 = {.name = " RSSI: ", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_WIFI_RSSI_CHANGE, .value = "N/A"};
submenu_t WiFi_sub_4 = {.name = " Authmode: ", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_WIFI_STATUS_CHANGE, .value = "N/A"};
submenu_t WiFi_sub_5 = {.name = " IP: ", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_WIFI_STATUS_CHANGE, .value = "N/A"};
submenu_t WiFi_sub_6 = {.name = " Mask: ", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_WIFI_STATUS_CHANGE, .value = "N/A"};
submenu_t WiFi_sub_7 = {.name = " GW: ", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_WIFI_STATUS_CHANGE, .value = "N/A"};
menu_t WiFi_menu = {.name = "Wi-Fi", .submenu_ptr = &WiFi_sub_1};

// RTC Menu
submenu_t rtc_sub_1 = {.name = " TZ: ", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH};
submenu_t rtc_sub_2 = {.name = " Time: ", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_TIME_CHANGE};
submenu_t rtc_sub_3 = {.name = " Date: ", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_DATE_CHANGE};
submenu_t rtc_sub_4 = {.name = " AutoSync ", .function_ptr = &toggle_rtc_autoSync, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_AUTOSYNC_STATUS_CHANGE, .value = "ON"};
submenu_t rtc_sub_5 = {.name = " SVR: ", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH, .value = "pool.ntp.org"};
menu_t rtc_menu = {.name = "RTC", .submenu_ptr = &rtc_sub_1};
// SD Card Menu
submenu_t sd_sub_1 = {.name = " Status: ", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_SD_CONNECTION_STATUS_CHANGE, .value = "D/C"};
submenu_t sd_sub_2 = {.name = " Usage: ", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_SD_CONNECTION_STATUS_CHANGE, .value = "N/A"};
submenu_t sd_sub_3 = {.name = " EJECT ^ ", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_SD_CONNECTION_STATUS_CHANGE};
menu_t sd_menu = {.name = "SD Card", .submenu_ptr = &sd_sub_1};
// Battery Menu
submenu_t battery_sub_1 = {.name = " Status ", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_CHARGE_STATUS_CHANGE};
submenu_t battery_sub_2 = {.name = " Voltage: ", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_BATTERY_VOLTAGE_CHANGE};
submenu_t battery_sub_3 = {.name = " Est. SOC: ", .function_ptr = NULL, .key = REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_BATTERY_VOLTAGE_CHANGE};
menu_t battery_menu = {.name = "BATTERY", .submenu_ptr = &battery_sub_1};
// Other
menu_t *current_menu = NULL;
submenu_t *current_submenu = NULL;
submenu_t *sub1_disp = NULL;
submenu_t *sub2_disp = NULL;
submenu_t *sub3_disp = NULL;
uint8_t pageScroller_Y = PAGE_SCROLLER_INITIAL_Y;

// ------------------------------------------ App Main --------------------------------------------

void app_main(void)
{
    if (from_deep_sleep)
    {
        ESP_LOGI("Sleep", "Wake up from deep sleep");
        system_status = system_status_record;                // Restore system_status
        ESP_ERROR_CHECK(rtc_gpio_deinit(PIN_TS_DEEP_SLEEP)); // Deinit the Deep Sleep PIN from being an rtc gpio
    }

    // Configure Tactile Switch GPIOs
    gpio_config_t GPIO_config_TS = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 0,
        .pull_up_en = 1,
        .pin_bit_mask = (1ULL << PIN_TS_DEEP_SLEEP) | (1ULL << PIN_TS_MENU) | (1ULL << PIN_TS_SUBMENU) | (1ULL << PIN_TS_ACTIVATE)};
    ESP_ERROR_CHECK(gpio_config(&GPIO_config_TS));

    // Configure LED GPIOs and Electromagnet GPIO
    gpio_config_t GPIO_config_LED = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .pin_bit_mask = (1ULL << PIN_LED_B) | (1ULL << PIN_LED_O) | (1ULL << PIN_LED_R) | (1ULL << PIN_EM)};
    ESP_ERROR_CHECK(gpio_config(&GPIO_config_LED));

    // Configure Charging Status and Card Detect GPIOs
    gpio_config_t GPIO_config_CHG = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 0,
        .pull_up_en = 1,
        .pin_bit_mask = (1ULL << PIN_CHG_STAT1) | (1ULL << PIN_CHG_STAT2) | (1ULL << PIN_SD_DETECT)};
    ESP_ERROR_CHECK(gpio_config(&GPIO_config_CHG));

    // Initialize ADC1 to Detect Battery Voltage
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(PIN_VBAT_SENSE_ADC_CHANNEL, ADC_ATTEN_DB_12);

    // Initialise Timer for sync signal generation
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // Resolution = 1 us
        .intr_priority = 3        // Medium piority
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_on_alarm_cb};
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer)); // Enable Timer
    ESP_LOGI("GPTimer", "GPTimer Enabled");
    xTaskCreate(&sync_finished_task, "sync_finished_task", 4096, NULL, 2, &sync_finished_notification_handler); // This task has higher priority than others

    // Mount SD card if detected
    prev_cd_gpio_status = gpio_get_level(PIN_SD_DETECT);
    if (prev_cd_gpio_status == 0)
    {
        if (sd_card_init(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_DAT0, PIN_SD_DAT1, PIN_SD_DAT2, PIN_SD_DAT3) == ESP_OK)
        {
            system_status.sd_connected = 1;
            strcpy(sd_sub_1.value, "Connected");
            sd_sub_3.function_ptr = &eject_sd_card; // Enable the sd eject function
            sd_capacity_info = get_sd_capacity_info();

            // if the sd capacity info returned is valid:
            if (sd_capacity_info.result_available)
            {
                sprintf(sd_sub_2.value, "%.2f/%.2fGB", sd_capacity_info.used_GB, sd_capacity_info.total_GB);
            }
        }
    }

    // Enable GPIO ISR
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // Install ISR for Tactile Switches
    interrupt_queue_TS = xQueueCreate(10, sizeof(int));
    xTaskCreate(TS_pushed_task, "TS_pushed_task", 4096, NULL, 1, NULL);
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_TS_ACTIVATE, gpio_isr_cb_TS, (void *)PIN_TS_ACTIVATE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_TS_MENU, gpio_isr_cb_TS, (void *)PIN_TS_MENU));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_TS_SUBMENU, gpio_isr_cb_TS, (void *)PIN_TS_SUBMENU));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_TS_DEEP_SLEEP, gpio_isr_cb_TS, (void *)PIN_TS_DEEP_SLEEP));
    ESP_LOGI("Tactile Switch", "Tactile Switch ISR Installed");

    // Install ISR for Charging Status Change
    xTaskCreate(CHG_changed_task, "CHG_changed_task", 4096, NULL, 1, &CHG_status_change_notification_handler);
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_CHG_STAT1, gpio_isr_cb_CHG, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_CHG_STAT2, gpio_isr_cb_CHG, NULL));
    ESP_LOGI("Charging IC", "Charging Status Detection ISR Installed");

    // Install ISR for SD Card Detect
    xTaskCreate(SD_card_detect_task, "SD_card_detect_task", 4096, NULL, 2, &SD_card_detect_notification_handler);
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_SD_DETECT, gpio_isr_cb_SD, NULL));
    ESP_LOGI("SDMMC", "SD Card Detection ISR Installed");

    // Initialise menus
    menu_init();

    // Initialise u8g2 (I2C bus is initialised here simultaneously)
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.sda = PIN_SDA;
    u8g2_esp32_hal.scl = PIN_SCL;
    u8g2_esp32_hal_init(u8g2_esp32_hal);
    u8g2_Setup_sh1106_i2c_128x64_noname_f(
        &u8g2,
        U8G2_R0,
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb);
    u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);
    u8g2_InitDisplay(&u8g2);     // send init sequence to the display, display is in sleep mode after this,
    u8g2_SetPowerSave(&u8g2, 0); // wake up display

    // Configure On Board IMU
    esp_err_t ret = icm20948_configure(DEFAULT_ONBOARD_IMU_ACCELEROMETER_RANGE, DEFAULT_ONBOARD_IMU_GYROSCOPE_RANGE);
    if (ret != ESP_OK)
    {
        ESP_LOGE("On-board IMU", "ICM configuration failure");
    }
    ESP_LOGI("On-board IMU", "ICM20948 configuration successfull!");
    ESP_ERROR_CHECK(icm20948_sleep(icm20948)); // Put on_board imu to sleep

    xTaskCreate(&icm_read_task, "icm read task", 4096, NULL, 1, &imu_read_suspend_handler);

    // Start screen related tasks
    xTaskCreate(&refresh_time, "refresh_time", 4096, NULL, 1, NULL);                                     // Create task to refresh time displayed on screen
    xTaskCreate(&battery_voltage_sampling, "battery_voltage_sampling", 4096, NULL, 1, NULL);             // Create task to monitor battery voltage periodically
    xTaskCreate(&screen_refresh, "screen_refresh", 8192, NULL, 1, &screen_refresh_notification_handler); // Create Screen Refresh Task

    // Detect Charge Status from the Charging IC
    detect_charge_status();

    // Connect WiFi
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_connect_init(&wifi_connected_cb, &wifi_disconnected_cb);
    wifi_connect_sta(system_status.wifi_ssid, system_status.wifi_password, WIFI_CONNECT_TIMEOUT_MS);

    // Create task to update Wifi RSSI
    xTaskCreate(&refresh_WiFi_RSSI, "refresh_WiFi_RSSI", 4096, NULL, 1, NULL);

    // Initilize SNTP to enable auto time synchronising
    time_client_init();
    time_client_set_tz(system_status.POSIX_tz);

    // Main Loop
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ------------------------------------------ Functions for Tactile Switches --------------------------------------------
// ISR for Tactile Switches
static void IRAM_ATTR gpio_isr_cb_TS(void *args)
{
    int pinNumber = (int)args;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(interrupt_queue_TS, &pinNumber, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Task Triggered by Tactile Switches' Interrupts
static void TS_pushed_task(void *params)
{
    int pinNumber;
    while (xQueueReceive(interrupt_queue_TS, &pinNumber, portMAX_DELAY))
    {
        // Disable interrupt
        ESP_ERROR_CHECK(gpio_isr_handler_remove(pinNumber));

        uint16_t counter = 0;
        // Wait for release
        do
        {
            vTaskDelay(pdMS_TO_TICKS(20));
            counter++;
        } while (gpio_get_level(pinNumber) == 0);

        // Task
        ESP_LOGI("Tactile Switch", "Tactile Switch %d Triggered", pinNumber);
        switch (pinNumber)
        {
        // Menu TS pressed
        case PIN_TS_MENU:
            current_menu = current_menu->next_menu;      // Switch to next menu
            current_submenu = current_menu->submenu_ptr; // Update current_submenu

            // Update submenus to be displayed
            sub1_disp = current_menu->submenu_ptr;
            sub2_disp = sub1_disp != NULL ? sub1_disp->next_submenu_ptr : NULL;
            sub3_disp = sub2_disp != NULL ? sub2_disp->next_submenu_ptr : NULL;

            pageScroller_Y = PAGE_SCROLLER_INITIAL_Y; // Reset page scroller position

            xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_FORCE_REFRESH, eSetBits); // Request refresh
            break;

        // Submenu TS pressed
        case PIN_TS_SUBMENU:
            // Switch to next submenu tab
            if (current_submenu->next_submenu_ptr != NULL)
            {
                current_submenu = current_submenu->next_submenu_ptr;
                pageScroller_Y += current_menu->pageScroller_length;

                // If current_submenu goes out of screen, change the 3 submenus to be displayed
                if ((current_submenu != sub1_disp) && (current_submenu != sub2_disp) && (current_submenu != sub3_disp))
                {
                    sub1_disp = current_submenu;
                    sub2_disp = sub1_disp != NULL ? sub1_disp->next_submenu_ptr : NULL;
                    sub3_disp = sub2_disp != NULL ? sub2_disp->next_submenu_ptr : NULL;
                }
            }
            // If reaches the end
            else
            {
                current_submenu = current_menu->submenu_ptr;
                sub1_disp = current_menu->submenu_ptr;
                sub2_disp = sub1_disp != NULL ? sub1_disp->next_submenu_ptr : NULL;
                sub3_disp = sub2_disp != NULL ? sub2_disp->next_submenu_ptr : NULL;
                pageScroller_Y = PAGE_SCROLLER_INITIAL_Y; // Reset page scroller position
            }

            xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_FORCE_REFRESH, eSetBits); // Request refresh
            break;

        // Activate TS pressed
        case PIN_TS_ACTIVATE:
            if (current_submenu->function_ptr != NULL)
            {
                current_submenu->function_ptr();
            }
            break;

        // Deep Sleep TS pressed
        case PIN_TS_DEEP_SLEEP:
            // Only triggered if long pressed more than 3 seconds
            if (counter >= 150)
            {
                enter_deep_sleep();
            }
            else
            {
                break;
            }
        default:
            break;
        }

        // Re-enable interrupt
        ESP_ERROR_CHECK(gpio_isr_handler_add(pinNumber, gpio_isr_cb_TS, (void *)pinNumber));
    }
}

// ------------------------------------------ Function to Enter Deep Sleep --------------------------------------------
// Prepare and enter deep sleep
static void enter_deep_sleep(void)
{
    ESP_LOGI("Sleep", "Preparing to enter deep-sleep");
    // Stop the synchronising group
    if (system_status.is_syncing)
    {
        start_stop_sync_group();
    }

    // Stop On-board IMU
    if (system_status.onboard_imu_status)
    {
        toggle_on_board_imu();
    }

    // Eject SD
    if (system_status.sd_connected)
    {
        sd_card_deinit();
        system_status.sd_connected = 0;
    }

    // Stop Wi-Fi
    if (system_status.wifi_status)
    {
        wifi_disconnect();
    }

    // Suspend screen refresh task
    sleep_flag = 1;
    vTaskDelay(pdMS_TO_TICKS(100));

    // Turn Off Screen
    u8g2_ClearDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Configure Deep Sleep Pin
    ESP_ERROR_CHECK(rtc_gpio_init(PIN_TS_DEEP_SLEEP));
    ESP_ERROR_CHECK(rtc_gpio_pullup_en(PIN_TS_DEEP_SLEEP));
    ESP_ERROR_CHECK(rtc_gpio_pulldown_dis(PIN_TS_DEEP_SLEEP));
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(PIN_TS_DEEP_SLEEP, 0));

    from_deep_sleep = 1;
    system_status_record = system_status; // Record system_status

    ESP_LOGI("Sleep", "Going to sleep :)");
    esp_deep_sleep_start();
}

// ------------------------------------------ Functions for GPTimer (Sync Signal Generation) --------------------------------------------
// ISR for GPTimer
static void IRAM_ATTR timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    EM_gpio_status = !EM_gpio_status;
    gpio_set_level(PIN_EM, EM_gpio_status);
    if (number_of_pulses > 1)
    {
        number_of_pulses--;
    }
    else
    {
        vTaskNotifyGiveFromISR(sync_finished_notification_handler, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Task triggered when one sync finished
static void sync_finished_task(void *params)
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_ERROR_CHECK(gptimer_stop(gptimer));
        system_status.sending_sync_signal = 0;

        // Notify user the end of syncing
        gpio_set_level(PIN_LED_B, 0); // Turn off LED to indicate that the sync is finished
        ESP_LOGI("IMU SYNC", "Sync No.%d Finished", system_status.sync_count);
        sprintf(FM_sub_2.value, "No.%d", ++system_status.sync_count);
        strcpy(FM_sub_2.name, "* SYNC IMU *");
        FM_sub_2.function_ptr = &sync_imu_now;
        xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_SYNC_COUNT_CHANGE, eSetBits);
    }
}

// ------------------------------------------ Functions for Battery Charge Status --------------------------------------------
// ISR for Charging Status pins
static void IRAM_ATTR gpio_isr_cb_CHG(void *args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(CHG_status_change_notification_handler, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Task Triggered by the Charging Status Change Interrupt
static void CHG_changed_task(void *params)
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(5));
        detect_charge_status();
    }
}

// Detect Charge Status
static void detect_charge_status(void)
{
    int stat1 = gpio_get_level(PIN_CHG_STAT1);
    int stat2 = gpio_get_level(PIN_CHG_STAT2);
    const char *TAG = "Charging IC";

    // Case 1: Charge Completed
    if ((stat1 == 1) && (stat2 == 1))
    {
        ESP_LOGI(TAG, "Charge Completed");
        gpio_set_level(PIN_LED_O, 0);
        gpio_set_level(PIN_LED_R, 0);
        strcpy(battery_sub_1.value, "C/Complete");
        system_status.charging_status = 1;
    }
    // Case 2: Charge in Progress
    else if ((stat1 == 1) && (stat2 == 0))
    {
        ESP_LOGI(TAG, "Charge in Progress");
        gpio_set_level(PIN_LED_O, 1);
        gpio_set_level(PIN_LED_R, 0);
        strcpy(battery_sub_1.value, "Charging");
        system_status.charging_status = 2;
    }
    // Case 3: Recoverable Fault
    else if ((stat1 == 0) && (stat2 == 1))
    {
        ESP_LOGE(TAG, "Recoverable Fault");
        gpio_set_level(PIN_LED_O, 0);
        gpio_set_level(PIN_LED_R, 1);
        strcpy(battery_sub_1.value, "R/Fault");
        system_status.charging_status = 3;
    }
    // Case 4: Non-recoverable or Latch-off Fault
    else
    {
        ESP_LOGE(TAG, "Non-recoverable or Latch-off Fault");
        gpio_set_level(PIN_LED_O, 0);
        gpio_set_level(PIN_LED_R, 1);
        strcpy(battery_sub_1.value, "NR/Fault");
        system_status.charging_status = 4;
    }
    xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_CHARGE_STATUS_CHANGE | REFRESH_EVENT_FORCE_REFRESH, eSetBits);
}

// Periodically Sample battery voltage and estimate SOC
static void battery_voltage_sampling(void *params)
{
    while (true)
    {
        uint16_t adc_raw_accumulate = 0;
        // Only do sampling if not charging
        if (system_status.charging_status == 2)
        {
            strcpy(battery_sub_2.value, "N/A");
            strcpy(battery_sub_3.value, "N/A");
        }
        else
        {
            for (uint8_t i = 0; i < 10; i++)
            {
                adc_raw_accumulate += adc1_get_raw(PIN_VBAT_SENSE_ADC_CHANNEL);
            }
            system_status.battery_voltage = adc_raw_accumulate * 0.0001061675452; // Calculate average battery voltage in volts
            sprintf(battery_sub_2.value, "%.3fV", system_status.battery_voltage);

            // Predict SOC, thresholds based on 0.1C discharge at RTP
            if (system_status.battery_voltage >= 3.875)
            {
                if (system_status.battery_estimated_soc != 1)
                {
                    system_status.battery_estimated_soc = 1;
                    strcpy(battery_sub_3.value, "100%");
                    xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_FORCE_REFRESH, eSetBits);
                }
            }
            else if ((system_status.battery_voltage < 3.875) && (system_status.battery_voltage >= 3.667))
            {
                if (system_status.battery_estimated_soc != 2)
                {
                    system_status.battery_estimated_soc = 2;
                    strcpy(battery_sub_3.value, "75%");
                    xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_FORCE_REFRESH, eSetBits);
                }
            }
            else if ((system_status.battery_voltage < 3.667) && (system_status.battery_voltage >= 3.56))
            {
                if (system_status.battery_estimated_soc != 3)
                {
                    system_status.battery_estimated_soc = 3;
                    strcpy(battery_sub_3.value, "50%");
                    xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_FORCE_REFRESH, eSetBits);
                }
            }
            else if ((system_status.battery_voltage < 3.56) && (system_status.battery_voltage >= 3.42))
            {
                if (system_status.battery_estimated_soc != 4)
                {
                    system_status.battery_estimated_soc = 4;
                    strcpy(battery_sub_3.value, "25%");
                    xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_FORCE_REFRESH, eSetBits);
                }
            }
            else
            {
                if (system_status.battery_estimated_soc != 5)
                {
                    system_status.battery_estimated_soc = 5;
                    strcpy(battery_sub_3.value, "<5%");
                    xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_FORCE_REFRESH, eSetBits);
                }
            }
        }
        xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_BATTERY_VOLTAGE_CHANGE, eSetBits);
        vTaskDelay(pdMS_TO_TICKS(BATTERY_VOLTAGE_SAMPLE_PERIOD));
    }
}

// ------------------------------------------ Functions for SD Card --------------------------------------------
// ISR for SD Card Detect
static void IRAM_ATTR gpio_isr_cb_SD(void *args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(SD_card_detect_notification_handler, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Task Triggered by SD Card Detect Interrupt
static void SD_card_detect_task(void *params)
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Disable ISR
        ESP_ERROR_CHECK(gpio_isr_handler_remove(PIN_SD_DETECT));

        if (prev_cd_gpio_status == 0) // Indicating a falling edge, card removed
        {
            if (system_status.sd_connected) // If SD card is removed without proper ejection
            {
                ESP_LOGI("SDMMC", "Detected the removal of SD Card");
                // Turn off on_board IMU immediately
                if (system_status.onboard_imu_status)
                {
                    toggle_on_board_imu();
                }

                // Terminate the synchronising group immediately
                if (system_status.is_syncing)
                {
                    start_stop_sync_group();
                }

                sd_card_deinit();
                system_status.sd_connected = 0;
                strcpy(sd_sub_1.value, "D/C");
                strcpy(sd_sub_2.value, "N/A");
                sd_sub_3.function_ptr = NULL; // Disable sd eject function
                xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_SD_CONNECTION_STATUS_CHANGE | REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_SD_CAPACITY_INFO_CHANGE, eSetBits);
            }
        }
        else // Indicating a rising edge, card inserted
        {
            ESP_LOGI("SDMMC", "Detected the insertion of SD Card");
            if (system_status.sd_connected == 0)
            {
                if (sd_card_init(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_DAT0, PIN_SD_DAT1, PIN_SD_DAT2, PIN_SD_DAT3) == ESP_OK)
                {
                    system_status.sd_connected = 1;
                    strcpy(sd_sub_1.value, "Connected");
                    sd_sub_3.function_ptr = &eject_sd_card; // Enable sd eject function
                    sd_capacity_info = get_sd_capacity_info();
                    // if the sd capacity info returned is valid:
                    if (sd_capacity_info.result_available)
                    {
                        sprintf(sd_sub_2.value, "%.2f/%.2fGB", sd_capacity_info.used_GB, sd_capacity_info.total_GB);
                    }
                    xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_SD_CONNECTION_STATUS_CHANGE | REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_SD_CAPACITY_INFO_CHANGE, eSetBits);
                }
            }
        }

        vTaskDelay(100);
        prev_cd_gpio_status = gpio_get_level(PIN_SD_DETECT); // update cd gpio status

        // Re-enable ISR
        ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_SD_DETECT, gpio_isr_cb_SD, NULL));
    }
}

// Function to safe eject SD Card
static void eject_sd_card(void)
{
    // sd card can only be ejected safely if currently no data logging
    if (system_status.is_syncing == 0)
    {
        if (system_status.onboard_imu_status == 0)
        {
            sd_card_deinit();
            system_status.sd_connected = 0;
            strcpy(sd_sub_1.value, "D/C");
            strcpy(sd_sub_2.value, "N/A");
            sd_sub_3.function_ptr = NULL; // Disable sd eject function
            xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_SD_CONNECTION_STATUS_CHANGE | REFRESH_EVENT_FORCE_REFRESH | REFRESH_EVENT_SD_CAPACITY_INFO_CHANGE, eSetBits);
        }
        else
        {
            ESP_LOGW("SDMMC", "Please turn off the on-board imu first");
        }
    }
    else
    {
        ESP_LOGW("SDMMC", "Please terminate the current synchronising group first");
    }
}

// ------------------------------------------ Functions for Wi-Fi --------------------------------------------
// Callback function when wifi is connected
static void wifi_connected_cb(void)
{
    system_status.wifi_status = 1;
    strcpy(WiFi_sub_1.value, "CONNECTED");

    // Update IP info
    esp_netif_ip_info_t ip_info = wifi_get_ip_info();

    ipv4_address_to_string(ip_info.ip.addr, WiFi_sub_5.value);
    ipv4_address_to_string(ip_info.gw.addr, WiFi_sub_7.value);
    ipv4_address_to_string(ip_info.netmask.addr, WiFi_sub_6.value);

    // Update Authmode
    /* Authmodes:
        0: open
        1: WEP
        2: WPA_PSK
        3: WPA2_PSK
        4: WPA_WPA2_PSK
        5: WiFi EAP security
        6: WPA3_PSK
        7: WPA2_WPA3_PSK
        8: WAPI_PSK
        9: OWE
        10: WPA3_ENT_SUITE_B_192_BIT
        11: WPA3_PSK_EXT_KEY
        12: WPA3_PSK + WPA3_PSK_EXT_KEY
        13: max
    */
    wifi_ap_record_t ap;
    esp_wifi_sta_get_ap_info(&ap);
    sprintf(WiFi_sub_4.value, "%d", ap.authmode);

    xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_WIFI_STATUS_CHANGE | REFRESH_EVENT_FORCE_REFRESH, eSetBits);
}

// Callback function when wifi is disconnected
static void wifi_disconnected_cb(void)
{
    system_status.wifi_status = 0;
    strcpy(WiFi_sub_1.value, "D/C");
    strcpy(WiFi_sub_3.value, "N/A");
    strcpy(WiFi_sub_4.value, "N/A");
    strcpy(WiFi_sub_5.value, "N/A");
    strcpy(WiFi_sub_6.value, "N/A");
    strcpy(WiFi_sub_7.value, "N/A");
    xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_WIFI_STATUS_CHANGE | REFRESH_EVENT_FORCE_REFRESH, eSetBits);
}

// Function to toggle the wifi connection status i.e. connect to wifi if currently unconnected and vise versa
static void wifi_connection_control(void)
{
    if (system_status.wifi_status == 1)
    {
        wifi_disconnect();
    }
    else
    {
        wifi_connect_sta(system_status.wifi_ssid, system_status.wifi_password, WIFI_CONNECT_TIMEOUT_MS);
    }
}

// Function to convert ip address from uint32_t to string
static void ipv4_address_to_string(uint32_t addr, char *addr_string)
{
    sprintf(addr_string, "%ld.%ld.%ld.%ld", addr & 0xFF, (addr >> 8) & 0xFF, (addr >> 16) & 0xFF, (addr >> 24) & 0xFF);
}

// Task to periodically update Wi-Fi RSSI when connected
static void refresh_WiFi_RSSI(void *params)
{
    while (true)
    {
        // Update when connected
        if (system_status.wifi_status)
        {
            wifi_ap_record_t ap;
            esp_wifi_sta_get_ap_info(&ap);
            if (ap.rssi != system_status.wifi_RSSI)
            {
                sprintf(WiFi_sub_3.value, "%d dBm", ap.rssi);
                xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_WIFI_RSSI_CHANGE, eSetBits);
            }

            // Determine wifi quality
            uint8_t updated_wifi_quality;
            if (ap.rssi >= -60)
            {
                updated_wifi_quality = 3;
            }
            else if (ap.rssi >= -70)
            {
                updated_wifi_quality = 2;
            }
            else
            {
                updated_wifi_quality = 1;
            }

            // if quality changed, update the wifi icon
            if (updated_wifi_quality != wifi_quality)
            {
                xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_FORCE_REFRESH, eSetBits);
            }

            wifi_quality = updated_wifi_quality;
            system_status.wifi_RSSI = ap.rssi;
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

// ------------------------------------------ Functions for RTC --------------------------------------------
// Task to periodically update time information on screen
static void refresh_time(void *params)
{
    struct tm prev_timeinfo = time_client_get_time();
    struct tm timeinfo;
    while (true)
    {
        // Update time and date
        timeinfo = time_client_get_time();
        if ((timeinfo.tm_sec != prev_timeinfo.tm_sec) | (timeinfo.tm_min != prev_timeinfo.tm_min) | (timeinfo.tm_hour != prev_timeinfo.tm_hour))
        {
            sprintf(rtc_sub_2.value, "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
            sprintf(rtc_sub_3.value, "%02d/%02d/%d", timeinfo.tm_mday, 1 + (uint8_t)timeinfo.tm_mon, 1900 + (uint16_t)timeinfo.tm_year);
            xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_TIME_CHANGE | REFRESH_EVENT_DATE_CHANGE, eSetBits);
        }
        prev_timeinfo = timeinfo;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Function to toggle the autosync status of rtc
static void toggle_rtc_autoSync(void)
{
    if (esp_sntp_enabled())
    {
        time_client_deinit();
        system_status.sntp_status = 0;
        strcpy(rtc_sub_4.value, "OFF");
        ESP_LOGI("TIME CLIENT", "RTC AutoSync Disabled");
    }
    else
    {
        time_client_init();
        system_status.sntp_status = 1;
        strcpy(rtc_sub_4.value, "ON");
        ESP_LOGI("TIME CLIENT", "RTC AutoSync Enabled");
    }
    xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_AUTOSYNC_STATUS_CHANGE, eSetBits);
}

// Function to disable autosync of rtc
static void disable_rtc_autoSync(void)
{
    if (esp_sntp_enabled())
    {
        time_client_deinit();
        system_status.sntp_status = 0;
        strcpy(rtc_sub_4.value, "OFF");
        ESP_LOGI("TIME CLIENT", "RTC AutoSync Disabled");
    }
    xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_AUTOSYNC_STATUS_CHANGE, eSetBits);
}

// Function to enable autosync of rtc
static void enable_rtc_autoSync(void)
{
    if (!esp_sntp_enabled())
    {
        time_client_init();
        system_status.sntp_status = 1;
        strcpy(rtc_sub_4.value, "ON");
        ESP_LOGI("TIME CLIENT", "RTC AutoSync Enabled");
    }
    xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_AUTOSYNC_STATUS_CHANGE, eSetBits);
}

// ------------------------------------------ Functions for IMU Syncing --------------------------------------------
// Estimate synchronising signal duration
static void calculate_sync_signal_duration(void)
{
    // Signal duration is calculated using the formula provided in the paper
    system_status.sync_signal_duration = system_status.target_imu_sample_period * (system_status.sync_signal_pulse_width + system_status.maximum_sync_error) / system_status.maximum_sync_error + system_status.sync_signal_pulse_width;
}

// Calculate number of pulses needed for the synchronisation
static uint8_t calculate_number_of_pulses(void)
{
    // Some modifications are made here compared to the theoretical value, mainly used to ensure the signal ends at logic low
    number_of_pulses = system_status.target_imu_sample_period / system_status.maximum_sync_error + 2;
    if (number_of_pulses % 2 == 0)
        number_of_pulses++;

    return number_of_pulses;
}

// Function to start / stop the current synchronising group
static void start_stop_sync_group(void)
{
    if (system_status.is_syncing == 0)
    {
        struct tm sync_group_start_time = time_client_get_time();
        // Event group can only start if the system time seems to be correct and sd card connected
        if (sync_group_start_time.tm_year >= 2024 - 1900)
        {
            if (system_status.sd_connected == 1)
            {
                // Create new .csv file to record synchronising data
                if (create_file_sync_data(sync_group_start_time) == ESP_FAIL)
                {
                    return;
                }

                // Enable and disable some functions and services
                FM_sub_2.function_ptr = &sync_imu_now; // Only enable "SYNC NOW" option when a sync group is initialised
                rtc_sub_4.function_ptr = NULL;         // Disable rtc_autoSync setting
                disable_rtc_autoSync();                // Turn off RTC autoSync

                system_status.sync_count = 1;                               // Clear sync count
                sprintf(FM_sub_2.value, "No.%d", system_status.sync_count); // Display value No.1 on screen
                xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_SYNC_COUNT_CHANGE, eSetBits);

                strcpy(FM_sub_1.name, " STOP SYNCING ");
                system_status.is_syncing = 1;
                ESP_LOGI("SYNC GROUP", "New Synchronising group created, ready to log synchronising data");
            }
            else
            {
                ESP_LOGW("SYNC GROUP", "SD card not connected. Please connect SD card and retry later.");
            }
        }
        else
        {
            ESP_LOGW("SYNC GROUP", "System time is not synchronised. Please connect Wi-Fi first and retry later.");
        }
    }
    else
    {
        // Enable and disable some functions and services
        FM_sub_2.function_ptr = NULL;                                       // Disable "SYNC NOW" option when a sync group is ended
        if (!(system_status.is_syncing & system_status.onboard_imu_status)) // autosync will not be enabled if both onboard imu and sync group is runnig
        {
            rtc_sub_4.function_ptr = &toggle_rtc_autoSync; // re-enable rtc_autoSync setting
            enable_rtc_autoSync();                         // Turn on RTC autoSync
        }

        strcpy(FM_sub_2.value, ""); // Remove No. on screen

        // Update SD capacity information
        // if the sd capacity info returned is valid:
        sd_capacity_info = get_sd_capacity_info();
        if (sd_capacity_info.result_available)
        {
            sprintf(sd_sub_2.value, "%.2f/%.2fGB", sd_capacity_info.used_GB, sd_capacity_info.total_GB);
        }

        xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_SYNC_COUNT_CHANGE | REFRESH_EVENT_SD_CAPACITY_INFO_CHANGE, eSetBits);

        strcpy(FM_sub_1.name, " START SYNC ");
        system_status.is_syncing = 0;
        ESP_LOGI("SYNC GROUP", "Synchronising group terminated");
    }
    xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_START_STOP_SYNCING, eSetBits);
}

// Function to send synchronising signal to the target imu and record UNIX timestamp
static void sync_imu_now(void)
{
    FM_sub_2.function_ptr = NULL;
    system_status.sending_sync_signal = 1;
    // Calculate and Initialise sync parameters
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = (system_status.sync_signal_pulse_width + system_status.maximum_sync_error) * 1000, // w + a
        .flags.auto_reload_on_alarm = true};
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer, system_status.maximum_sync_error * 1000)); // Set starting value to a
    EM_gpio_status = !EM_gpio_status;
    uint8_t pulse_number = calculate_number_of_pulses(); // Refresh number of pulse

    // Notify user the sync event
    gpio_set_level(PIN_LED_B, 1); // Turn on LED to indicate that the sync is started
    strcpy(FM_sub_2.name, " Syncing... ");
    xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_SYNC_COUNT_CHANGE, eSetBits);
    ESP_LOGI("IMU SYNC", "Start Sync, Sync Count = %d, Number of pulses = %d", system_status.sync_count, number_of_pulses);

    // Start
    gpio_set_level(PIN_EM, EM_gpio_status);
    gptimer_start(gptimer); // Start timer

    int64_t UNIX_timestamp_ms = time_client_get_UNIX_timestamp_ms(); // Create UNIX Timestamp
    ESP_LOGI("IMU SYNC", "Sync No. %d, UNIX Time Stamp: %" PRId64, system_status.sync_count, UNIX_timestamp_ms);
    log_sync_data(UNIX_timestamp_ms, system_status.sync_count, system_status.poi, system_status.maximum_sync_error, pulse_number);
}

// Function to create a new .csv file for the data of IMU synchronisations
static esp_err_t create_file_sync_data(struct tm current_time)
{
    // Generate filename
    sprintf(file_syncData_path, "/sdcard/SYNC_%d-%02d-%02d_%02d-%02d-%02d.csv", 1900 + (uint16_t)current_time.tm_year, 1 + (uint8_t)current_time.tm_mon,
            current_time.tm_mday, current_time.tm_hour, current_time.tm_min, current_time.tm_sec);

    FILE *fptr = fopen(file_syncData_path, "w");
    if (fptr == NULL)
    {
        ESP_LOGE("FATFS", "Failed to create file %s", file_syncData_path);
        return ESP_FAIL;
    }

    // Write first row
    fprintf(fptr, "Timestamp,Sync Number,POI,Max Error,Pulse Number\n");

    fclose(fptr);
    ESP_LOGI("FATFS", "File %s created", file_syncData_path);
    return ESP_OK;
}

// Function to log data of the current IMU synchronisation into SD card
static void log_sync_data(int64_t timestamp, uint16_t sync_num, uint16_t point_of_interest, float max_error, uint8_t pulse_num)
{
    FILE *fptr = fopen(file_syncData_path, "a");

    // Check if file is successfully openend
    if (fptr == NULL)
    {
        ESP_LOGE("FATFS", "Failed to log Sync No. %d. Reason: Cannot open file: %s", sync_num, file_syncData_path);
        return;
    }

    // Timestamp, Sync Number, POI, Max Error, Pulse Number
    if (sync_num != point_of_interest)
    {
        fprintf(fptr, "%" PRId64 ",%d,N,%.2f,%d\n", timestamp, sync_num, max_error, pulse_num); // Is point of interest
    }
    else
    {
        fprintf(fptr, "%" PRId64 ",%d,Y,%.2f,%d\n", timestamp, sync_num, max_error, pulse_num); // Not point of interest
    }

    fclose(fptr);
}

// ------------------------------------------ Functions for on-board IMU (To be implemented) --------------------------------------------
// Function to configure on-board IMU
static esp_err_t icm20948_configure(icm20948_acce_fs_t acce_fs, icm20948_gyro_fs_t gyro_fs)
{
    esp_err_t ret;

    /*
     * One might need to change ICM20948_I2C_ADDRESS to ICM20948_I2C_ADDRESS_1
     * if address pin pulled low (to GND)
     */
    icm20948 = icm20948_create(I2C_MASTER_NUM, ICM20948_I2C_ADDRESS_1);
    if (icm20948 == NULL)
    {
        ESP_LOGE("On-board IMU", "ICM20948 create returned NULL!");
        return ESP_FAIL;
    }
    ESP_LOGI("On-board IMU", "ICM20948 creation successfull!");

    ret = icm20948_reset(icm20948);
    if (ret != ESP_OK)
    {
        return ret;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);

    ret = icm20948_wake_up(icm20948);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = icm20948_set_bank(icm20948, 0);
    if (ret != ESP_OK)
    {
        return ret;
    }

    uint8_t device_id;
    ret = icm20948_get_deviceid(icm20948, &device_id);
    if (ret != ESP_OK)
    {
        return ret;
    }
    ESP_LOGI("On-board IMU", "0x%02X", device_id);
    if (device_id != ICM20948_WHO_AM_I_VAL)
    {
        return ESP_FAIL;
    }

    ret = icm20948_set_gyro_fs(icm20948, gyro_fs);
    if (ret != ESP_OK)
    {
        return ESP_FAIL;
    }

    ret = icm20948_set_acce_fs(icm20948, acce_fs);
    if (ret != ESP_OK)
    {
        return ESP_FAIL;
    }

    return ret;
}

// Function to log IMU Data
static void log_IMU_data(icm20948_acce_value_t acce, icm20948_gyro_value_t gyro)
{
    int64_t timestamp = time_client_get_UNIX_timestamp_ms();
    fprintf(fptr_imu, "%" PRId64 ",%f,%f,%f,%f,%f,%f\n", timestamp, acce.acce_x, acce.acce_y, acce.acce_z, gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
}

// Function to read IMU Data
void icm_read_task(void *params)
{
    icm20948_acce_value_t acce;
    icm20948_gyro_value_t gyro;
    while (true)
    {
        if (system_status.onboard_imu_status)
        {
            ESP_ERROR_CHECK(icm20948_get_acce(icm20948, &acce));
            ESP_ERROR_CHECK(icm20948_get_gyro(icm20948, &gyro));
            log_IMU_data(acce, gyro);

            vTaskDelay(pdMS_TO_TICKS(DEFAULT_ONBOARD_IMU_SAMPLE_PERIOD));
        }
        else
        {
            vTaskSuspend(NULL);
        }
    }
}

// Function to toggle the on board imu to do sampling
static void toggle_on_board_imu(void)
{
    if (system_status.onboard_imu_status == 0)
    {
        struct tm sync_group_start_time = time_client_get_time();

        // On Board IMU can only start if the system time seems to be correct and sd card connected
        if (sync_group_start_time.tm_year >= 2024 - 1900)
        {
            if (system_status.sd_connected == 1)
            {
                // Create new .csv file to record imu data
                if (create_file_onboard_IMU_data(sync_group_start_time) == ESP_FAIL)
                {
                    return;
                }

                // Disable rtc autosync
                rtc_sub_4.function_ptr = NULL;
                disable_rtc_autoSync();

                // Wake up IMU
                ESP_ERROR_CHECK(icm20948_wake_up(icm20948));

                strcpy(FM_sub_8.value, "ON");
                system_status.onboard_imu_status = 1;
                vTaskResume(imu_read_suspend_handler);
                ESP_LOGI("On_board IMU", "Data logging started");
            }
            else
            {
                ESP_LOGW("On_board IMU", "SD card not connected. Please connect SD card and retry later.");
            }
        }
        else
        {
            ESP_LOGW("On_board IMU", "System time is not synchronised. Please connect Wi-Fi first and retry later.");
        }
    }
    else
    {
        if (!(system_status.is_syncing & system_status.onboard_imu_status)) // autosync will not be enabled if both onboard imu and sync group is runnig
        {
            rtc_sub_4.function_ptr = &toggle_rtc_autoSync; // re-enable rtc_autoSync setting
            enable_rtc_autoSync();                         // Turn on RTC autoSync
        }

        strcpy(FM_sub_8.value, "OFF");
        system_status.onboard_imu_status = 0;
        fclose(fptr_imu);
        fptr_imu = NULL;
        ESP_ERROR_CHECK(icm20948_sleep(icm20948));

        ESP_LOGI("On_board IMU", "Data logging terminated");

        // Update SD capacity information
        // if the sd capacity info returned is valid:
        sd_capacity_info = get_sd_capacity_info();
        if (sd_capacity_info.result_available)
        {
            sprintf(sd_sub_2.value, "%.2f/%.2fGB", sd_capacity_info.used_GB, sd_capacity_info.total_GB);
            xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_SD_CAPACITY_INFO_CHANGE, eSetBits);
        }
    }
    xTaskNotify(screen_refresh_notification_handler, REFRESH_EVENT_ONBOARD_IMU_STATUS_CHANGE, eSetBits);
}

// Function to create a new .csv file for the data of IMU synchronisations
static esp_err_t create_file_onboard_IMU_data(struct tm current_time)
{
    // Generate filename
    sprintf(file_IMUData_path, "/sdcard/IMU_%d-%02d-%02d_%02d-%02d-%02d.csv", 1900 + (uint16_t)current_time.tm_year, 1 + (uint8_t)current_time.tm_mon,
            current_time.tm_mday, current_time.tm_hour, current_time.tm_min, current_time.tm_sec);

    fptr_imu = fopen(file_IMUData_path, "a");
    if (fptr_imu == NULL)
    {
        ESP_LOGE("FATFS", "Failed to create file %s", file_IMUData_path);
        return ESP_FAIL;
    }

    // Write first row
    fprintf(fptr_imu, "Timestamp,ax,ay,az,gx,gy,gz\n");

    ESP_LOGI("FATFS", "File %s created", file_IMUData_path);
    return ESP_OK;
}

// ------------------------------------------ Functions for Screen (Menus, Refresh Control, etc.) --------------------------------------------
// Initialise the menu
static void menu_init(void)
{
    // Front Menu and Submenu Tabs
    FM_sub_1.next_submenu_ptr = &FM_sub_2;
    FM_sub_2.next_submenu_ptr = &FM_sub_3;
    FM_sub_3.next_submenu_ptr = &FM_sub_4;
    FM_sub_4.next_submenu_ptr = &FM_sub_5;
    FM_sub_5.next_submenu_ptr = &FM_sub_6;
    FM_sub_6.next_submenu_ptr = &FM_sub_7;
    FM_sub_7.next_submenu_ptr = &FM_sub_8;
    FM_sub_8.next_submenu_ptr = &FM_sub_9;
    FM_sub_9.next_submenu_ptr = NULL;

    front_menu.pageScroller_length = calculate_pageScroller_length(&front_menu);

    // Wifi Menu and Submenu Tabs
    WiFi_sub_1.next_submenu_ptr = &WiFi_sub_2;
    WiFi_sub_2.next_submenu_ptr = &WiFi_sub_3;
    WiFi_sub_3.next_submenu_ptr = &WiFi_sub_4;
    WiFi_sub_4.next_submenu_ptr = &WiFi_sub_5;
    WiFi_sub_5.next_submenu_ptr = &WiFi_sub_6;
    WiFi_sub_6.next_submenu_ptr = &WiFi_sub_7;
    WiFi_sub_7.next_submenu_ptr = NULL;

    WiFi_menu.pageScroller_length = calculate_pageScroller_length(&WiFi_menu);

    // RTC Menu and Submenu Tabs
    rtc_sub_1.next_submenu_ptr = &rtc_sub_2;
    rtc_sub_2.next_submenu_ptr = &rtc_sub_3;
    rtc_sub_3.next_submenu_ptr = &rtc_sub_4;
    rtc_sub_4.next_submenu_ptr = &rtc_sub_5;
    rtc_sub_5.next_submenu_ptr = NULL;

    rtc_menu.pageScroller_length = calculate_pageScroller_length(&rtc_menu);

    // SD Card Menu and Submenu Tabs
    sd_sub_1.next_submenu_ptr = &sd_sub_2;
    sd_sub_2.next_submenu_ptr = &sd_sub_3;
    sd_sub_3.next_submenu_ptr = NULL;

    sd_menu.pageScroller_length = calculate_pageScroller_length(&sd_menu);

    // Battery Menu and Submenu Tabs
    battery_sub_1.next_submenu_ptr = &battery_sub_2;
    battery_sub_2.next_submenu_ptr = &battery_sub_3;
    battery_sub_3.next_submenu_ptr = NULL;

    battery_menu.pageScroller_length = calculate_pageScroller_length(&battery_menu);

    // Link Together Menus
    front_menu.next_menu = &WiFi_menu;
    WiFi_menu.next_menu = &rtc_menu;
    rtc_menu.next_menu = &sd_menu;
    sd_menu.next_menu = &battery_menu;
    battery_menu.next_menu = &front_menu;

    current_menu = &front_menu;
    current_submenu = front_menu.submenu_ptr;
    sub1_disp = current_menu->submenu_ptr;
    sub2_disp = sub1_disp != NULL ? sub1_disp->next_submenu_ptr : NULL;
    sub3_disp = sub2_disp != NULL ? sub2_disp->next_submenu_ptr : NULL;

    // Fill in some initial values
    sprintf(FM_sub_3.value, "%d", system_status.poi);
    strcpy(rtc_sub_1.value, system_status.POSIX_tz);
    strcpy(WiFi_sub_2.value, system_status.wifi_ssid);
    sprintf(FM_sub_4.value, "%.1fms", system_status.maximum_sync_error);
    sprintf(FM_sub_5.value, "%.1fms", system_status.target_imu_sample_period);
    sprintf(FM_sub_6.value, "%.1fms", system_status.sync_signal_pulse_width);
    calculate_sync_signal_duration();
    sprintf(FM_sub_7.value, "%.3fs", system_status.sync_signal_duration / 1000);
    sprintf(FM_sub_9.value, "%dms", system_status.onboard_imu_sample_period);
}

// Task to refresh the screen when notification is received
static void screen_refresh(void *params)
{
    uint32_t state; // Indicating which type of change triggered the refresh event

    u8g2_SetFont(&u8g2, FONT_MENU);
    u8g2_DrawStr(&u8g2, 0, MENU_Y, current_menu->name);
    u8g2_DrawHLine(&u8g2, 0, HLINE_Y, SCREEN_WIDTH);
    u8g2_SetFont(&u8g2, FONT_SUBMENU);
    u8g2_DrawStr(&u8g2, CURSOR_INDENT, SUB1_Y, ">");
    if (sub1_disp != NULL)
    {
        u8g2_SetFont(&u8g2, FONT_SUBMENU);
        u8g2_SetDrawColor(&u8g2, sub1_disp->function_ptr != NULL ? 0 : 1);
        u8g2_DrawStr(&u8g2, SUBMENU_INDENT, SUB1_Y, sub1_disp->name);
        u8g2_SetDrawColor(&u8g2, 1);
        u8g2_SetFont(&u8g2, FONT_VALUE);
        u8g2_DrawStr(&u8g2, SCREEN_WIDTH - VALUE_INDENT - u8g2_GetStrWidth(&u8g2, sub1_disp->value), SUB1_Y, sub1_disp->value);
    }
    if (sub2_disp != NULL)
    {
        u8g2_SetFont(&u8g2, FONT_SUBMENU);
        u8g2_DrawStr(&u8g2, SUBMENU_INDENT, SUB2_Y, sub2_disp->name);
        u8g2_SetFont(&u8g2, FONT_VALUE);
        u8g2_DrawStr(&u8g2, SCREEN_WIDTH - VALUE_INDENT - u8g2_GetStrWidth(&u8g2, sub2_disp->value), SUB2_Y, sub2_disp->value);
    }
    if (sub3_disp != NULL)
    {
        u8g2_SetFont(&u8g2, FONT_SUBMENU);
        u8g2_DrawStr(&u8g2, SUBMENU_INDENT, SUB3_Y, sub3_disp->name);
        u8g2_SetFont(&u8g2, FONT_VALUE);
        u8g2_DrawStr(&u8g2, SCREEN_WIDTH - VALUE_INDENT - u8g2_GetStrWidth(&u8g2, sub3_disp->value), SUB3_Y, sub3_disp->value);
    }
    u8g2_DrawHLine(&u8g2, SCREEN_WIDTH - 2, HLINE_Y + 2, 2);
    u8g2_DrawHLine(&u8g2, SCREEN_WIDTH - 2, SCREEN_HEIGHT - 1, 2);
    u8g2_DrawVLine(&u8g2, SCREEN_WIDTH - 1, pageScroller_Y, current_menu->pageScroller_length);
    u8g2_DrawVLine(&u8g2, SCREEN_WIDTH - 2, pageScroller_Y, current_menu->pageScroller_length);

    u8g2_SendBuffer(&u8g2);

    while (true)
    {
        xTaskNotifyWait(0xFFFFFFFF, 0, &state, portMAX_DELAY);
        if ((state & ((sub1_disp != NULL ? sub1_disp->key : 0) | (sub2_disp != NULL ? sub2_disp->key : 0) | (sub3_disp != NULL ? sub3_disp->key : 0))) != 0)
        {
            u8g2_ClearBuffer(&u8g2);
            u8g2_SetFont(&u8g2, FONT_MENU);
            u8g2_DrawStr(&u8g2, 0, MENU_Y, current_menu->name); // Draw menu
            u8g2_DrawHLine(&u8g2, 0, HLINE_Y, 128);             // Draw seperation line

            // Draw Submenu 1
            if (sub1_disp != NULL)
            {
                u8g2_SetFont(&u8g2, FONT_SUBMENU);
                if (sub1_disp == current_submenu)
                {
                    u8g2_DrawStr(&u8g2, CURSOR_INDENT, SUB1_Y, ">");
                    u8g2_SetDrawColor(&u8g2, sub1_disp->function_ptr != NULL ? 0 : 1);
                    u8g2_DrawStr(&u8g2, SUBMENU_INDENT, SUB1_Y, sub1_disp->name);
                    u8g2_SetDrawColor(&u8g2, 1);
                }
                else
                {
                    u8g2_DrawStr(&u8g2, SUBMENU_INDENT, SUB1_Y, sub1_disp->name);
                }
                u8g2_SetFont(&u8g2, FONT_VALUE);
                u8g2_DrawStr(&u8g2, SCREEN_WIDTH - VALUE_INDENT - u8g2_GetStrWidth(&u8g2, sub1_disp->value), SUB1_Y, sub1_disp->value);
            }

            // Draw Submenu 2
            if (sub2_disp != NULL)
            {
                u8g2_SetFont(&u8g2, FONT_SUBMENU);
                if (sub2_disp == current_submenu)
                {
                    u8g2_DrawStr(&u8g2, CURSOR_INDENT, SUB2_Y, ">");
                    u8g2_SetDrawColor(&u8g2, sub2_disp->function_ptr != NULL ? 0 : 1);
                    u8g2_DrawStr(&u8g2, SUBMENU_INDENT, SUB2_Y, sub2_disp->name);
                    u8g2_SetDrawColor(&u8g2, 1);
                }
                else
                {
                    u8g2_DrawStr(&u8g2, SUBMENU_INDENT, SUB2_Y, sub2_disp->name);
                }
                u8g2_SetFont(&u8g2, FONT_VALUE);
                u8g2_DrawStr(&u8g2, SCREEN_WIDTH - VALUE_INDENT - u8g2_GetStrWidth(&u8g2, sub2_disp->value), SUB2_Y, sub2_disp->value);
            }

            // Draw Submenu 3
            if (sub3_disp != NULL)
            {
                u8g2_SetFont(&u8g2, FONT_SUBMENU);
                if (sub3_disp == current_submenu)
                {
                    u8g2_DrawStr(&u8g2, CURSOR_INDENT, SUB3_Y, ">");
                    u8g2_SetDrawColor(&u8g2, sub3_disp->function_ptr != NULL ? 0 : 1);
                    u8g2_DrawStr(&u8g2, SUBMENU_INDENT, SUB3_Y, sub3_disp->name);
                    u8g2_SetDrawColor(&u8g2, 1);
                }
                else
                {
                    u8g2_DrawStr(&u8g2, SUBMENU_INDENT, SUB3_Y, sub3_disp->name);
                }
                u8g2_SetFont(&u8g2, FONT_VALUE);
                u8g2_DrawStr(&u8g2, SCREEN_WIDTH - VALUE_INDENT - u8g2_GetStrWidth(&u8g2, sub3_disp->value), SUB3_Y, sub3_disp->value);
            }

            // Draw Page Scroller
            u8g2_DrawHLine(&u8g2, SCREEN_WIDTH - 2, HLINE_Y + 2, 2);
            u8g2_DrawHLine(&u8g2, SCREEN_WIDTH - 2, SCREEN_HEIGHT - 1, 2);
            u8g2_DrawVLine(&u8g2, SCREEN_WIDTH - 1, pageScroller_Y, current_menu->pageScroller_length);
            u8g2_DrawVLine(&u8g2, SCREEN_WIDTH - 2, pageScroller_Y, current_menu->pageScroller_length);

            // Draw Icons
            u8g2_SetFont(&u8g2, STATUS_ICONS_font);
            if (system_status.charging_status == 2)
            {
                u8g2_DrawGlyph(&u8g2, SCREEN_WIDTH - 8, MENU_Y, ICON_BATTERY_CHG);
            }
            else
            {
                u8g2_DrawGlyph(&u8g2, SCREEN_WIDTH - 8, MENU_Y, ICON_BATTERY_CHG - system_status.battery_estimated_soc);
            }

            if (system_status.wifi_status)
            {
                switch (wifi_quality)
                {
                case 3:
                    u8g2_DrawGlyph(&u8g2, SCREEN_WIDTH - 21, MENU_Y, ICON_WIFI_CONNECTED_GOOD);
                    break;
                case 2:
                    u8g2_DrawGlyph(&u8g2, SCREEN_WIDTH - 21, MENU_Y, ICON_WIFI_CONNECTED_WEAK);
                    break;
                case 1:
                    u8g2_DrawGlyph(&u8g2, SCREEN_WIDTH - 21, MENU_Y, ICON_WIFI_CONNECTED_POOR);
                    break;
                }
            }

            if (system_status.BLE_status)
            {
                u8g2_DrawGlyph(&u8g2, SCREEN_WIDTH - 34, MENU_Y, ICON_BT_CONNECTED);
            }

            if (system_status.sd_connected)
            {
                u8g2_SetFont(&u8g2, u8g2_font_4x6_mr);
                u8g2_DrawStr(&u8g2, SCREEN_WIDTH - 45, MENU_Y, "SD");
            }

            u8g2_SendBuffer(&u8g2);
            ESP_LOGI("Screen", "Screen Refreshed, state = %lu, key = %lu", state, ((sub1_disp != NULL ? sub1_disp->key : 0) | (sub2_disp != NULL ? sub2_disp->key : 0) | (sub3_disp != NULL ? sub3_disp->key : 0)));
        }

        // Pause task when device is going to sleep
        while (sleep_flag)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

// Function returns the number of total submenu tabs under the menu. Input a submenu_ptr to use.
static uint8_t get_number_of_submenus(submenu_t *submenu)
{
    if (submenu == NULL)
    {
        return 0;
    }
    else
    {
        return 1 + get_number_of_submenus(submenu->next_submenu_ptr);
    }
}

// Function to calculate how long (pixels) the page scroller should be
static uint8_t calculate_pageScroller_length(menu_t *menu)
{
    return (SCREEN_HEIGHT - HLINE_Y - 6) / (get_number_of_submenus(menu->submenu_ptr));
}
