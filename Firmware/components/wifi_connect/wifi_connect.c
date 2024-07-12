#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

static esp_netif_t *esp_netif;
static char *TAG = "WIFI CONNECT";
static EventGroupHandle_t wifi_events;
static int CONNECTED = BIT0;
static int DISCONNECTED = BIT1;
static bool attempt_reconnect = false;
static const int max_disconnection_retry_count = 3;
static const int retry_interval = 5000;
static void (*wifi_connected_cb)(void);
static void (*wifi_disconnected_cb)(void);
static bool esp_netif_destroyed = 1;

char *get_wifi_disconnection_string(wifi_err_reason_t wifi_err_reason); // Get disconnected error reason

int disconnection_err_count = 0;

// Disconnect WiFi
void wifi_disconnect(void)
{
    attempt_reconnect = false; // Avoid trying to reconnect
    ESP_ERROR_CHECK(esp_wifi_stop());
    esp_netif_destroy(esp_netif);
    esp_netif_destroyed = 1;
}

// Event Handler
void event_handler(void *event_handler_arg, esp_event_base_t event_base,
                   int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        ESP_LOGI(TAG, "WIFI_EVENT_STA_START");
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;
    case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGI(TAG, "WIFI_EVENT_STA_CONNECTED");
        disconnection_err_count = 0;
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
    {
        wifi_event_sta_disconnected_t *wifi_event_sta_disconnected = event_data;
        ESP_LOGW(TAG, "DISCONNECTED %d: %s", wifi_event_sta_disconnected->reason, get_wifi_disconnection_string(wifi_event_sta_disconnected->reason));
        xEventGroupSetBits(wifi_events, DISCONNECTED);
        wifi_disconnected_cb();
        // Attempt to reconnect if WiFi disconnected temporarily
        if (attempt_reconnect)
        {
            if (wifi_event_sta_disconnected->reason == WIFI_REASON_NO_AP_FOUND ||
                wifi_event_sta_disconnected->reason == WIFI_REASON_ASSOC_LEAVE ||
                wifi_event_sta_disconnected->reason == WIFI_REASON_AUTH_EXPIRE)
            {
                if (disconnection_err_count++ < max_disconnection_retry_count)
                {
                    vTaskDelay(pdMS_TO_TICKS(retry_interval)); // Time wait to retry connection
                    ESP_LOGI(TAG, "Retry Connection (%d/%d)", disconnection_err_count, max_disconnection_retry_count);
                    ESP_ERROR_CHECK(esp_wifi_connect());
                    break;
                }
            }
        }
        break;
    }
    case IP_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "IP_EVENT_STA_GOT_IP");
        xEventGroupSetBits(wifi_events, CONNECTED);
        wifi_connected_cb();
        break;

        // Add case here to handle AP events if needed

    default:
        break;
    }
}

void wifi_connect_init(void (*wifi_connected_callBack)(void), void (*wifi_disconnected_callBack)(void))
{
    wifi_connected_cb = wifi_connected_callBack;
    wifi_disconnected_cb = wifi_disconnected_callBack;
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL));  // All WIFI_EVENT
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event_handler, NULL)); // IP_EVENT_STA_GOT_IP
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
}

// Connect as station
esp_err_t wifi_connect_sta(char *ssid, char *pass, int timeout)
{
    attempt_reconnect = false;
    wifi_events = xEventGroupCreate();
    if (esp_netif_destroyed == 1)
    {
        esp_netif = esp_netif_create_default_wifi_sta();
        esp_netif_destroyed = 0;
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

        // Configure WiFi
        wifi_config_t wifi_config = {};
        strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
        strncpy((char *)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password) - 1);
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start()); // Start WiFi
    }
    else
    {
        esp_wifi_connect();
    }

    EventBits_t result = xEventGroupWaitBits(wifi_events, CONNECTED | DISCONNECTED, true, false, pdMS_TO_TICKS(timeout)); // Blocking, waiting for connection status
    if (result == CONNECTED)
    {
        disconnection_err_count = 0;
        attempt_reconnect = true;
        return ESP_OK;
    }
    wifi_disconnect();
    return ESP_FAIL;
}

// Connect as AP
void wifi_connect_ap(const char *ssid, const char *pass)
{
    esp_netif = esp_netif_create_default_wifi_ap();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    // Configure WiFi
    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid) - 1);
    strncpy((char *)wifi_config.ap.password, pass, sizeof(wifi_config.ap.password) - 1);
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_WPA3_PSK;
    wifi_config.ap.max_connection = 8;
    wifi_config.ap.beacon_interval = 100;
    wifi_config.ap.channel = 1;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start()); // Start WiFi
}

// Get ip info
esp_netif_ip_info_t wifi_get_ip_info(void)
{
    esp_netif_ip_info_t ip_info;
    ESP_ERROR_CHECK(esp_netif_get_ip_info(esp_netif, &ip_info));
    return ip_info;
}
