#include <stdio.h>
#include <time.h>
#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"

#ifndef DEFAULT_SNTP_TIME_SERVER
#define DEFAULT_SNTP_TIME_SERVER "pool.ntp.org"
#endif

#ifndef INET6_ADDRSTRLEN
#define INET6_ADDRSTRLEN 48
#endif

static const char *TAG = "TIME CLIENT";
char strftime_buf[64];

// Print SNTP servers
static void print_servers(void)
{
    if (esp_sntp_getservername(0))
    {
        ESP_LOGI(TAG, "Configured NTP server: %s", esp_sntp_getservername(0));
    }
    else
    {
        // we have either IPv4 or IPv6 address, let's print it
        char buff[INET6_ADDRSTRLEN];
        ip_addr_t const *ip = esp_sntp_getserver(0);
        if (ipaddr_ntoa_r(ip, buff, INET6_ADDRSTRLEN) != NULL)
            ESP_LOGI(TAG, "Configured NTP server: %s", buff);
    }
}

// Time synchronization event callback
void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

/* Initialise SNTP, automatic sync interval can be configured in
 * project config by adjusting CONFIG_LWIP_SNTP_UPDATE_DELAY
 */
void time_client_init(void)
{
    ESP_LOGI(TAG, "Initializing and starting SNTP");
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(DEFAULT_SNTP_TIME_SERVER); // Config only one SNTP server
    config.sync_cb = time_sync_notification_cb;                                         // Callback function for time sync event
    ESP_ERROR_CHECK(esp_netif_sntp_init(&config));
    print_servers();
}

// Stop SNTP
void time_client_deinit(void)
{
    ESP_LOGI(TAG, "Stopping SNTP");
    esp_netif_sntp_deinit();
}

// Set timezone
void time_client_set_tz(char *POSIX_tz)
{
    setenv("TZ", POSIX_tz, 1);
    tzset();
}

// Get current time with one second resolution in string form
char *time_client_get_time_str(void)
{
    time_t now;
    struct tm timeinfo;

    time(&now);

    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    // ESP_LOGI(TAG, "The current date/time in XXX is: %s", strftime_buf);
    return strftime_buf;
}

// Get current time in tm struct form
struct tm time_client_get_time(void)
{
    time_t now;
    struct tm timeinfo;

    time(&now);

    localtime_r(&now, &timeinfo);
    return timeinfo;
}

// Get millisecond of current time
int64_t time_client_get_UNIX_timestamp_ms(void)
{
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    return (int64_t)tv_now.tv_sec * 1000LL + (int64_t)tv_now.tv_usec / 1000LL;
}
