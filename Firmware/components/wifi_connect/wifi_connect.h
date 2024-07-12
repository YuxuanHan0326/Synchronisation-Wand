#ifndef WIFI_CONNECT_H
#define WIFI_CONNECT_H

void wifi_connect_init(void (*wifi_connected_callBack)(void), void (*wifi_disconnected_callBack)(void));
esp_err_t wifi_connect_sta(char *ssid, char *pass, int timeout);
void wifi_connect_ap(const char *ssid, const char *pass);
void wifi_disconnect(void);
esp_netif_ip_info_t wifi_get_ip_info(void);

#endif
