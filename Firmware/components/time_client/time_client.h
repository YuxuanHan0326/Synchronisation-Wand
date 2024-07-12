#ifndef TIME_CLIENT_H
#define TIME_CLIENT_H

void time_client_init(void);
void time_client_deinit(void);
void time_client_set_tz(char *POSIX_tz);
char *time_client_get_time_str(void);
struct tm time_client_get_time(void);
int64_t time_client_get_UNIX_timestamp_ms(void);

#endif