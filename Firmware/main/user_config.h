// Configure Timezone
// Fill in POSIX time zone
#define DEFAULT_TZ "GMT"

// Configure WiFi SSID and password
#define DEFAULT_WIFI_SSID "SSID"
#define DEFAULT_WIFI_PASSWORD "Password"

// IMU Synchronisation Settings
#define DEFAULT_POINT_OF_INTEREST 0
#define DEFAULT_MAX_SYNC_ERROR 10
#define DEFAULT_TARGET_IMU_SAMPLE_PERIOD 40
#define DEFAULT_SYNC_SIGNAL_PULSE_WIDTH 320

// On Board IMU settings
#define DEFAULT_ONBOARD_IMU_SAMPLE_PERIOD 10
#define DEFAULT_ONBOARD_IMU_ACCELEROMETER_RANGE ACCE_FS_16G
#define DEFAULT_ONBOARD_IMU_GYROSCOPE_RANGE GYRO_FS_1000DPS
/* Available Options:
    Accelerometer:
        ACCE_FS_2G
        ACCE_FS_4G
        ACCE_FS_8G
        ACCE_FS_16G
    Gyroscope:
        GYRO_FS_250DPS 
        GYRO_FS_500DPS 
        GYRO_FS_1000DPS
        GYRO_FS_2000DPS
*/

